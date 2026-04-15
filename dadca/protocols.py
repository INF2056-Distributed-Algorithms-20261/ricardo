"""
Core protocol logic for all agent types in the DADCA simulation.
"""
import json
import logging
import math
from typing import Dict, List, Optional, Tuple

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.communication import (
    BroadcastMessageCommand,
    SendMessageCommand,
)
from gradysim.protocol.messages.mobility import (
    GotoCoordsMobilityCommand,
    SetSpeedMobilityCommand,
)
from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.protocol.plugin.mission_mobility import (
    LoopMission,
    MissionMobilityConfiguration,
    MissionMobilityPlugin,
)

from common import (
    BATTERY_CAPACITY,
    BATTERY_DRAIN_RATE,
    BATTERY_LOW_THRESHOLD,
    RECHARGE_RATE,
    SENSOR_PACKET_INTERVAL,
    UAV_SPEED,
    AgentType,
    DADCAMessage,
    EStationMessage,
    UAVState,
)


# ---------------------------------------------------------------------------
# Priority helpers for MEx-Recharge
# ---------------------------------------------------------------------------

def _charge_priority(battery: float, packet_count: int) -> tuple:
    """
    Lower value = higher charging priority.
    Primary key:   battery level (lower battery → charge first).
    Tiebreaker:    -packet_count (more packets → charge first when equal battery).
    """
    return (battery, -packet_count)


# ---------------------------------------------------------------------------
# 2. Sensor Protocol
# ---------------------------------------------------------------------------


class SensorProtocol(IProtocol):
    """Generates one packet every SENSOR_PACKET_INTERVAL seconds and transfers
    all buffered packets to the first UAV that broadcasts a heartbeat within
    radio range."""

    packet_count: int

    def initialize(self) -> None:
        self.packet_count = 0
        self._schedule_packet()

    def _schedule_packet(self) -> None:
        self.provider.schedule_timer(
            "gen", self.provider.current_time() + SENSOR_PACKET_INTERVAL
        )

    def handle_timer(self, timer: str) -> None:
        if timer == "gen":
            self.packet_count += 1
            self._schedule_packet()

    def handle_packet(self, message: str) -> None:
        try:
            msg: DADCAMessage = json.loads(message)
        except json.JSONDecodeError:
            return
        if msg.get("sender_type") == AgentType.UAV.value:
            response: DADCAMessage = {
                "packet_count": self.packet_count,
                "sender_type":  AgentType.SENSOR.value,
                "sender_id":    self.provider.get_id(),
            }
            self.provider.send_communication_command(
                SendMessageCommand(json.dumps(response), msg["sender_id"])
            )
            self.packet_count = 0

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        logging.info(
            f"Sensor {self.provider.get_id()} "
            f"final undelivered packets: {self.packet_count}"
        )


# ---------------------------------------------------------------------------
# 3. Energy Station Protocol
#
# Manages one charging slot.  It is deliberately simple: it grants or denies
# the slot and broadcasts "slot_free" when the charging UAV finishes.
# Priority ordering is fully enforced by the UAVs themselves (see UAV protocol).
# ---------------------------------------------------------------------------

class EnergyStationProtocol(IProtocol):

    def initialize(self) -> None:
        self._charging_uav: Optional[int] = None
        logging.info(f"EnergyStation {self.provider.get_id()} ready")

    def handle_packet(self, message: str) -> None:
        try:
            msg: EStationMessage = json.loads(message)
        except json.JSONDecodeError:
            return
        if "msg_type" not in msg:
            return

        sender = msg["sender_id"]

        if msg["msg_type"] == "request_charge":
            if self._charging_uav is None:
                self._charging_uav = sender
                reply: EStationMessage = {
                    "msg_type":     "ack_charge",
                    "sender_id":    self.provider.get_id(),
                    "battery":      0.0,
                    "packet_count": 0,
                }
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(reply), sender)
                )
                logging.info(f"EStation: slot granted to UAV {sender}")
            else:
                reply_busy: EStationMessage = {
                    "msg_type":     "slot_busy",
                    "sender_id":    self.provider.get_id(),
                    "battery":      0.0,
                    "packet_count": 0,
                }
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(reply_busy), sender)
                )
                logging.info(
                    f"EStation: slot busy (UAV {self._charging_uav}), "
                    f"told UAV {sender}"
                )

        elif msg["msg_type"] == "charge_done":
            if self._charging_uav == sender:
                self._charging_uav = None
                logging.info(f"EStation: UAV {sender} done — slot free")
                free_msg: EStationMessage = {
                    "msg_type":     "slot_free",
                    "sender_id":    self.provider.get_id(),
                    "battery":      0.0,
                    "packet_count": 0,
                }
                self.provider.send_communication_command(
                    BroadcastMessageCommand(json.dumps(free_msg))
                )

    def handle_timer(self, timer: str) -> None:
        pass

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        logging.info(f"EnergyStation {self.provider.get_id()} shutting down")


# ---------------------------------------------------------------------------
# 4. UAV Protocol factory
#
# MEx-Recharge — distributed priority election
# ─────────────────────────────────────────────
# When a UAV reaches the E-Station it enters ANNOUNCING state and broadcasts
# a "want_charge" message containing its (battery, packet_count) priority.
#
# Every UAV that receives a "want_charge" from a peer:
#   • Stores the peer's priority.
#   • Sends back a "want_ack" with its own priority (so the announcer knows
#     all current contenders, including ones already in WAITING state).
#
# After an election window (ANNOUNCE_WINDOW seconds) the announcing UAV
# compares its own priority against all replies it received:
#   • If it has the highest priority (lowest battery, tiebreak most packets)
#     → it sends "request_charge" to the E-Station.
#   • Otherwise → it enters WAITING state and waits for "slot_free" before
#     re-running the election.
#
# A UAV already in WAITING state that receives a new "want_charge" from a
# peer also replies with a "want_ack" so the new announcer gets a complete
# picture of all contenders.
#
# This ensures mutual exclusion without a central coordinator and naturally
# handles any number of simultaneous contenders.
# ---------------------------------------------------------------------------

ANNOUNCE_WINDOW = 3.0   # seconds to collect want_ack replies before deciding

def make_uav_protocol(
    mission_waypoints: List[tuple],
    estation_pos: Tuple[float, float, float],
    departure_delay: float = 0.0,
) -> type:
    """
    Returns a UAV protocol class configured for a specific patrol route and
    E-Station position.  Battery consumption is proportional to distance
    flown (BATTERY_DRAIN_RATE units per metre); when it falls below the
    threshold the UAV diverts to the E-Station, runs the distributed
    MEx-Recharge election, recharges, then resumes patrol.
    """

    class _UAVProtocol(IProtocol):
        _waypoints:       List[tuple]                = mission_waypoints
        _estation_pos:    Tuple[float, float, float] = estation_pos
        _departure_delay: float                      = departure_delay

        def initialize(self) -> None:
            self.packet_count   = 0
            self.battery        = BATTERY_CAPACITY
            self.state          = UAVState.PATROLLING
            self._at_base       = False
            self._last_pos: Optional[Tuple[float, float, float]] = None
            self._estation_id:  Optional[int] = None

            # MEx-Recharge election data
            # Maps peer_id → (battery, packet_count) received during election
            self._peer_priorities: Dict[int, Tuple[float, int]] = {}

            if self._departure_delay > 0:
                self.provider.schedule_timer(
                    "depart", self.provider.current_time() + self._departure_delay
                )
            else:
                self._start_patrol()
            self._send_heartbeat()

        # ── patrol helpers ────────────────────────────────────────────────

        def _start_patrol(self) -> None:
            cfg = MissionMobilityConfiguration(
                loop_mission=LoopMission.REVERSE,
                speed=UAV_SPEED,
            )
            self._mission_plugin = MissionMobilityPlugin(self, cfg)
            self._mission_plugin.start_mission(self._waypoints)

        # ── heartbeat ─────────────────────────────────────────────────────

        def _send_heartbeat(self) -> None:
            if self.state == UAVState.PATROLLING:
                msg: DADCAMessage = {
                    "packet_count": self.packet_count,
                    "sender_type":  AgentType.UAV.value,
                    "sender_id":    self.provider.get_id(),
                }
                self.provider.send_communication_command(
                    BroadcastMessageCommand(json.dumps(msg))
                )
            self.provider.schedule_timer(
                "heartbeat", self.provider.current_time() + 1
            )

        # ── timer handling ────────────────────────────────────────────────

        def handle_timer(self, timer: str) -> None:
            if timer == "heartbeat":
                self._send_heartbeat()

            elif timer == "recharge_tick":
                if self.state == UAVState.CHARGING:
                    self.battery = min(BATTERY_CAPACITY, self.battery + RECHARGE_RATE)
                    logging.info(
                        f"UAV {self.provider.get_id()} charging … "
                        f"{self.battery:.2f}/{BATTERY_CAPACITY:.0f}"
                    )
                    if self.battery >= BATTERY_CAPACITY:
                        self._finish_charging()
                    else:
                        self.provider.schedule_timer(
                            "recharge_tick", self.provider.current_time() + 1
                        )

            elif timer == "election_done":
                # ANNOUNCING window closed — decide based on collected replies
                if self.state == UAVState.ANNOUNCING:
                    self._resolve_election()

            elif timer == "retry_request":
                # Slot was busy after we won the election — retry
                if self.state == UAVState.ANNOUNCING:
                    self._send_request_charge()

            elif timer == "depart":
                logging.info(f"UAV {self.provider.get_id()} departing now")
                self._start_patrol()

        # ── packet handling ───────────────────────────────────────────────

        def handle_packet(self, message: str) -> None:
            try:
                raw = json.loads(message)
            except json.JSONDecodeError:
                return

            # ── DADCA messages ────────────────────────────────────────────
            if "sender_type" in raw:
                sender_type = raw["sender_type"]
                if sender_type == AgentType.SENSOR.value:
                    if self.state == UAVState.PATROLLING:
                        self.packet_count += raw["packet_count"]
                        logging.info(
                            f"UAV {self.provider.get_id()} collected "
                            f"{raw['packet_count']} packets from sensor "
                            f"{raw['sender_id']}. Buffer: {self.packet_count}"
                        )
                elif sender_type == AgentType.BASE_STATION.value:
                    logging.info(
                        f"UAV {self.provider.get_id()} ACK — cleared "
                        f"{self.packet_count} packets"
                    )
                    self.packet_count = 0
                return

            # ── E-Station / peer messages ─────────────────────────────────
            if "msg_type" not in raw:
                return

            mtype  = raw["msg_type"]
            sender = raw["sender_id"]

            # ── E-Station replies ─────────────────────────────────────────
            if mtype == "ack_charge":
                self._estation_id = sender
                self._start_charging()

            elif mtype == "slot_busy":
                self._estation_id = sender
                # Our request arrived while someone else was charging.
                # Re-enter ANNOUNCING so we run a fresh election when the
                # slot becomes free.
                if self.state == UAVState.ANNOUNCING:
                    self.state = UAVState.WAITING
                    logging.info(
                        f"UAV {self.provider.get_id()} slot busy — "
                        f"waiting for slot_free"
                    )

            elif mtype == "slot_free":
                if self.state == UAVState.WAITING:
                    # Re-run the election now that the slot is free
                    self._start_election()

            # ── Peer-to-peer election messages ────────────────────────────
            elif mtype == "want_charge":
                # A peer wants to charge — record its priority
                peer_battery      = raw["battery"]
                peer_packets      = raw["packet_count"]
                self._peer_priorities[sender] = (peer_battery, peer_packets)

                # Always reply with our own priority so the announcer gets a
                # full picture, regardless of our current state
                ack: EStationMessage = {
                    "msg_type":     "want_ack",
                    "sender_id":    self.provider.get_id(),
                    "battery":      self.battery,
                    "packet_count": self.packet_count,
                }
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(ack), sender)
                )

            elif mtype == "want_ack":
                # A peer replied to our want_charge broadcast
                if self.state == UAVState.ANNOUNCING:
                    peer_battery = raw["battery"]
                    peer_packets = raw["packet_count"]
                    self._peer_priorities[sender] = (peer_battery, peer_packets)

        # ── telemetry: battery drain + delivery + divert trigger ──────────

        def handle_telemetry(self, telemetry: Telemetry) -> None:
            pos = telemetry.current_position

            # ── drain battery by distance flown ───────────────────────────
            if self._last_pos is not None:
                dx = pos[0] - self._last_pos[0]
                dy = pos[1] - self._last_pos[1]
                dz = pos[2] - self._last_pos[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                self.battery = max(0.0, self.battery - BATTERY_DRAIN_RATE * dist)
            self._last_pos = pos

            # ── low-battery divert (only while patrolling) ────────────────
            if self.state == UAVState.PATROLLING:
                if self.battery <= BATTERY_LOW_THRESHOLD * BATTERY_CAPACITY:
                    self._divert_to_estation()
                    return

            # ── delivery to Base Station (only while patrolling) ──────────
            if self.state == UAVState.PATROLLING and hasattr(self, "_mission_plugin"):
                at_base_now = (
                    self._mission_plugin.current_waypoint == 0
                    and self._mission_plugin.is_reversed
                )
                if at_base_now and not self._at_base and self.packet_count > 0:
                    self._deliver_to_base()
                self._at_base = at_base_now

            # ── arrival at E-Station ──────────────────────────────────────
            if self.state == UAVState.TO_ESTATION:
                ex, ey, ez = self._estation_pos
                dist_to_e = math.sqrt(
                    (pos[0]-ex)**2 + (pos[1]-ey)**2 + (pos[2]-ez)**2
                )
                if dist_to_e < 2.0:
                    self._start_election()

        # ── private helpers ───────────────────────────────────────────────

        def _deliver_to_base(self) -> None:
            msg: DADCAMessage = {
                "packet_count": self.packet_count,
                "sender_type":  AgentType.UAV.value,
                "sender_id":    self.provider.get_id(),
            }
            self.provider.send_communication_command(
                BroadcastMessageCommand(json.dumps(msg))
            )
            logging.info(
                f"UAV {self.provider.get_id()} delivering "
                f"{self.packet_count} packets to base station"
            )

        def _divert_to_estation(self) -> None:
            logging.info(
                f"UAV {self.provider.get_id()} battery low "
                f"({self.battery:.2f}), diverting to E-Station"
            )
            self.state = UAVState.TO_ESTATION
            if hasattr(self, "_mission_plugin"):
                self._mission_plugin.stop_mission()
            ex, ey, ez = self._estation_pos
            self.provider.send_mobility_command(
                GotoCoordsMobilityCommand(ex, ey, ez)
            )
            self.provider.send_mobility_command(
                SetSpeedMobilityCommand(UAV_SPEED)
            )

        # ── MEx-Recharge election ─────────────────────────────────────────

        def _start_election(self) -> None:
            """
            Begin a distributed priority election.

            The UAV broadcasts "want_charge" with its own battery and packet
            count, then waits ANNOUNCE_WINDOW seconds for peers to reply with
            "want_ack".  After the window closes, _resolve_election() picks
            the winner.
            """
            self.state = UAVState.ANNOUNCING
            self._peer_priorities = {}   # fresh slate for this election round

            announce: EStationMessage = {
                "msg_type":     "want_charge",
                "sender_id":    self.provider.get_id(),
                "battery":      self.battery,
                "packet_count": self.packet_count,
            }
            self.provider.send_communication_command(
                BroadcastMessageCommand(json.dumps(announce))
            )
            logging.info(
                f"UAV {self.provider.get_id()} started election "
                f"(battery={self.battery:.2f}, pkts={self.packet_count})"
            )
            self.provider.schedule_timer(
                "election_done",
                self.provider.current_time() + ANNOUNCE_WINDOW,
            )

        def _resolve_election(self) -> None:
            """
            Compare own priority against all peers that replied.
            Winner → send request_charge.
            Loser  → enter WAITING.
            """
            my_id       = self.provider.get_id()
            my_priority = _charge_priority(self.battery, self.packet_count)

            winner    = True
            winner_id = my_id

            for peer_id, (p_bat, p_pkts) in self._peer_priorities.items():
                peer_priority = _charge_priority(p_bat, p_pkts)
                if peer_priority < my_priority:
                    # Peer has higher charging priority
                    winner    = False
                    winner_id = peer_id
                    break
                elif peer_priority == my_priority and peer_id < my_id:
                    # Exact tie: lower node ID wins (deterministic tiebreaker)
                    winner    = False
                    winner_id = peer_id
                    break

            if winner:
                logging.info(
                    f"UAV {my_id} won election — requesting charge slot "
                    f"(battery={self.battery:.2f}, pkts={self.packet_count})"
                )
                self._send_request_charge()
            else:
                logging.info(
                    f"UAV {my_id} lost election to UAV {winner_id} — waiting"
                )
                self.state = UAVState.WAITING

        def _send_request_charge(self) -> None:
            """Send request_charge to the E-Station (directed if ID known)."""
            # Stay in ANNOUNCING so we can handle ack_charge / slot_busy
            req: EStationMessage = {
                "msg_type":     "request_charge",
                "sender_id":    self.provider.get_id(),
                "battery":      self.battery,
                "packet_count": self.packet_count,
            }
            if self._estation_id is not None:
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(req), self._estation_id)
                )
            else:
                self.provider.send_communication_command(
                    BroadcastMessageCommand(json.dumps(req))
                )

        def _start_charging(self) -> None:
            self.state = UAVState.CHARGING
            logging.info(
                f"UAV {self.provider.get_id()} started charging "
                f"(battery={self.battery:.2f})"
            )
            self.provider.schedule_timer(
                "recharge_tick", self.provider.current_time() + 1
            )

        def _finish_charging(self) -> None:
            self.battery = BATTERY_CAPACITY
            logging.info(
                f"UAV {self.provider.get_id()} fully charged — resuming patrol"
            )
            done: EStationMessage = {
                "msg_type":     "charge_done",
                "sender_id":    self.provider.get_id(),
                "battery":      self.battery,
                "packet_count": self.packet_count,
            }
            if self._estation_id is not None:
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(done), self._estation_id)
                )
            else:
                self.provider.send_communication_command(
                    BroadcastMessageCommand(json.dumps(done))
                )
            self._at_base  = False
            self.state     = UAVState.PATROLLING
            self._start_patrol()

        def finish(self) -> None:
            logging.info(
                f"UAV {self.provider.get_id()} done | "
                f"buffer={self.packet_count} | battery={self.battery:.2f}"
            )

    _UAVProtocol.__name__ = f"UAVProtocol_{id(mission_waypoints)}"
    return _UAVProtocol


# ---------------------------------------------------------------------------
# 5. Base Station Protocol
# ---------------------------------------------------------------------------


class BaseStationProtocol(IProtocol):
    total_received: int

    def initialize(self) -> None:
        self.total_received = 0

    def handle_packet(self, message: str) -> None:
        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return
        if (
            msg.get("sender_type") == AgentType.UAV.value
            and msg.get("packet_count", 0) > 0
        ):
            self.total_received += msg["packet_count"]
            logging.info(
                f"Base Station received {msg['packet_count']} packets from "
                f"UAV {msg['sender_id']}. Total: {self.total_received}"
            )
            response: DADCAMessage = {
                "packet_count": 0,
                "sender_type":  AgentType.BASE_STATION.value,
                "sender_id":    self.provider.get_id(),
            }
            self.provider.send_communication_command(
                SendMessageCommand(json.dumps(response), msg["sender_id"])
            )

    def handle_timer(self, timer: str) -> None:
        pass

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def finish(self) -> None:
        logging.info(f"Base Station total packets received: {self.total_received}")