"""
Core protocol logic for all agent types in the DADCA simulation.
"""
import json
import logging
import math
from typing import List, Optional, Tuple

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
    BATTERY_LOW_THRESHOLD,
    RECHARGE_RATE,
    UAV_SPEED,
    AgentType,
    DADCAMessage,
    EStationMessage,
    UAVState,
)


# ---------------------------------------------------------------------------
# 2. Sensor Protocol
# ---------------------------------------------------------------------------


class SensorProtocol(IProtocol):
    """Generates one packet per second and transfers all buffered packets
    to the first UAV that broadcasts a heartbeat within radio range."""

    packet_count: int

    def initialize(self) -> None:
        self.packet_count = 0
        self._schedule_packet()

    def _schedule_packet(self) -> None:
        self.provider.schedule_timer("gen", self.provider.current_time() + 1)

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
                "sender_type": AgentType.SENSOR.value,
                "sender_id": self.provider.get_id(),
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
# Manages one charging slot with a simple request / grant / release protocol.
# UAVs compete for the slot; the E-Station is stateless w.r.t. priority —
# the MEx-Recharge priority ordering (part d) is enforced by the UAVs
# themselves before they send "request_charge".
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
                    "msg_type":        "ack_charge",
                    "sender_id":       self.provider.get_id(),
                    "battery":         0.0,
                    "packet_count":    0,
                    "charging_uav_id": 0,
                }
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(reply), sender)
                )
                logging.info(f"EStation: slot granted to UAV {sender}")
            else:
                # Tell the requester *who* is charging so it can query that UAV
                reply_busy: EStationMessage = {
                    "msg_type":        "slot_busy",
                    "sender_id":       self.provider.get_id(),
                    "battery":         0.0,
                    "packet_count":    0,
                    "charging_uav_id": self._charging_uav,
                }
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(reply_busy), sender)
                )
 
        elif msg["msg_type"] == "charge_done":
            if self._charging_uav == sender:
                self._charging_uav = None
                logging.info(f"EStation: UAV {sender} done — slot free")
                free_msg: EStationMessage = {
                    "msg_type":        "slot_free",
                    "sender_id":       self.provider.get_id(),
                    "battery":         0.0,
                    "packet_count":    0,
                    "charging_uav_id": 0,
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
# ---------------------------------------------------------------------------
 
def make_uav_protocol(
    mission_waypoints: List[tuple],
    estation_pos: Tuple[float, float, float],
    departure_delay: float = 0.0,
) -> type:
    """
    Returns a UAV protocol class configured for a specific patrol route and
    E-Station position.  Battery consumption is proportional to distance
    flown; when it falls below the threshold the UAV diverts to the E-Station,
    queues for the single charging slot, recharges, then resumes patrol.
 
    departure_delay — seconds to wait at the base before starting the first
    patrol.  Staggering UAVs by ~15 s prevents them from all hitting the
    low-battery threshold simultaneously on the first cycle.
    """
 
    class _UAVProtocol(IProtocol):
        # Baked-in class-level config (safe because each call creates a new class)
        _waypoints:       List[tuple]               = mission_waypoints
        _estation_pos:    Tuple[float, float, float] = estation_pos
        _departure_delay: float                      = departure_delay
 
        def initialize(self) -> None:
            self.packet_count   = 0
            self.battery        = BATTERY_CAPACITY
            self.state          = UAVState.PATROLLING
            self._at_base       = False
            self._last_pos: Optional[Tuple[float, float, float]] = None
            self._estation_id: Optional[int] = None   # learned on first contact
 
            if self._departure_delay > 0:
                # Sit at the base until the delay expires, then start patrol
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
                        f"{self.battery:.0f}/{BATTERY_CAPACITY:.0f}"
                    )
                    if self.battery >= BATTERY_CAPACITY:
                        self._finish_charging()
                    else:
                        self.provider.schedule_timer(
                            "recharge_tick", self.provider.current_time() + 1
                        )
 
            elif timer == "retry_charge":
                # Only retry if we haven't already resolved the situation
                if self.state == UAVState.WAITING:
                    self._request_charge_slot()
 
            elif timer == "depart":
                # Staggered start: begin patrol after the departure delay
                logging.info(f"UAV {self.provider.get_id()} departing now")
                self._start_patrol()
 
            elif timer == "query_timeout":
                # No reply arrived from the charging UAV — just wait for slot_free
                if self.state == UAVState.QUERYING:
                    self.state = UAVState.WAITING
                    logging.info(
                        f"UAV {self.provider.get_id()} query timed out, "
                        f"waiting for slot_free"
                    )
 
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
 
            mtype    = raw["msg_type"]
            sender   = raw["sender_id"]
 
            if mtype == "ack_charge":
                self._estation_id = sender
                self._start_charging()
 
            elif mtype == "slot_busy":
                # Learn the E-Station's ID from its reply
                self._estation_id = sender
                charging_uav = raw.get("charging_uav_id", 0)
                if charging_uav and charging_uav != self.provider.get_id():
                    # Ask the charging UAV how full its battery is so we can
                    # decide whether to wait here or go back and patrol a bit more
                    self.state = UAVState.QUERYING
                    query: EStationMessage = {
                        "msg_type":        "status_query",
                        "sender_id":       self.provider.get_id(),
                        "battery":         self.battery,
                        "packet_count":    self.packet_count,
                        "charging_uav_id": 0,
                    }
                    self.provider.send_communication_command(
                        SendMessageCommand(json.dumps(query), charging_uav)
                    )
                    # If no reply within 5 s just wait
                    self.provider.schedule_timer(
                        "query_timeout", self.provider.current_time() + 5
                    )
                else:
                    # Couldn't identify the charging UAV — wait for slot_free
                    self.state = UAVState.WAITING
                    self.provider.schedule_timer(
                        "retry_charge", self.provider.current_time() + 5
                    )
 
            elif mtype == "slot_free":
                if self.state == UAVState.WAITING:
                    self._request_charge_slot()
 
            elif mtype == "status_query":
                # A waiting UAV is asking about our battery — reply honestly
                if self.state == UAVState.CHARGING:
                    reply: EStationMessage = {
                        "msg_type":        "status_reply",
                        "sender_id":       self.provider.get_id(),
                        "battery":         self.battery,
                        "packet_count":    self.packet_count,
                        "charging_uav_id": 0,
                    }
                    self.provider.send_communication_command(
                        SendMessageCommand(json.dumps(reply), sender)
                    )
 
            elif mtype == "status_reply":
                # We are in QUERYING state. Decide: wait here or go back?
                if self.state != UAVState.QUERYING:
                    return
                charger_battery = raw["battery"]
                # Cancel the query timeout since we got a reply
                self.provider.cancel_timer("query_timeout")
 
                # If the charger is more than halfway done (battery > 50%),
                # it will finish soon — worth waiting here.
                # If the charger still has a long way to go AND our own battery
                # is not critically low (below 15%), go back and patrol a bit
                # more rather than idling at the E-Station.
                critically_low     = self.battery < 0.15 * BATTERY_CAPACITY
                charger_almost_done = charger_battery > 0.50 * BATTERY_CAPACITY
 
                if charger_almost_done or critically_low:
                    self.state = UAVState.WAITING
                    logging.info(
                        f"UAV {self.provider.get_id()} will wait "
                        f"(charger at {charger_battery:.0f}, "
                        f"my battery {self.battery:.0f})"
                    )
                else:
                    # Go back to patrol; we'll come back when battery is lower
                    logging.info(
                        f"UAV {self.provider.get_id()} returning to patrol — "
                        f"charger at {charger_battery:.0f}, "
                        f"my battery {self.battery:.0f} still OK"
                    )
                    self.state = UAVState.PATROLLING
                    self._start_patrol()
 
        # ── telemetry: battery drain + delivery + divert trigger ──────────
 
        def handle_telemetry(self, telemetry: Telemetry) -> None:
            pos = telemetry.current_position
 
            # ── drain battery by distance flown ───────────────────────────
            if self._last_pos is not None:
                dx = pos[0] - self._last_pos[0]
                dy = pos[1] - self._last_pos[1]
                dz = pos[2] - self._last_pos[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                self.battery = max(0.0, self.battery - dist)
            self._last_pos = pos
 
            # ── low-battery divert (only while patrolling) ────────────────
            if self.state == UAVState.PATROLLING:
                if self.battery <= BATTERY_LOW_THRESHOLD * BATTERY_CAPACITY:
                    self._divert_to_estation()
                    return
 
            # ── delivery to Base Station (only while patrolling) ──────────
            # Guard: _mission_plugin doesn't exist until _start_patrol() is
            # called, which is deferred for staggered UAVs until "depart" fires.
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
                    self._request_charge_slot()
 
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
                f"({self.battery:.0f}), diverting to E-Station"
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
 
        def _request_charge_slot(self) -> None:
            self.state = UAVState.WAITING
            req: EStationMessage = {
                "msg_type":        "request_charge",
                "sender_id":       self.provider.get_id(),
                "battery":         self.battery,
                "packet_count":    self.packet_count,
                "charging_uav_id": 0,
            }
            # Use a directed message if we know the E-Station's ID;
            # fall back to broadcast on first contact.
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
                f"(battery={self.battery:.0f})"
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
                "msg_type":        "charge_done",
                "sender_id":       self.provider.get_id(),
                "battery":         self.battery,
                "packet_count":    self.packet_count,
                "charging_uav_id": 0,
            }
            # Directed to E-Station (we always know its ID by now)
            if self._estation_id is not None:
                self.provider.send_communication_command(
                    SendMessageCommand(json.dumps(done), self._estation_id)
                )
            else:
                self.provider.send_communication_command(
                    BroadcastMessageCommand(json.dumps(done))
                )
            self._at_base = False
            self.state    = UAVState.PATROLLING
            self._start_patrol()
 
        def finish(self) -> None:
            logging.info(
                f"UAV {self.provider.get_id()} done | "
                f"buffer={self.packet_count} | battery={self.battery:.0f}"
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
                "sender_type": AgentType.BASE_STATION.value,
                "sender_id": self.provider.get_id(),
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