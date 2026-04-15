"""
Common definitions for the DADCA simulation: constants, agent types,
message schemas, and state machines.
"""
import enum
from typing import TypedDict

# ---------------------------------------------------------------------------
# Constants — tweak these to change simulation behaviour
# ---------------------------------------------------------------------------

BATTERY_CAPACITY      = 1000.0   # total metres of flight before empty
BATTERY_LOW_THRESHOLD = 0.25     # go recharge when battery drops below this fraction
RECHARGE_TIME         = 30.0     # seconds to fully recharge
RECHARGE_RATE         = BATTERY_CAPACITY / RECHARGE_TIME   # battery units / second
UAV_SPEED             = 10.0     # m/s

COMM_RANGE            = 60.0     # radio range (m) — wide enough for E-Station negotiation

# Stagger UAV departures so they never hit the low-battery threshold at the
# same time.  A spacing of ~15 s at 10 m/s spreads them ~150 m apart along
# their routes, which is enough to desynchronize the first recharge cycle.
UAV_STAGGER_DELAY     = 20.0     # seconds between each UAV's departure

# ---------------------------------------------------------------------------
# 1. Agent types and message schemas
# ---------------------------------------------------------------------------

class AgentType(enum.Enum):
    SENSOR         = 0
    UAV            = 1
    BASE_STATION   = 2
    ENERGY_STATION = 3


class DADCAMessage(TypedDict):
    packet_count: int
    sender_type:  int
    sender_id:    int


class EStationMessage(TypedDict):
    """
    Protocol between UAVs and the Energy Station.

    msg_type values
    ---------------
    "request_charge"  UAV → E-Station : I need to charge
    "ack_charge"      E-Station → UAV : slot granted, start charging
    "slot_busy"       E-Station → UAV : slot taken, retry later
    "charge_done"     UAV → E-Station : I'm done, releasing the slot
    "slot_free"       E-Station → all : slot is free (broadcast)
    """
    msg_type:     str
    sender_id:    int
    battery:      float
    packet_count: int


# ---------------------------------------------------------------------------
# UAV state machine
# ---------------------------------------------------------------------------

class UAVState(enum.Enum):
    PATROLLING  = "patrolling"    # Normal DADCA shuttle
    TO_ESTATION = "to_estation"   # Flying toward E-Station
    QUERYING    = "querying"      # Asked charging UAV for its battery; deciding
    WAITING     = "waiting"       # Committed to waiting for the slot
    CHARGING    = "charging"      # Actively charging
    RETURNING   = "returning"     # Flying back to resume patrol (transient)
 