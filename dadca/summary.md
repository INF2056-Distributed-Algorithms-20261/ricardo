# DADCA Simulation with Battery Management

A multi-UAV data collection simulation built on [GrADyS-SIM NextGen](https://github.com/Project-GrADyS/gradys-sim-nextgen).

---

## Overview

The system models a network of UAVs that shuttle between a Base Station and a line of ground sensors, collecting data packets and delivering them back. Because UAVs have finite batteries, a shared Energy Station handles recharging — one UAV at a time — using a negotiated mutual exclusion protocol.

---

## Agents

The simulation contains five types of nodes:

| Agent | Colour | Role |
|---|---|---|
| Base Station | 🟠 Orange | Receives all collected data; the sink of the network |
| Sensor (×3) | 🩵 Cyan | Generates one data packet per second; uploads to any nearby UAV |
| UAV (×3) | 🟢/🟡/🔴 | Collects packets from sensors and delivers them to the Base Station |
| Energy Station | 🟣 Purple | Single-slot charging station; manages mutual exclusion |

### UAV colour key

| Colour | Meaning |
|---|---|
| 🟢 Green | Patrolling normally |
| 🟡 Yellow | Battery low — flying to, querying, or waiting at the E-Station |
| 🔴 Red | Actively charging |

---

## Spatial Layout

```
Sensor 3  (0, 90)
    |
Sensor 2  (0, 60)       E-Station  (~78, 45)
    |
Sensor 1  (0, 30)
    |
Base Station  (0, 0)          
```

The three sensors are placed in a straight vertical line north of the Base Station at 30 m intervals. Each UAV patrols one segment of that line at a flight altitude of 20 m.

The Energy Station is placed at the third vertex of an equilateral triangle whose base connects the Base Station `(0, 0)` and the last sensor `(0, 90)`. This puts it clearly off to the side of the sensor line — roughly at `(78, 45)` — so UAVs must make a deliberate detour to reach it rather than passing through it incidentally.

---

## DADCA Protocol

DADCA (Distributed Aerial Data Collection Algorithm) works as follows:

1. Each UAV follows a fixed two-waypoint route — Base Station hover point ↔ assigned sensor — looping back and forth indefinitely using `LoopMission.REVERSE`.
2. Every second the UAV broadcasts a **heartbeat** containing its ID. Any sensor within radio range responds by uploading all its buffered packets and resetting its own counter.
3. When the UAV reaches the Base Station end of its route it **delivers** its buffered packets via broadcast. The Base Station acknowledges with an ACK and the UAV clears its buffer.

Because every UAV broadcasts, sensors opportunistically upload to whichever drone passes closest, not just the one "assigned" to their route.

---

## Battery System

Each UAV starts with a full battery of **1000 m** of flight range. Battery is drained continuously in `handle_telemetry` by subtracting the Euclidean distance flown since the last telemetry tick.

When battery drops below **25%** (250 m remaining) while patrolling, the UAV:

1. Stops its patrol mission.
2. Flies directly to the Energy Station hover point at `(ex, ey, 20 m)`.
3. Enters the recharging negotiation protocol described below.

After a full recharge the UAV restarts its patrol from the Base Station waypoint.

### Tunable constants

| Constant | Default | Meaning |
|---|---|---|
| `BATTERY_CAPACITY` | 1000 m | Total flight range on a full charge |
| `BATTERY_LOW_THRESHOLD` | 0.25 | Fraction of capacity at which diversion triggers |
| `RECHARGE_TIME` | 30 s | Time to go from empty to full |
| `UAV_SPEED` | 10 m/s | Flight speed for all UAVs |
| `COMM_RANGE` | 60 m | Radio range for all messages |

---

## Recharge Protocol

The Energy Station has a single charging slot. When multiple UAVs need to recharge, they negotiate mutual exclusion using a four-message protocol.

### Messages

| Message | Direction | Meaning |
|---|---|---|
| `request_charge` | UAV → E-Station | Request the charging slot |
| `ack_charge` | E-Station → UAV | Slot granted; begin charging |
| `slot_busy` | E-Station → UAV | Slot occupied; here is the charging UAV's ID |
| `charge_done` | UAV → E-Station | Charging complete; releasing the slot |
| `slot_free` | E-Station → all | Slot is now free (broadcast) |
| `status_query` | UAV → UAV | What is your current battery level? |
| `status_reply` | UAV → UAV | My current battery level is X |

### State machine

```
PATROLLING
    │  battery < 25%
    ▼
TO_ESTATION
    │  arrived (dist < 2 m)
    ▼
  ┌─────────────────────────────────┐
  │        request_charge           │
  └──────────┬──────────────────────┘
             │
     ┌───────┴────────┐
     │ ack_charge     │ slot_busy
     ▼                ▼
  CHARGING         QUERYING ──── status_reply ──► decision
     │                │                               │
     │                │ query_timeout              ┌──┴──────────────────┐
     │                ▼                            │ charger almost done │
     │             WAITING ◄───────────────────────┤   OR critically low │
     │                │ slot_free                  └─────────────────────┘
     │                ▼                                    │ otherwise
     │           request_charge                           ▼
     │                                              PATROLLING (resume)
     │  battery full
     ▼
  charge_done → PATROLLING
```

### Smart waiting decision (QUERYING state)

When told the slot is busy, a UAV does not blindly queue. It first sends a `status_query` directly to the charging UAV. Based on the reply it makes a cost/benefit decision:

- **Wait** if the charger's battery is already above 50% (it will finish soon) or if its own battery is critically low (below 15%).
- **Return to patrol** otherwise — the charger still has a long way to go and the waiting UAV has enough battery to do useful work in the meantime.

This prevents all three UAVs from clustering at the E-Station simultaneously, keeping sensors covered while recharging happens.

### Directed vs broadcast messages

Once a UAV has learned the E-Station's node ID (from any `ack_charge` or `slot_busy` reply), all subsequent `request_charge` and `charge_done` messages are sent as **directed** unicast rather than broadcast. This eliminates a race condition where a broadcast could be processed by the E-Station in an ambiguous order relative to a concurrent `charge_done`, which previously caused a UAV to remain stuck in the WAITING state indefinitely.

---

## Visualization

The simulation connects to the [GrADyS visualization tool](https://project-gradys.github.io/gradys-sim-nextgen-visualization/) via WebSocket on port 5678. Open that URL in a browser while the simulation is running to see the 3D view.

Node colours are updated continuously via `handle_telemetry` (for moving nodes) and a 1-second repaint timer (for stationary nodes), so the browser picks up the correct colours whenever it connects, even mid-simulation.

To auto-open the browser when the simulation starts, set `open_browser=True` in `VisualizationConfiguration` inside `main()`.

---

## Running

```bash
pip install gradysim
python dadca.py
```