"""
Visualization wrappers for DADCA protocols. These classes inherit from the
core protocol classes and add visualization logic using the
VisualizationController.
"""
from typing import List, Tuple

from gradysim.protocol.messages.telemetry import Telemetry
from gradysim.simulator.extension.visualization_controller import (
    VisualizationController,
)

from common import UAVState
from protocols import (
    BaseStationProtocol,
    EnergyStationProtocol,
    SensorProtocol,
    make_uav_protocol,
)

# Colour palette (RGB 0-255)
COLOR_BASE_STATION = (255, 165, 0)  # orange
COLOR_SENSOR = (0, 200, 255)  # cyan
COLOR_UAV_PATROL = (0, 255, 0)  # green   — normal flight
COLOR_UAV_LOW_BAT = (255, 255, 0)  # yellow  — heading to / waiting at E-Station
COLOR_UAV_CHARGING = (255, 0, 0)  # red     — actively charging
COLOR_ENERGY_STATION = (200, 0, 255)  # purple

VIZ_NODE_SIZE = 4.0

class BaseStationProtocolViz(BaseStationProtocol):
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self._vc.resize_nodes(VIZ_NODE_SIZE)
        self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_BASE_STATION)
            self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_telemetry(self, telemetry: Telemetry) -> None:
        self._vc.paint_node(self.provider.get_id(), COLOR_BASE_STATION)


class SensorProtocolViz(SensorProtocol):
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_SENSOR)
            self.provider.schedule_timer("repaint", self.provider.current_time() + 1)


class EnergyStationProtocolViz(EnergyStationProtocol):
    def initialize(self) -> None:
        super().initialize()
        self._vc = VisualizationController(self)
        self.provider.schedule_timer("repaint", self.provider.current_time() + 1)

    def handle_timer(self, timer: str) -> None:
        super().handle_timer(timer)
        if timer == "repaint":
            self._vc.paint_node(self.provider.get_id(), COLOR_ENERGY_STATION)
            self.provider.schedule_timer("repaint", self.provider.current_time() + 1)


def make_uav_protocol_viz(
    mission_waypoints: List[tuple],
    estation_pos: Tuple[float, float, float],
    departure_delay: float = 0.0,
) -> type:
    BaseUAV = make_uav_protocol(mission_waypoints, estation_pos, departure_delay)
 
    class _UAVProtocolViz(BaseUAV):
        def initialize(self) -> None:
            super().initialize()
            self._vc = VisualizationController(self)
 
        def handle_telemetry(self, telemetry: Telemetry) -> None:
            super().handle_telemetry(telemetry)
            if self.state == UAVState.CHARGING:
                color = COLOR_UAV_CHARGING
            elif self.state in (UAVState.TO_ESTATION, UAVState.WAITING, UAVState.QUERYING):
                color = COLOR_UAV_LOW_BAT
            else:
                color = COLOR_UAV_PATROL
            self._vc.paint_node(self.provider.get_id(), color)
 
    _UAVProtocolViz.__name__ = f"UAVProtocolViz_{id(mission_waypoints)}"
    return _UAVProtocolViz