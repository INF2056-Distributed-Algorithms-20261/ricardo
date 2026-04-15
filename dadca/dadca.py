"""
========================= Enunciado ===========================
Enunciado do exercício #2 (para 25/03)

a) Usando o simulador GrADyS SIM NG, implementar o DADCA para M Sensores
(S_1,..,S_z) e N drones coletando e transportando dados ao longo da rota
dos sensores para a BaseStation.

b) implementar uma variável bateria (de capacidade igual) em cada um dos
UAVs, cuja carga vai sendo consumida a cada X metros percorridos pelo
UAV. Para recarregar  bateria, o UAV precisa voar para uma estação de
recarga, EnergyStation (E-Station).
O tempo de recarga é sempre igual para todos os UAVs, mas a E-Station só
pode carregar um UAV por vez.

c) Simular uma EnergyStation (E-Station) posicionada equidistante da
BaseStation e S_z formando um triangulo equilátero com estes.
Ignorar potenciais problemas de colisão entre UAVs nas proximidades da
E-Station (quando vários UAVs se aproximam ou afastam), mas o tempo de
voo do UAV da trajetória de sensores para a E-Station pode variar.

d) quando dois ou mais UAVs chegam perto da E-Station (já no raio de
cobertura de seus radios e podendo interagir), executam um algoritmo de
Escalonamento e Recarga em Exclusão Mutua (MEx-Recharge). Esse algoritmo
prioriza o pouso na E-Station do UAV que está com o menor nível e
bateria, depois aquele com 2o. menor nível etc.
Em caso de empate, ganha acesso aquele UAV que tiver carregando mais
dados coletados em seu buffer.
===================
"""
import logging

from gradysim.simulator.handler.communication import (
    CommunicationHandler,
    CommunicationMedium,
)
from gradysim.simulator.handler.mobility import MobilityHandler
from gradysim.simulator.handler.timer import TimerHandler
from gradysim.simulator.handler.visualization import (
    VisualizationHandler, VisualizationConfiguration,
)
from gradysim.simulator.simulation import SimulationBuilder, SimulationConfiguration

from common import COMM_RANGE, UAV_STAGGER_DELAY
from utils import equilateral_third_point
from visualization import (
    BaseStationProtocolViz,
    EnergyStationProtocolViz,
    SensorProtocolViz,
    make_uav_protocol_viz,
)


def main():
    config  = SimulationConfiguration(duration=300)
    builder = SimulationBuilder(config)

    # ── Positions ─────────────────────────────────────────────────────────
    base_pos   = (0, 0, 0)
    flight_alt = 20.0

    sensor_positions = [
        (0,  30, 0),
        (0,  60, 0),
        (0,  90, 0),
    ]

    # E-Station: equilateral triangle with base_pos and last sensor,
    # sitting off to the side so it is not on the sensor line.
    ex, ey = equilateral_third_point(
        (base_pos[0], base_pos[1]),
        (sensor_positions[-1][0], sensor_positions[-1][1]),
    )
    estation_ground = (ex, ey, 0.0)
    estation_hover  = (ex, ey, flight_alt)   # where UAVs fly to when recharging

    logging.info(f"E-Station ground position: ({ex:.1f}, {ey:.1f})")

    # ── Scene bounds (include E-Station) ──────────────────────────────────
    padding = 50
    all_x = [p[0] for p in sensor_positions] + [base_pos[0], ex]
    all_y = [p[1] for p in sensor_positions] + [base_pos[1], ey]
    x_range = (min(all_x) - padding, max(all_x) + padding)
    y_range = (min(all_y) - padding, max(all_y) + padding)
    z_range = (0, flight_alt + padding)

    viz_config = VisualizationConfiguration(
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        open_browser=False,   # set True to auto-open the browser
    )

    # ── Nodes ─────────────────────────────────────────────────────────────
    builder.add_node(BaseStationProtocolViz, base_pos)
    builder.add_node(EnergyStationProtocolViz, estation_ground)

    for pos in sensor_positions:
        builder.add_node(SensorProtocolViz, pos)

    for i, sensor_pos in enumerate(sensor_positions):
        waypoints = [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_pos[0], sensor_pos[1], flight_alt),
        ]
        
        # Stagger departures by UAV_STAGGER_DELAY seconds so UAVs naturally
        # desynchronize and never all hit the low-battery threshold together.
        UAVClass = make_uav_protocol_viz(waypoints, estation_hover, departure_delay=i * UAV_STAGGER_DELAY)
        builder.add_node(UAVClass, base_pos)

    # ── Handlers ──────────────────────────────────────────────────────────
    builder.add_handler(TimerHandler())
    builder.add_handler(MobilityHandler())
    builder.add_handler(
        CommunicationHandler(CommunicationMedium(transmission_range=COMM_RANGE))
    )
    builder.add_handler(VisualizationHandler(viz_config))

    simulation = builder.build()
    simulation.start_simulation()


if __name__ == "__main__":
    main()