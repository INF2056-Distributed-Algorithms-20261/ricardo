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

Layout
------
Base Station (G-Station):  (0, 0, 0)
7 Sensors:  (50,0,0), (100,0,0), (150,0,0), (250,0,0),
            (300,0,0), (350,0,0), (450,0,0)
E-Station:  (225, 389.71, 0)   [equilateral triangle vertex]
5 UAVs departing from (0, 0, 0), flight altitude 15 m, speed 3 m/s

UAV route assignment (base ↔ furthest sensor in segment):
  UAV 0: base ↔ sensor 0  (50,  0)
  UAV 1: base ↔ sensor 1  (100, 0)   [covers sensors 1–2]
  UAV 2: base ↔ sensor 3  (250, 0)   [covers sensors 3]
  UAV 3: base ↔ sensor 4  (350, 0)   [covers sensors 4–5]
  UAV 4: base ↔ sensor 6  (450, 0)   [covers sensor 6]
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

from common import COMM_RANGE, UAV_STAGGER_DELAY, SIMULATION_DURATION
from visualization import (
    BaseStationProtocolViz,
    EnergyStationProtocolViz,
    SensorProtocolViz,
    make_uav_protocol_viz,
)


def main():
    config  = SimulationConfiguration(duration=SIMULATION_DURATION)
    builder = SimulationBuilder(config)

    # ── Positions ─────────────────────────────────────────────────────────
    base_pos   = (0, 0, 0)
    flight_alt = 15.0          # UAVs fly at 15 m above ground

    # 7 sensors along the x-axis
    sensor_positions = [
        (50,  0, 0),
        (100, 0, 0),
        (150, 0, 0),
        (250, 0, 0),
        (300, 0, 0),
        (350, 0, 0),
        (450, 0, 0),
    ]

    # E-Station at the equilateral triangle vertex opposite the base→last-sensor edge.
    # Provided explicitly: (225, 389.71, 0)
    estation_ground = (225.0, 389.71, 0.0)
    estation_hover  = (225.0, 389.71, flight_alt)   # UAVs hover here when recharging

    logging.info(
        f"E-Station position: ({estation_ground[0]:.1f}, {estation_ground[1]:.1f})"
    )

    # ── Scene bounds ───────────────────────────────────────────────────────
    padding = 200
    all_x = [p[0] for p in sensor_positions] + [base_pos[0], estation_ground[0]]
    all_y = [p[1] for p in sensor_positions] + [base_pos[1], estation_ground[1]]
    x_range = (min(all_x) - padding, max(all_x) + padding)
    y_range = (min(all_y) - padding, max(all_y) + padding)
    z_range = (0, flight_alt + padding)

    viz_config = VisualizationConfiguration(
        x_range=x_range,
        y_range=y_range,
        z_range=z_range,
        open_browser=False,   # set True to auto-open the browser
    )

    # ── Static nodes ───────────────────────────────────────────────────────
    builder.add_node(BaseStationProtocolViz, base_pos)
    builder.add_node(EnergyStationProtocolViz, estation_ground)

    for pos in sensor_positions:
        builder.add_node(SensorProtocolViz, pos)

    # ── UAV routes ─────────────────────────────────────────────────────────
    # 5 UAVs; each patrols base ↔ the far end of its assigned sensor segment.
    # All waypoints are expressed at flight altitude.
    #
    # Sensor index map:
    #   0 → (50,0)   1 → (100,0)   2 → (150,0)   3 → (250,0)
    #   4 → (300,0)  5 → (350,0)   6 → (450,0)
    #
    # Assignment rationale: spread 5 UAVs across 7 sensors with slight
    # overlap — each UAV's heartbeat covers sensors within COMM_RANGE (20 m)
    # of its waypoints.
    uav_routes = [
        # (base hover point, far waypoint)
        [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_positions[0][0], sensor_positions[0][1], flight_alt),   # 50
        ],
        [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_positions[2][0], sensor_positions[2][1], flight_alt),   # 150
        ],
        [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_positions[3][0], sensor_positions[3][1], flight_alt),   # 250
        ],
        [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_positions[5][0], sensor_positions[5][1], flight_alt),   # 350
        ],
        [
            (base_pos[0], base_pos[1], flight_alt),
            (sensor_positions[6][0], sensor_positions[6][1], flight_alt),   # 450
        ],
    ]

    for i, waypoints in enumerate(uav_routes):
        UAVClass = make_uav_protocol_viz(
            waypoints,
            estation_hover,
            departure_delay=i * UAV_STAGGER_DELAY,
        )
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