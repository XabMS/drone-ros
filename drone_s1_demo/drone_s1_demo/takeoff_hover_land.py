"""Hito S1 (DOC-10): despegue, estacionario y aterrizaje del x500 desde ROS 2.

Nodo de demostración, desechable. No forma parte del software de misión (IDAL C);
sirve para validar la cadena ROS 2 -> uXRCE-DDS -> PX4 SITL -> Gazebo.

Secuencia:
  STREAM  -> envía consignas Offboard durante 1 s (PX4 lo exige antes de cambiar de modo)
  ARMING  -> pide modo Offboard y armado; espera confirmación en vehicle_status
  CLIMB   -> sube a `altitude_m`
  HOVER   -> mantiene la posición `hover_s` segundos
  LAND    -> ordena aterrizaje (modo de PX4) y espera al desarmado automático
  DONE    -> termina

PX4 conserva la autoridad (ADR-001): si este nodo deja de publicar el latido
OffboardControlMode, PX4 aplica su failsafe de pérdida de Offboard.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

PERIOD_S = 0.1          # 10 Hz; PX4 exige > 2 Hz de latido Offboard
STREAM_CYCLES = 10      # 1 s de consignas antes de pedir Offboard
ARMING_TIMEOUT_S = 5.0  # reintento de modo/armado
ALT_TOLERANCE_M = 0.5


class TakeoffHoverLand(Node):
    def __init__(self) -> None:
        super().__init__('takeoff_hover_land')

        self.declare_parameter('altitude_m', 10.0)
        self.declare_parameter('hover_s', 15.0)
        # PX4 v1.17 añade el sufijo _vN a los tópicos cuyo mensaje tiene MESSAGE_VERSION != 0.
        # VehicleStatus y VehicleLocalPosition están en la versión 1; OffboardControlMode,
        # TrajectorySetpoint y VehicleCommand en la 0 (sin sufijo). Comprobar con `ros2 topic list`.
        self.declare_parameter('topic_vehicle_status', '/fmu/out/vehicle_status_v1')
        self.declare_parameter('topic_local_position', '/fmu/out/vehicle_local_position_v1')

        self.altitude_m = float(self.get_parameter('altitude_m').value)
        self.hover_s = float(self.get_parameter('hover_s').value)
        topic_status = str(self.get_parameter('topic_vehicle_status').value)
        topic_position = str(self.get_parameter('topic_local_position').value)

        # QoS compatible con los publicadores de PX4
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_command = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.create_subscription(VehicleStatus, topic_status, self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, topic_position, self._on_position, qos)

        self.status = None
        self.position = None
        self.state = 'STREAM'
        self.cycles = 0
        self.state_t0 = self._now_s()
        self.hold_xy = (0.0, 0.0)

        self.timer = self.create_timer(PERIOD_S, self._step)
        self.get_logger().info(
            f'S1: objetivo {self.altitude_m:.1f} m, estacionario {self.hover_s:.0f} s')

    # --- Suscripciones -----------------------------------------------------
    def _on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self.position = msg

    # --- Utilidades --------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _set_state(self, new_state: str) -> None:
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.state = new_state
        self.state_t0 = self._now_s()

    def _publish_heartbeat(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self.pub_offboard.publish(msg)

    def _publish_setpoint(self, x: float, y: float, z_ned: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z_ned)]
        nan = float('nan')
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.yaw = nan  # mantiene el rumbo actual
        msg.timestamp = self._timestamp_us()
        self.pub_setpoint.publish(msg)

    def _send_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.pub_command.publish(msg)

    def _request_offboard_and_arm(self) -> None:
        # param1 = 1 (modo personalizado), param2 = 6 (Offboard en PX4)
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _is_offboard_and_armed(self) -> bool:
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    # --- Máquina de estados ------------------------------------------------
    def _step(self) -> None:
        if self.state in ('STREAM', 'ARMING', 'CLIMB', 'HOVER'):
            self._publish_heartbeat()
            self._publish_setpoint(self.hold_xy[0], self.hold_xy[1], -self.altitude_m)

        elapsed = self._now_s() - self.state_t0

        if self.state == 'STREAM':
            self.cycles += 1
            if self.position is not None:
                self.hold_xy = (self.position.x, self.position.y)
            if self.cycles >= STREAM_CYCLES and self.status is not None:
                self._request_offboard_and_arm()
                self._set_state('ARMING')

        elif self.state == 'ARMING':
            if self._is_offboard_and_armed():
                self._set_state('CLIMB')
            elif elapsed > ARMING_TIMEOUT_S:
                self.get_logger().warn('Sin Offboard/armado; reintentando')
                self._request_offboard_and_arm()
                self.state_t0 = self._now_s()

        elif self.state == 'CLIMB':
            if self.position is not None and math.isfinite(self.position.z):
                if abs(self.position.z + self.altitude_m) < ALT_TOLERANCE_M:
                    self._set_state('HOVER')

        elif self.state == 'HOVER':
            if elapsed >= self.hover_s:
                self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._set_state('LAND')

        elif self.state == 'LAND':
            if (self.status is not None
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED):
                self.get_logger().info('Aterrizado y desarmado. S1 completado.')
                self._set_state('DONE')

        elif self.state == 'DONE':
            self.timer.cancel()
            raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TakeoffHoverLand()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
