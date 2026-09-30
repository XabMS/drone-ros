"""pilot_confirm_bridge — puente entre la confirmación del piloto por MAVLink y payload_manager (ADR-006).

Solo traduce: la decisión de soltar sigue en payload_manager (que pide su propia comprobación) y en drop_guard.

- Anuncia al piloto que el dron espera su confirmación (STATUSTEXT, repetido cada announce_period_s por si se pierde
  en el enlace LTE) y, al terminar la suelta, su resultado. Emite también un latido y la fase como NAMED_VALUE_INT.
- Recibe MAV_CMD_USER_1 dirigido al componente 191, lo valida contra la zona anunciada y llama a
  payload_manager ~/confirm. Responde con COMMAND_ACK.

Parámetros: mavlink_url (por defecto el enlace onboard de PX4 SITL), state_max_age_s, announce_period_s.
"""

import time
from typing import Optional

import rclpy
from drone_interfaces.msg import PayloadState
from drone_interfaces.srv import ConfirmDrop
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from drone_gcs_bridge import confirm_logic as logic
from drone_gcs_bridge.confirm_link import ConfirmLink, ConfirmRequest, SEVERITY_INFO, SEVERITY_WARNING


class PilotConfirmBridge(Node):
    def __init__(self) -> None:
        super().__init__('pilot_confirm_bridge')
        url = self.declare_parameter('mavlink_url', 'udpin:0.0.0.0:14540').value
        self.max_age_s = self.declare_parameter('state_max_age_s', 3.0).value
        self.announce_period_s = self.declare_parameter('announce_period_s', 10.0).value

        self.link = ConfirmLink(url)
        self.view: Optional[logic.PayloadView] = None
        self.prev_phase = logic.PHASE_IDLE
        self.last_announce_s = 0.0

        self.create_subscription(
            PayloadState, '/drone/payload/state', self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.confirm_client = self.create_client(ConfirmDrop, '/payload_manager/confirm')
        self.create_timer(0.05, self.poll)
        self.create_timer(1.0, self.beat)
        self.get_logger().info(f'pilot_confirm_bridge listo en {url} (componente {logic.COMPONENT_ID}).')

    def on_state(self, msg: PayloadState) -> None:
        self.view = logic.PayloadView(msg.phase, msg.zone_id, msg.zone_hash, msg.last_result, time.monotonic())
        if self.prev_phase == logic.PHASE_IDLE and msg.phase == logic.PHASE_WAITING_CONFIRMATION:
            self.announce()
        elif self.prev_phase != logic.PHASE_IDLE and msg.phase == logic.PHASE_IDLE:
            severity = SEVERITY_INFO if msg.last_result == 1 else SEVERITY_WARNING
            self.link.send_statustext(logic.outcome_text(msg.zone_id, msg.last_result), severity)
        self.prev_phase = msg.phase

    def announce(self) -> None:
        if self.view is not None:
            self.link.send_statustext(logic.announcement(self.view), SEVERITY_WARNING)
            self.last_announce_s = time.monotonic()

    def beat(self) -> None:
        self.link.send_heartbeat()
        self.link.send_phase(self.view.phase if self.view is not None else logic.PHASE_IDLE)
        waiting = self.view is not None and self.view.phase == logic.PHASE_WAITING_CONFIRMATION
        if waiting and time.monotonic() - self.last_announce_s >= self.announce_period_s:
            self.announce()

    def poll(self) -> None:
        for req in self.link.poll():
            self.handle_request(req)

    def handle_request(self, req: ConfirmRequest) -> None:   # no llamarlo `handle`: pisa Node.handle
        d = logic.decide(req.param1, req.param2, self.view, time.monotonic(), self.max_age_s)
        self.get_logger().info(
            f'Confirmación de {req.source_system}/{req.source_component}: '
            f'{"reenviada" if d.forward else "rechazada"} ({d.reason})')
        if not d.forward:
            self.link.send_ack(req, d.result)
            self.link.send_statustext(logic.rejection_text(d.reason), SEVERITY_WARNING)
            return
        if not self.confirm_client.service_is_ready():
            self.link.send_ack(req, logic.RESULT_FAILED)
            self.link.send_statustext(logic.rejection_text('payload_manager no disponible'), SEVERITY_WARNING)
            return
        future = self.confirm_client.call_async(ConfirmDrop.Request(drop_zone_id=d.zone_id))
        future.add_done_callback(lambda f, r=req: self.on_confirmed(f, r))

    def on_confirmed(self, future, req: ConfirmRequest) -> None:
        res = future.result()
        if res is not None and res.accepted:
            self.link.send_ack(req, logic.RESULT_ACCEPTED)
            return
        self.link.send_ack(req, logic.RESULT_DENIED)
        self.link.send_statustext(logic.rejection_text(res.message if res else 'sin respuesta'), SEVERITY_WARNING)


def main() -> None:
    rclpy.init()
    node = PilotConfirmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
