"""Enlace MAVLink del companion para la confirmación de suelta. Sin ROS: solo pymavlink.

Escucha en el enlace del companion (en el dron real, una salida de mavlink-router; en SITL, el enlace
«onboard» de PX4, que reenvía lo que llega de tierra) y solo atiende COMMAND_LONG MAV_CMD_USER_1
dirigido al componente 191. Todo lo demás se ignora.
"""

import os
from dataclasses import dataclass
from typing import List

# Debe fijarse antes de importar mavutil para hablar MAVLink 2 (necesario para result_param2 del ack).
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil  # noqa: E402

from drone_gcs_bridge.confirm_logic import COMPONENT_ID, MAV_CMD_USER_1, ascii_text  # noqa: E402

M = mavutil.mavlink

SEVERITY_INFO = M.MAV_SEVERITY_INFO
SEVERITY_WARNING = M.MAV_SEVERITY_WARNING


@dataclass
class ConfirmRequest:
    param1: float
    param2: float
    source_system: int
    source_component: int


def _ensure_mavlink2() -> None:
    """Fuerza MAVLink 2 aunque pymavlink ya se hubiera importado en MAVLink 1 (depende del orden de imports).

    Con MAVLink 1, COMMAND_ACK no lleva target_system/target_component y el ack no llegaría a quien confirmó.
    """
    os.environ['MAVLINK20'] = '1'
    if getattr(mavutil.mavlink, 'WIRE_PROTOCOL_VERSION', '2.0') != '2.0':
        mavutil.set_dialect('common')


class ConfirmLink:
    def __init__(self, url: str, system_id: int = 1, component_id: int = COMPONENT_ID):
        _ensure_mavlink2()
        self.system_id = system_id
        self.component_id = component_id
        self.conn = mavutil.mavlink_connection(url, source_system=system_id, source_component=component_id)

    def close(self) -> None:
        self.conn.close()

    def poll(self) -> List[ConfirmRequest]:
        """Confirmaciones pendientes (no bloquea). Descarta cualquier otro mensaje."""
        found = []
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return found
            if msg.get_type() != 'COMMAND_LONG' or msg.command != MAV_CMD_USER_1:
                continue
            if msg.target_system != self.system_id or msg.target_component != self.component_id:
                continue  # solo dirigido a este componente: un broadcast (componente 0) no vale
            found.append(ConfirmRequest(msg.param1, msg.param2, msg.get_srcSystem(), msg.get_srcComponent()))

    def _send(self, fn, *args) -> bool:
        try:
            fn(*args)
            return True
        except (OSError, TypeError, AttributeError):
            return False  # aún no hay destino (udpin sin tráfico) o el enlace se cayó: se reintenta en el siguiente envío

    def send_heartbeat(self) -> bool:
        return self._send(self.conn.mav.heartbeat_send, M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0,
                          M.MAV_STATE_ACTIVE)

    def send_phase(self, phase: int) -> bool:
        return self._send(self.conn.mav.named_value_int_send, 0, b'PLD_PHASE', phase)

    def send_statustext(self, text: str, severity: int = SEVERITY_INFO) -> bool:
        return self._send(self.conn.mav.statustext_send, severity, ascii_text(text).encode('ascii'))

    def send_ack(self, req: ConfirmRequest, result: int) -> bool:
        return self._send(self.conn.mav.command_ack_send, MAV_CMD_USER_1, result, 0, 0,
                          req.source_system, req.source_component)
