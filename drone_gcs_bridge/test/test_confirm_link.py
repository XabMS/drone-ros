"""Tests del enlace MAVLink de la confirmación (bucle local por UDP entre dos conexiones pymavlink)."""
import socket
import time

import pytest

pytest.importorskip('pymavlink')

from pymavlink import mavutil  # noqa: E402

from drone_gcs_bridge import confirm_logic as logic  # noqa: E402
from drone_gcs_bridge.confirm_link import ConfirmLink  # noqa: E402

M = mavutil.mavlink
GCS_SYSTEM, GCS_COMPONENT = 255, 190


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def wait_for(fn, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        value = fn()
        if value:
            return value
        time.sleep(0.02)
    return None


@pytest.fixture
def pair():
    port = free_port()
    link = ConfirmLink(f'udpin:127.0.0.1:{port}')
    gcs = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=GCS_SYSTEM,
                                     source_component=GCS_COMPONENT)
    yield link, gcs
    link.close()
    gcs.close()


def send_confirm(gcs, p1=1.0, p2=2.0, target_system=1, target_component=logic.COMPONENT_ID,
                 command=logic.MAV_CMD_USER_1):
    gcs.mav.command_long_send(target_system, target_component, command, 0, p1, p2, 0, 0, 0, 0, 0)


def test_confirmation_addressed_to_the_companion_is_received(pair):
    link, gcs = pair
    send_confirm(gcs, 1234.0, 55.0)
    reqs = wait_for(link.poll)
    assert reqs is not None and len(reqs) == 1
    assert (reqs[0].param1, reqs[0].param2) == (1234.0, 55.0)
    assert (reqs[0].source_system, reqs[0].source_component) == (GCS_SYSTEM, GCS_COMPONENT)


def test_other_messages_and_addressees_are_ignored(pair):
    link, gcs = pair
    send_confirm(gcs, target_component=0)                      # broadcast: no vale
    send_confirm(gcs, target_component=1)                      # al autopiloto
    send_confirm(gcs, target_system=2)                         # a otro sistema
    send_confirm(gcs, command=M.MAV_CMD_NAV_RETURN_TO_LAUNCH)  # otra orden
    gcs.mav.heartbeat_send(M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    send_confirm(gcs, 7.0, 8.0)                                # la única válida
    time.sleep(0.3)
    reqs = wait_for(link.poll)
    assert reqs is not None and [(r.param1, r.param2) for r in reqs] == [(7.0, 8.0)]


def test_ack_reaches_the_sender_with_the_result(pair):
    link, gcs = pair
    send_confirm(gcs)
    req = wait_for(link.poll)[0]
    assert link.send_ack(req, logic.RESULT_DENIED)
    ack = wait_for(lambda: gcs.recv_match(type='COMMAND_ACK', blocking=False))
    assert ack is not None
    assert ack.command == logic.MAV_CMD_USER_1 and ack.result == logic.RESULT_DENIED
    assert ack.get_srcComponent() == logic.COMPONENT_ID


def test_statustext_is_ascii_and_keeps_severity(pair):
    link, gcs = pair
    send_confirm(gcs)
    wait_for(link.poll)   # el enlace aprende la dirección de tierra
    assert link.send_statustext('SUELTA: confirmación válida', M.MAV_SEVERITY_WARNING)
    msg = wait_for(lambda: gcs.recv_match(type='STATUSTEXT', blocking=False))
    assert msg is not None
    assert msg.text == 'SUELTA: confirmacion valida' and msg.severity == M.MAV_SEVERITY_WARNING


def test_heartbeat_and_phase_identify_the_companion(pair):
    link, gcs = pair
    send_confirm(gcs)
    wait_for(link.poll)
    assert link.send_heartbeat() and link.send_phase(1)
    hb = wait_for(lambda: gcs.recv_match(type='HEARTBEAT', blocking=False))
    assert hb is not None and hb.type == M.MAV_TYPE_ONBOARD_CONTROLLER
    assert hb.get_srcComponent() == logic.COMPONENT_ID
    nv = wait_for(lambda: gcs.recv_match(type='NAMED_VALUE_INT', blocking=False))
    assert nv is not None and nv.name == 'PLD_PHASE' and nv.value == 1


def test_sending_before_any_peer_is_known_does_not_raise(pair):
    link, _ = pair
    link.send_heartbeat()
    link.send_phase(0)
    link.send_statustext('sin destino aun')   # no debe lanzar aunque nadie haya escrito todavía
