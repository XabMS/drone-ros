"""Tests de la lógica de confirmación de suelta (ADR-006, SR-PLD-002). No necesitan ROS ni pymavlink."""
import math
import struct

import pytest

from drone_gcs_bridge import confirm_logic as logic

INT32_VALUES = [0, 1, -1, 0x7FFFFFFF, -0x80000000, -943523694, -1657142294, 0x00010000, 0x0000FFFF]


@pytest.mark.parametrize('zone_hash', INT32_VALUES)
def test_hash_roundtrip(zone_hash):
    p1, p2 = logic.hash_to_params(zone_hash)
    assert logic.params_to_hash(p1, p2) == zone_hash


@pytest.mark.parametrize('zone_hash', INT32_VALUES)
def test_params_survive_float32(zone_hash):
    # MAVLink manda los parámetros como float32: no debe perderse ningún bit del hash
    p1, p2 = (struct.unpack('<f', struct.pack('<f', p))[0] for p in logic.hash_to_params(zone_hash))
    assert logic.params_to_hash(p1, p2) == zone_hash


@pytest.mark.parametrize('p1,p2', [
    (math.nan, 0.0), (0.0, math.nan), (math.inf, 0.0), (0.0, -math.inf),
    (-1.0, 0.0), (0.0, -1.0), (65536.0, 0.0), (0.0, 65536.0), (1.5, 0.0), (0.0, 2.5),
])
def test_invalid_params_are_rejected(p1, p2):
    assert logic.params_to_hash(p1, p2) is None


def make_view(phase=logic.PHASE_WAITING_CONFIRMATION, zone_hash=-943523694, received_s=100.0):
    return logic.PayloadView(phase, 'tls_dz_01', zone_hash, 0, received_s)


def test_valid_confirmation_is_forwarded_with_the_announced_zone():
    view = make_view()
    p1, p2 = logic.hash_to_params(view.zone_hash)
    d = logic.decide(p1, p2, view, now_s=101.0, max_age_s=3.0)
    assert d.forward and d.result == logic.RESULT_ACCEPTED and d.zone_id == 'tls_dz_01'


def test_no_state_or_stale_state_is_temporarily_rejected():
    p1, p2 = logic.hash_to_params(-943523694)
    d = logic.decide(p1, p2, None, now_s=1.0, max_age_s=3.0)
    assert not d.forward and d.result == logic.RESULT_TEMPORARILY_REJECTED
    d = logic.decide(p1, p2, make_view(received_s=100.0), now_s=103.5, max_age_s=3.0)
    assert not d.forward and d.result == logic.RESULT_TEMPORARILY_REJECTED
    d = logic.decide(p1, p2, make_view(received_s=100.0), now_s=103.0, max_age_s=3.0)
    assert d.forward   # justo en el límite todavía vale


@pytest.mark.parametrize('phase', [0, 2, 3])
def test_confirmation_outside_waiting_is_denied(phase):
    view = make_view(phase=phase)
    p1, p2 = logic.hash_to_params(view.zone_hash)
    d = logic.decide(p1, p2, view, now_s=100.5, max_age_s=3.0)
    assert not d.forward and d.result == logic.RESULT_DENIED


def test_wrong_hash_is_denied():
    view = make_view()
    p1, p2 = logic.hash_to_params(view.zone_hash + 1)
    d = logic.decide(p1, p2, view, now_s=100.5, max_age_s=3.0)
    assert not d.forward and d.result == logic.RESULT_DENIED
    assert 'hash' in d.reason


def test_malformed_params_are_denied():
    d = logic.decide(math.nan, 0.0, make_view(), now_s=100.5, max_age_s=3.0)
    assert not d.forward and d.result == logic.RESULT_DENIED


def test_ascii_text_strips_accents_and_truncates():
    assert logic.ascii_text('DENEGADA: fuera de la zona de suelta') == 'DENEGADA: fuera de la zona de suelta'
    assert logic.ascii_text('confirmación válida') == 'confirmacion valida'
    assert len(logic.ascii_text('x' * 200)) == logic.STATUSTEXT_MAX


def test_announcement_shows_zone_and_unsigned_hash_within_limit():
    text = logic.announcement(make_view(zone_hash=-1))
    assert text == 'SUELTA tls_dz_01: confirmar FFFFFFFF'
    long_zone = logic.PayloadView(1, 'zona_con_un_identificador_muy_largo_' * 3, 5, 0, 0.0)
    assert len(logic.announcement(long_zone)) <= logic.STATUSTEXT_MAX


def test_every_rejection_reason_fits_in_one_statustext():
    # STATUSTEXT corta a 50 caracteres: ningún motivo de rechazo debe quedar truncado
    view = make_view()
    p1, p2 = logic.hash_to_params(view.zone_hash)
    reasons = {
        logic.decide(p1, p2, None, 1.0, 3.0).reason,
        logic.decide(p1, p2, make_view(phase=0), 100.5, 3.0).reason,
        logic.decide(math.nan, 0.0, view, 100.5, 3.0).reason,
        logic.decide(*logic.hash_to_params(view.zone_hash + 1), view, 100.5, 3.0).reason,
        'payload_manager no disponible',
        'la zona no es la de la suelta en curso',   # respuesta de payload_manager ~/confirm (debe coincidir con su código)
        'no se espera confirmación',
    }
    for reason in reasons:
        text = logic.rejection_text(reason)
        assert text == 'RECHAZO: ' + logic.ascii_text(reason, 100), reason


def test_outcome_text_names_every_result():
    assert logic.outcome_text('z1', 1) == 'SUELTA z1: CARGA LIBERADA'
    assert logic.outcome_text('z1', 3) == 'SUELTA z1: SIN CONFIRMACION'
    assert logic.outcome_text('z1', 4) == 'SUELTA z1: CARGA NO LIBERADA'
    assert logic.outcome_text('z1', 99) == 'SUELTA z1: ?'
