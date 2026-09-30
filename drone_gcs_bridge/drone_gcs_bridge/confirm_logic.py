"""Lógica de la confirmación de suelta del piloto (ADR-006). Python puro: sin ROS ni MAVLink.

Comando de confirmación (DOC-06 §4): COMMAND_LONG dirigido al companion (componente 191) con
MAV_CMD_USER_1. Lleva el DG_ZONE_HASH de la zona anunciada al piloto, partido en dos mitades de
16 bits (un float32 solo representa enteros exactos hasta 2^24):

    param1 = hash & 0xFFFF        param2 = (hash >> 16) & 0xFFFF      (hash visto como uint32)

Así una confirmación de otra misión, o pulsada por error, no autoriza la suelta actual. Esto no
sustituye a payload_manager ni a drop_guard: ellos siguen comprobando la zona por su cuenta.
"""

import math
import unicodedata
from dataclasses import dataclass
from typing import Optional

MAV_CMD_USER_1 = 31010
COMPONENT_ID = 191  # companion (DOC-06 §4)

# MAV_RESULT
RESULT_ACCEPTED = 0
RESULT_TEMPORARILY_REJECTED = 1
RESULT_DENIED = 2
RESULT_FAILED = 4

# PayloadState.phase
PHASE_IDLE = 0
PHASE_WAITING_CONFIRMATION = 1

# PayloadState.last_result / DropPayload.RESULT_*
RESULT_NAMES = {
    0: 'SIN RESULTADO',
    1: 'CARGA LIBERADA',
    2: 'DENEGADA',
    3: 'SIN CONFIRMACION',
    4: 'CARGA NO LIBERADA',
}

STATUSTEXT_MAX = 50


def hash_to_params(zone_hash: int) -> tuple:
    """Devuelve (param1, param2) de MAV_CMD_USER_1 para un DG_ZONE_HASH (int32 con signo)."""
    unsigned = zone_hash & 0xFFFFFFFF
    return float(unsigned & 0xFFFF), float(unsigned >> 16)


def params_to_hash(param1: float, param2: float) -> Optional[int]:
    """Inverso de hash_to_params. None si los parámetros no son dos enteros de 16 bits."""
    halves = []
    for p in (param1, param2):
        if not math.isfinite(p) or p != math.floor(p) or not 0 <= p <= 0xFFFF:
            return None
        halves.append(int(p))
    unsigned = (halves[1] << 16) | halves[0]
    return unsigned - (1 << 32) if unsigned >= (1 << 31) else unsigned


def ascii_text(text: str, limit: int = STATUSTEXT_MAX) -> str:
    """STATUSTEXT es ASCII de 50 caracteres: quita acentos y recorta."""
    plain = unicodedata.normalize('NFKD', text).encode('ascii', 'ignore').decode('ascii')
    return plain[:limit]


@dataclass
class PayloadView:
    """Lo último que publicó payload_manager (PayloadState)."""
    phase: int
    zone_id: str
    zone_hash: int
    last_result: int
    received_s: float


@dataclass
class Decision:
    forward: bool          # true: llamar a payload_manager ~/confirm
    result: int            # MAV_RESULT que se devuelve ya si no se reenvía
    reason: str
    zone_id: str = ''


def decide(param1: float, param2: float, view: Optional[PayloadView], now_s: float,
           max_age_s: float) -> Decision:
    """Decide qué hacer con una confirmación recibida por MAVLink."""
    if view is None or now_s - view.received_s > max_age_s:
        return Decision(False, RESULT_TEMPORARILY_REJECTED, 'sin estado reciente de payload_manager')
    if view.phase != PHASE_WAITING_CONFIRMATION:
        return Decision(False, RESULT_DENIED, 'no se espera confirmacion de suelta')
    zone_hash = params_to_hash(param1, param2)
    if zone_hash is None:
        return Decision(False, RESULT_DENIED, 'parametros de confirmacion no validos')
    if zone_hash != view.zone_hash:
        return Decision(False, RESULT_DENIED, 'el hash no coincide con la zona anunciada')
    return Decision(True, RESULT_ACCEPTED, 'confirmacion valida', view.zone_id)


def announcement(view: PayloadView) -> str:
    """STATUSTEXT que avisa al piloto de que el dron espera su confirmación."""
    unsigned = view.zone_hash & 0xFFFFFFFF
    return ascii_text(f'SUELTA {view.zone_id}: confirmar {unsigned:08X}')


def rejection_text(reason: str) -> str:
    """STATUSTEXT con el motivo por el que se rechazó una confirmación (cabe entero en 50 caracteres)."""
    return ascii_text(f'RECHAZO: {reason}')


def outcome_text(zone_id: str, last_result: int) -> str:
    """STATUSTEXT con el resultado de la suelta."""
    return ascii_text(f'SUELTA {zone_id}: {RESULT_NAMES.get(last_result, "?")}')
