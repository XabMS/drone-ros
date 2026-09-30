#include "drone_payload/payload_state_machine.hpp"

#include <cmath>

namespace drone_payload
{

const char * to_string(Phase p)
{
  switch (p) {
    case Phase::IDLE: return "IDLE";
    case Phase::WAITING_CONFIRMATION: return "WAITING_CONFIRMATION";
    case Phase::RELEASING: return "RELEASING";
    case Phase::VERIFYING: return "VERIFYING";
  }
  return "?";
}

const char * to_string(Outcome o)
{
  switch (o) {
    case Outcome::NONE: return "NONE";
    case Outcome::RELEASED: return "RELEASED";
    case Outcome::DENIED: return "DENIED";
    case Outcome::TIMEOUT: return "TIMEOUT";
    case Outcome::NOT_RELEASED: return "NOT_RELEASED";
  }
  return "?";
}

const char * drop_guard_reason_name(int32_t reason)
{
  switch (reason) {
    case 0: return "OK";
    case 1: return "DISABLED";
    case 2: return "NOT_ARMED";
    case 3: return "PARAMS_INVALID";
    case 4: return "POS_STALE";
    case 5: return "POS_INVALID";
    case 6: return "EPH_HIGH";
    case 7: return "EPV_HIGH";
    case 8: return "HOME_INVALID";
    case 9: return "OUTSIDE_RADIUS";
    case 10: return "BELOW_BAND";
    case 11: return "ABOVE_BAND";
    default: return "DESCONOCIDO";
  }
}

PayloadStateMachine::PayloadStateMachine(const Params & params)
: p_(params)
{
}

bool PayloadStateMachine::start(
  const drone_core::DropZone & zone, int32_t zone_hash, double now_s, std::string & message)
{
  if (phase_ != Phase::IDLE) {
    message = "ya hay una suelta en curso";
    return false;
  }
  if (!drone_core::is_valid(zone.center) || !(zone.radius_m > 0.0) ||
    !(zone.alt_min_m < zone.alt_max_m))
  {
    message = "zona de suelta no válida";
    return false;
  }
  zone_ = zone;
  zone_hash_ = zone_hash;
  phase_ = Phase::WAITING_CONFIRMATION;
  outcome_ = Outcome::NONE;
  message_.clear();
  entry_s_ = now_s;
  confirm_pending_ = false;
  release_commanded_ = false;
  ack_received_ = false;
  ack_accepted_ = false;
  ack_reason_ = 0;
  message = "esperando la confirmación del piloto";
  return true;
}

bool PayloadStateMachine::confirm(std::string & message)
{
  if (phase_ != Phase::WAITING_CONFIRMATION) {
    message = "no se espera confirmación";
    return false;
  }
  if (confirm_pending_) {
    message = "confirmación ya recibida";
    return false;
  }
  confirm_pending_ = true;
  message = "confirmación aceptada";
  return true;
}

bool PayloadStateMachine::cancel(std::string & message)
{
  if (phase_ != Phase::WAITING_CONFIRMATION) {
    message = "no se puede cancelar: la apertura ya está ordenada o no hay suelta";
    return false;
  }
  phase_ = Phase::IDLE;
  confirm_pending_ = false;
  message = "suelta cancelada";
  return true;
}

void PayloadStateMachine::on_ack(bool accepted, int32_t reason)
{
  if ((phase_ != Phase::RELEASING && phase_ != Phase::VERIFYING) || ack_received_) {
    return;
  }
  ack_received_ = true;
  ack_accepted_ = accepted;
  ack_reason_ = reason;
}

void PayloadStateMachine::finish(Outcome o, const std::string & message)
{
  phase_ = Phase::IDLE;
  outcome_ = o;
  last_outcome_ = o;
  message_ = message;
  confirm_pending_ = false;
}

std::string PayloadStateMachine::release_blocker(const Inputs & in) const
{
  if (!in.armed) {
    return "el dron no está armado";
  }
  if (!in.pos_valid || !(in.pos_age_s <= p_.position_timeout_s) || !drone_core::is_valid(in.pos) ||
    !std::isfinite(in.alt_amsl_m) || !std::isfinite(in.eph_m) || !std::isfinite(in.epv_m) ||
    in.eph_m < 0.0 || in.epv_m < 0.0)
  {
    return "posición no válida o caducada";
  }
  if (!in.home_valid || !std::isfinite(in.home_alt_amsl_m)) {
    return "altura del punto de despegue no válida";
  }
  if (!in.payload_valid) {
    return "el sensor de carga no tiene lectura";
  }
  if (!in.payload_present) {
    return "no hay carga en el mecanismo";
  }
  if (!in.dg_valid) {
    return "sin estado de drop_guard";
  }
  if (in.dg_zone_hash != zone_hash_) {
    return "el hash de zona de drop_guard no coincide con la misión";
  }
  const double d = drone_core::haversine_m(in.pos, zone_.center);
  if (!(d + in.eph_m <= zone_.radius_m)) {
    return "fuera del radio de la zona de suelta";
  }
  const double h = in.alt_amsl_m - in.home_alt_amsl_m;
  if (!(h >= zone_.alt_min_m + in.epv_m) || !(h <= zone_.alt_max_m - in.epv_m)) {
    return "fuera de la banda de altura de suelta";
  }
  return {};
}

Command PayloadStateMachine::step(const Inputs & in)
{
  Command cmd;

  // Un rechazo explícito de PX4 manda sobre cualquier otra evidencia, también si llega tarde.
  if ((phase_ == Phase::RELEASING || phase_ == Phase::VERIFYING) && ack_received_ && !ack_accepted_) {
    finish(Outcome::DENIED, std::string("drop_guard rechazó la apertura: ") + drop_guard_reason_name(ack_reason_));
    return cmd;
  }

  switch (phase_) {
    case Phase::IDLE:
      break;

    case Phase::WAITING_CONFIRMATION:
      if (confirm_pending_) {
        confirm_pending_ = false;
        const std::string why = release_blocker(in);
        if (!why.empty()) {
          finish(Outcome::DENIED, why);
        } else {
          phase_ = Phase::RELEASING;
          command_s_ = in.now_s;
          release_commanded_ = true;
          cmd.type = CommandType::RELEASE;
        }
      } else if (in.now_s - entry_s_ >= p_.confirm_timeout_s) {
        finish(Outcome::TIMEOUT, "el piloto no confirmó a tiempo");
      }
      break;

    case Phase::RELEASING:
      if (ack_received_) {
        phase_ = Phase::VERIFYING;
      } else if (in.now_s - command_s_ >= p_.ack_timeout_s) {
        // Sin ack: la evidencia física decide en VERIFYING.
        phase_ = Phase::VERIFYING;
      }
      break;

    case Phase::VERIFYING:
      if (in.payload_valid && !in.payload_present) {
        finish(Outcome::RELEASED, ack_received_ ? "carga liberada" : "carga liberada (sin ack de PX4)");
      } else if (in.now_s - command_s_ >= p_.verify_timeout_s) {
        finish(Outcome::NOT_RELEASED,
          in.payload_valid ? "la carga sigue en el mecanismo" : "sin lectura del sensor de carga");
      }
      break;
  }
  return cmd;
}

}  // namespace drone_payload
