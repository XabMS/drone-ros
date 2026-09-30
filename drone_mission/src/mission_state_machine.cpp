#include "drone_mission/mission_state_machine.hpp"

#include <cmath>
#include <cstdio>

namespace drone_mission
{

const char * to_string(State s)
{
  switch (s) {
    case State::PREFLIGHT: return "PREFLIGHT";
    case State::ARMING: return "ARMING";
    case State::TAKEOFF: return "TAKEOFF";
    case State::CRUISE: return "CRUISE";
    case State::APPROACH: return "APPROACH";
    case State::DROP: return "DROP";
    case State::RETURN: return "RETURN";
    case State::CONTINGENCY: return "CONTINGENCY";
    case State::COMPLETED: return "COMPLETED";
  }
  return "?";
}

const char * to_string(DropResult r)
{
  switch (r) {
    case DropResult::NONE: return "NONE";
    case DropResult::RELEASED: return "RELEASED";
    case DropResult::DENIED: return "DENIED";
    case DropResult::TIMEOUT: return "TIMEOUT";
    case DropResult::NOT_RELEASED: return "NOT_RELEASED";
    case DropResult::UNAVAILABLE: return "UNAVAILABLE";
  }
  return "?";
}

namespace
{

/** Motivo de la transición DROP -> RETURN según el resultado de la suelta. */
const char * drop_cause(DropResult r)
{
  switch (r) {
    case DropResult::RELEASED: return "carga liberada en la zona de suelta";
    case DropResult::DENIED: return "suelta denegada: regreso con la carga";
    case DropResult::TIMEOUT: return "el piloto no confirmó la suelta: regreso con la carga";
    case DropResult::NOT_RELEASED: return "la carga no se liberó: regreso con la carga";
    case DropResult::UNAVAILABLE:
    case DropResult::NONE: return "sin resultado de payload_manager: regreso con la carga";
  }
  return "?";
}

}  // namespace

MissionStateMachine::MissionStateMachine(const Params & params)
: p_(params)
{
}

bool MissionStateMachine::set_plan(const Plan & plan, std::string & message)
{
  if (state_ != State::PREFLIGHT && state_ != State::COMPLETED) {
    message = "solo se puede cargar un plan en PREFLIGHT o COMPLETED";
    return false;
  }
  if (!drone_core::is_valid(plan.drop_center) || !(plan.cruise_alt_m > 0.0) || !(plan.drop_alt_m > 0.0)) {
    message = "plan incompleto";
    return false;
  }
  plan_ = plan;
  targets_.clear();
  for (const LatLon & p : plan.route) {
    targets_.push_back(Target{p, plan.cruise_alt_m});
  }
  targets_.push_back(Target{plan.drop_center, plan.cruise_alt_m});
  has_plan_ = true;
  message = "plan '" + plan.mission_id + "' cargado";
  return true;
}

std::vector<std::string> MissionStateMachine::preflight_blockers(const Inputs & in) const
{
  std::vector<std::string> b;
  if (!has_plan_) {
    b.emplace_back("sin misión cargada");
  }
  if (!in.config_ok) {
    b.emplace_back("configuración de operación no válida o distinta de la cargada");
  }
  if (!p_.simulated_drop) {
    if (!in.payload_ready) {
      b.emplace_back("payload_manager no disponible");
    }
    if (!in.drop_guard_ok) {
      b.emplace_back("drop_guard no está activo con la zona de suelta de la misión (DG_ENABLE, DG_ZONE_HASH)");
    }
  }
  if (!in.status_valid) {
    b.emplace_back("sin estado de PX4");
    return b;
  }
  if (in.armed) {
    b.emplace_back("el dron ya está armado");
  }
  if (in.failsafe) {
    b.emplace_back("PX4 en failsafe");
  }
  if (!in.preflight_checks_pass) {
    b.emplace_back("comprobaciones de prevuelo de PX4 no superadas");
  }
  if (!in.pos_valid) {
    b.emplace_back("posición global no válida");
  }
  // El home no se exige aquí: PX4 publica home_position solo cuando cambia y el mensaje puede
  // haberse perdido antes de conectar el agente DDS. PX4 lo vuelve a fijar al armar (ver ARMING).
  return b;
}

bool MissionStateMachine::request_start(std::string & message)
{
  if (state_ != State::PREFLIGHT && state_ != State::COMPLETED) {
    message = std::string("no se puede iniciar en ") + to_string(state_);
    return false;
  }
  if (!have_inputs_) {
    message = "aún no hay datos del vehículo";
    return false;
  }
  const std::vector<std::string> blockers = preflight_blockers(last_inputs_);
  if (!blockers.empty()) {
    message = "prevuelo incompleto: " + blockers.front();
    return false;
  }
  start_requested_ = true;
  message = "inicio aceptado";
  return true;
}

bool MissionStateMachine::request_abort(std::string & message)
{
  if (state_ == State::ARMING || state_ == State::TAKEOFF || state_ == State::CRUISE ||
    state_ == State::APPROACH || state_ == State::DROP)
  {
    abort_requested_ = true;
    message = "aborto aceptado";
    return true;
  }
  message = std::string("nada que abortar en ") + to_string(state_);
  return false;
}

void MissionStateMachine::transition(State next, const std::string & cause, double now_s)
{
  previous_ = state_;
  state_ = next;
  cause_ = cause;
  ++transitions_;
  state_entry_s_ = now_s;
  last_command_s_ = -1e9;  // fuerza el envío de la orden del nuevo estado
  mode_bad_since_s_ = -1.0;
  if (is_goto_state()) {
    need_goto_ = true;
    goto_confirmed_ = false;
    goto_sent_count_ = 0;
    sp_mismatch_since_s_ = -1.0;
  }
}

bool MissionStateMachine::is_goto_state() const
{
  return state_ == State::CRUISE || state_ == State::APPROACH || state_ == State::DROP;
}

bool MissionStateMachine::setpoint_matches(const Inputs & in) const
{
  if (!in.setpoint_valid || !drone_core::is_valid(in.setpoint)) {
    return false;
  }
  const Command target = goto_current_target(in);
  return drone_core::haversine_m(in.setpoint, target.pos) <= p_.setpoint_tolerance_m &&
         std::fabs(in.setpoint_alt_amsl_m - target.alt_amsl_m) <= p_.setpoint_alt_tolerance_m;
}

bool MissionStateMachine::in_flight_state() const
{
  return state_ == State::TAKEOFF || state_ == State::CRUISE || state_ == State::APPROACH ||
         state_ == State::DROP || state_ == State::RETURN;
}

bool MissionStateMachine::mode_is_expected(uint8_t nav_state) const
{
  switch (state_) {
    case State::TAKEOFF:
      return nav_state == nav::kAutoTakeoff || nav_state == nav::kAutoLoiter;
    case State::CRUISE:
    case State::APPROACH:
    case State::DROP:
      return nav_state == nav::kAutoLoiter;
    case State::RETURN:
      return nav_state == nav::kAutoRtl || nav_state == nav::kAutoLand;
    default:
      return true;
  }
}

bool MissionStateMachine::reached(const Inputs & in, const Target & t)
{
  if (!in.pos_valid || !in.home_valid) {
    return false;
  }
  distance_to_target_ = drone_core::haversine_m(in.pos, t.pos);
  const double alt_error = std::fabs(in.alt_amsl_m - (in.home_alt_amsl_m + t.alt_rel_m));
  return distance_to_target_ <= p_.acceptance_radius_m && alt_error <= p_.alt_tolerance_m;
}

double MissionStateMachine::leg_timeout_s(const Inputs & in, const Target & t) const
{
  const double horiz = in.pos_valid ? drone_core::haversine_m(in.pos, t.pos) : 0.0;
  const double vert = std::fabs(in.alt_amsl_m - (in.home_alt_amsl_m + t.alt_rel_m));
  return (horiz + vert) / p_.leg_speed_mps + p_.leg_timeout_margin_s;
}

Command MissionStateMachine::goto_current_target(const Inputs & in) const
{
  const Target & t = (state_ == State::APPROACH || state_ == State::DROP) ?
    targets_.back() : targets_[target_index_];
  const double alt_rel = (state_ == State::APPROACH || state_ == State::DROP) ?
    plan_.drop_alt_m : t.alt_rel_m;
  Command c;
  c.type = CommandType::GOTO;
  c.pos = t.pos;
  c.alt_amsl_m = in.home_alt_amsl_m + alt_rel;
  return c;
}

Command MissionStateMachine::command_for_state(const Inputs & in)
{
  Command c;
  switch (state_) {
    case State::ARMING:
      c.type = CommandType::ARM;
      break;
    case State::TAKEOFF:
      c.type = CommandType::TAKEOFF;
      c.pos = in.home;
      c.alt_amsl_m = in.home_alt_amsl_m + plan_.cruise_alt_m;
      break;
    case State::RETURN:
      c.type = CommandType::RTL;
      break;
    default:
      break;
  }
  return c;
}

Command MissionStateMachine::step(const Inputs & in)
{
  last_inputs_ = in;
  have_inputs_ = true;
  const double now = in.now_s;

  // --- Peticiones del operador ---------------------------------------------
  if (abort_requested_) {
    abort_requested_ = false;
    if (state_ == State::ARMING) {
      start_requested_ = false;
      transition(State::PREFLIGHT, "abortado antes del despegue", now);
    } else if (state_ == State::TAKEOFF || state_ == State::CRUISE || state_ == State::APPROACH ||
      state_ == State::DROP)
    {
      aborted_ = true;
      transition(State::RETURN, "abortado por el operador", now);
    }
  }

  // --- Vigilancia común de las fases de vuelo --------------------------------
  if (in_flight_state()) {
    if (!in.armed) {
      const bool normal_end = (state_ == State::RETURN);
      result_ = normal_end ? (aborted_ ? Result::ABORTED : Result::COMPLETED) : Result::CONTINGENCY;
      transition(State::COMPLETED, normal_end ? "aterrizado y desarmado en el hub" : "desarmado inesperado en vuelo", now);
      return Command{};
    }
    if (in.failsafe && state_ != State::RETURN) {
      transition(State::CONTINGENCY, "failsafe de PX4: PX4 toma el control", now);
      return Command{};
    }
    if (!mode_is_expected(in.nav_state)) {
      if (mode_bad_since_s_ < 0.0) {
        mode_bad_since_s_ = now;
      }
      const bool grace_over = (now - mode_bad_since_s_ > p_.mode_grace_s) &&
        (now - last_command_s_ > p_.mode_grace_s);
      if (grace_over) {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "modo cambiado externamente (nav_state %u): toma de control", in.nav_state);
        transition(State::CONTINGENCY, buf, now);
        return Command{};
      }
    } else {
      mode_bad_since_s_ = -1.0;
    }

    // Consigna de PX4 frente a la orden enviada (AR-017: pausa o reposicionamiento del piloto).
    if (is_goto_state() && goto_sent_count_ > 0) {
      const bool match = setpoint_matches(in);
      if (!goto_confirmed_) {
        if (match) {
          goto_confirmed_ = true;
        } else if (now - goto_sent_s_ > p_.mode_grace_s) {
          if (goto_sent_count_ >= p_.goto_attempts) {
            transition(State::CONTINGENCY, "PX4 no acepta la orden de reposicionamiento", now);
            return Command{};
          }
          need_goto_ = true;  // reintento
        }
      } else if (!match) {
        if (sp_mismatch_since_s_ < 0.0) {
          sp_mismatch_since_s_ = now;
        } else if (now - sp_mismatch_since_s_ > p_.setpoint_mismatch_s) {
          transition(State::CONTINGENCY, "consigna cambiada externamente (pausa o reposicionamiento del piloto)", now);
          return Command{};
        }
      } else {
        sp_mismatch_since_s_ = -1.0;
      }
    }
  }

  // Resultado tardío de una suelta ya pedida (p. ej. tras un aborto en DROP): solo se registra.
  if (!p_.simulated_drop && drop_requested_ && drop_result_ == DropResult::NONE &&
    state_ != State::DROP && in.drop_result != DropResult::NONE)
  {
    drop_result_ = in.drop_result;
  }

  // --- Lógica de cada estado --------------------------------------------------
  switch (state_) {
    case State::PREFLIGHT:
    case State::COMPLETED:
      if (start_requested_) {
        start_requested_ = false;
        aborted_ = false;
        result_ = Result::NONE;
        drop_requested_ = false;
        drop_request_pending_ = false;
        drop_result_ = DropResult::NONE;
        target_index_ = 0;
        transition(State::ARMING, "inicio de misión", now);
      }
      break;

    case State::ARMING:
      if (in.armed && in.home_valid) {
        transition(State::TAKEOFF, "armado y con home", now);
        leg_timeout_ = p_.takeoff_timeout_s;
      } else if (now - state_entry_s_ > p_.arming_timeout_s) {
        // Si armó pero no llegó el home, PX4 desarma solo al no despegar (COM_DISARM_PRFLT).
        transition(State::PREFLIGHT, in.armed ? "armado pero sin home de PX4" : "PX4 no armó a tiempo", now);
      }
      break;

    case State::TAKEOFF:
      if (in.home_valid && in.alt_amsl_m >= in.home_alt_amsl_m + plan_.cruise_alt_m - p_.alt_tolerance_m) {
        target_index_ = 0;
        transition(State::CRUISE, "altura de crucero alcanzada", now);
        leg_start_s_ = now;
        leg_timeout_ = leg_timeout_s(in, targets_[0]);
      } else if (now - state_entry_s_ > p_.takeoff_timeout_s) {
        transition(State::RETURN, "despegue demasiado lento", now);
      }
      break;

    case State::CRUISE:
      if (reached(in, targets_[target_index_])) {
        ++target_index_;
        if (target_index_ >= targets_.size()) {
          target_index_ = targets_.size() - 1;
          transition(State::APPROACH, "sobre la zona de suelta: descenso", now);
          leg_start_s_ = now;
          leg_timeout_ = leg_timeout_s(in, Target{plan_.drop_center, plan_.drop_alt_m});
        } else {
          need_goto_ = true;  // nuevo tramo: enviar la orden ya
          goto_confirmed_ = false;
          goto_sent_count_ = 0;
          sp_mismatch_since_s_ = -1.0;
          leg_start_s_ = now;
          leg_timeout_ = leg_timeout_s(in, targets_[target_index_]);
        }
      } else if (now - leg_start_s_ > leg_timeout_) {
        transition(State::RETURN, "tramo de crucero demasiado lento", now);
      }
      break;

    case State::APPROACH:
      if (reached(in, Target{plan_.drop_center, plan_.drop_alt_m})) {
        transition(State::DROP, "en la zona de suelta", now);
      } else if (now - leg_start_s_ > leg_timeout_) {
        transition(State::RETURN, "aproximación demasiado lenta", now);
      }
      break;

    case State::DROP:
      if (p_.simulated_drop) {
        if (now - state_entry_s_ >= p_.drop_wait_s) {
          transition(State::RETURN, "suelta simulada completada", now);
        }
        break;
      }
      if (!drop_requested_) {
        // Se pide cuando PX4 ya aceptó el Hold sobre la zona (GOTO confirmado). Si no lo acepta, la
        // vigilancia común pasa a CONTINGENCY tras los reintentos.
        if (goto_confirmed_) {
          drop_requested_ = true;
          drop_request_pending_ = true;
          drop_requested_s_ = now;
        }
        break;
      }
      if (in.drop_result != DropResult::NONE) {
        drop_result_ = in.drop_result;
        transition(State::RETURN, drop_cause(drop_result_), now);
      } else if (now - drop_requested_s_ >= p_.drop_timeout_s) {
        drop_result_ = DropResult::UNAVAILABLE;
        transition(State::RETURN, drop_cause(drop_result_), now);
      }
      break;

    case State::RETURN:
      // El fin del regreso (desarmado) se trata en la vigilancia común de las fases de vuelo.
      break;

    case State::CONTINGENCY:
      if (!in.armed) {
        result_ = Result::CONTINGENCY;
        transition(State::COMPLETED, "desarmado tras contingencia", now);
      }
      break;
  }

  // --- Orden a enviar ---------------------------------------------------------
  if (state_ == State::CONTINGENCY || state_ == State::PREFLIGHT || state_ == State::COMPLETED) {
    return Command{};
  }
  if (state_ == State::DROP && drop_request_pending_) {
    drop_request_pending_ = false;
    last_command_s_ = now;
    Command c;
    c.type = CommandType::DROP;
    return c;
  }
  if (is_goto_state()) {
    // GOTO se envía una vez por tramo y solo se repite si PX4 no lo confirma (ver vigilancia).
    if (!need_goto_) {
      return Command{};
    }
    need_goto_ = false;
    ++goto_sent_count_;
    goto_sent_s_ = now;
    last_command_s_ = now;
    return goto_current_target(in);
  }
  if (state_ == State::RETURN && (in.nav_state == nav::kAutoRtl || in.nav_state == nav::kAutoLand) &&
    last_command_s_ > -1e8)
  {
    return Command{};  // PX4 ya está regresando: no repetir
  }
  if (state_ == State::TAKEOFF && in.nav_state == nav::kAutoTakeoff && last_command_s_ > -1e8) {
    return Command{};  // PX4 ya está despegando: no repetir
  }
  if (now - last_command_s_ < p_.command_resend_s) {
    return Command{};
  }
  last_command_s_ = now;
  return command_for_state(in);
}

}  // namespace drone_mission
