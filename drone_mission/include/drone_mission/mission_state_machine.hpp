// drone_mission — máquina de estados de la misión (ConOps §5, SR-MSN-001). IDAL C.
//
// Clase C++ pura, sin ROS: recibe el estado del vehículo y devuelve la orden a enviar a PX4.
// El nodo mission_manager solo traduce entre uORB/DDS y esta clase.
//
// Principio ADR-001: la máquina solo propone órdenes de alto nivel a PX4 (armar, despegar,
// ir a un punto, regresar). Si PX4 entra en failsafe o alguien cambia el modo (piloto),
// pasa a CONTINGENCY y deja de mandar órdenes.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "drone_core/geo.hpp"

namespace drone_mission
{

using drone_core::LatLon;

enum class State : uint8_t {
  PREFLIGHT = 0,
  ARMING,
  TAKEOFF,
  CRUISE,
  APPROACH,
  DROP,
  RETURN,
  CONTINGENCY,
  COMPLETED,
};

enum class Result : uint8_t {
  NONE = 0,
  COMPLETED,
  ABORTED,
  CONTINGENCY,
};

/** Resultado de la suelta pedida a payload_manager. 1..4 coinciden con DropPayload.RESULT_*. */
enum class DropResult : uint8_t {
  NONE = 0,
  RELEASED,
  DENIED,
  TIMEOUT,
  NOT_RELEASED,
  UNAVAILABLE,   ///< payload_manager rechazó el objetivo, no estaba o no respondió a tiempo
};

const char * to_string(State s);
const char * to_string(DropResult r);

/** Valores de nav_state de PX4 v1.17 (VehicleStatus.msg) que usa la máquina. */
namespace nav
{
constexpr uint8_t kPosCtl = 2;
constexpr uint8_t kAutoMission = 3;
constexpr uint8_t kAutoLoiter = 4;
constexpr uint8_t kAutoRtl = 5;
constexpr uint8_t kOffboard = 14;
constexpr uint8_t kAutoTakeoff = 17;
constexpr uint8_t kAutoLand = 18;
}  // namespace nav

struct Params {
  double acceptance_radius_m{3.0};   ///< Llegada a un punto: distancia horizontal
  double alt_tolerance_m{2.0};       ///< Llegada a un punto: error de altura
  double arming_timeout_s{10.0};
  double takeoff_timeout_s{60.0};
  double leg_speed_mps{3.0};         ///< Velocidad conservadora para calcular el tiempo máximo de un tramo
  double leg_timeout_margin_s{30.0};
  bool simulated_drop{false};        ///< true: suelta simulada de S2 (espera drop_wait_s), sin payload_manager
  double drop_wait_s{5.0};           ///< Solo con simulated_drop
  double drop_timeout_s{180.0};      ///< Espera máxima del resultado de la suelta (mayor que confirm_timeout_s de payload_manager)
  double command_resend_s{5.0};      ///< Reenvío de ARM, TAKEOFF y RTL si PX4 no cambia de modo
  double mode_grace_s{3.0};          ///< Tiempo tolerado con un modo inesperado tras una orden
  double setpoint_tolerance_m{5.0};      ///< Consigna de PX4 = objetivo si están a menos de esto
  double setpoint_alt_tolerance_m{3.0};
  double setpoint_mismatch_s{1.0};       ///< Consigna cambiada durante más de esto = intervención externa
  int goto_attempts{3};                  ///< Envíos de GOTO sin confirmación antes de CONTINGENCY
};

/** Plan ya validado por drone_core::validate_mission. Alturas relativas al punto de despegue. */
struct Plan {
  std::string mission_id;
  std::vector<LatLon> route;
  LatLon drop_center;
  double cruise_alt_m{0.0};
  double drop_alt_m{0.0};
};

struct Inputs {
  double now_s{0.0};
  bool status_valid{false};          ///< vehicle_status recibido recientemente
  bool armed{false};
  uint8_t nav_state{0};
  bool failsafe{false};
  bool preflight_checks_pass{false};
  bool landed{true};
  bool pos_valid{false};
  LatLon pos;
  double alt_amsl_m{0.0};
  bool home_valid{false};
  LatLon home;
  double home_alt_amsl_m{0.0};
  bool config_ok{false};             ///< Configuración activa válida y con el mismo hash que la cargada
  bool setpoint_valid{false};        ///< Consigna actual de PX4 (position_setpoint_triplet.current)
  LatLon setpoint;
  double setpoint_alt_amsl_m{0.0};
  bool payload_ready{false};         ///< payload_manager disponible (solo se exige sin simulated_drop)
  bool drop_guard_ok{false};         ///< drop_guard activo y con la zona de la misión (idem)
  DropResult drop_result{DropResult::NONE};  ///< Resultado de la suelta pedida (la limpia el nodo al pedirla)
};

enum class CommandType : uint8_t {
  NONE = 0,
  ARM,
  TAKEOFF,
  GOTO,
  RTL,
  DROP,                   ///< Pedir la suelta a payload_manager (acción DropPayload)
};

struct Command {
  CommandType type{CommandType::NONE};
  LatLon pos;             ///< GOTO
  double alt_amsl_m{0.0}; ///< TAKEOFF y GOTO
};

class MissionStateMachine
{
public:
  explicit MissionStateMachine(const Params & params);

  /** Carga un plan. Solo en PREFLIGHT o COMPLETED. */
  bool set_plan(const Plan & plan, std::string & message);
  bool has_plan() const {return has_plan_;}

  /** Motivos por los que no se puede iniciar con estas entradas (vacío = listo). */
  std::vector<std::string> preflight_blockers(const Inputs & in) const;

  /** Petición de inicio (se ejecuta en el siguiente step). Usa las últimas entradas recibidas. */
  bool request_start(std::string & message);

  /** Petición de aborto: regreso al hub desde cualquier fase de vuelo. */
  bool request_abort(std::string & message);

  /** Avanza la máquina con las entradas actuales y devuelve la orden a enviar (o NONE). */
  Command step(const Inputs & in);

  State state() const {return state_;}
  State previous_state() const {return previous_;}
  Result result() const {return result_;}
  /** Resultado de la suelta de esta misión (NONE si no se llegó a pedir o aún no hay respuesta). */
  DropResult drop_result() const {return drop_result_;}
  const std::string & cause() const {return cause_;}
  uint32_t transition_count() const {return transitions_;}
  std::size_t waypoint_index() const {return target_index_;}
  std::size_t waypoint_count() const {return targets_.size();}
  double distance_to_target_m() const {return distance_to_target_;}

private:
  struct Target {
    LatLon pos;
    double alt_rel_m;
  };

  void transition(State next, const std::string & cause, double now_s);
  Command command_for_state(const Inputs & in);
  Command goto_current_target(const Inputs & in) const;
  bool is_goto_state() const;
  bool setpoint_matches(const Inputs & in) const;
  bool reached(const Inputs & in, const Target & t);
  bool mode_is_expected(uint8_t nav_state) const;
  double leg_timeout_s(const Inputs & in, const Target & t) const;
  bool in_flight_state() const;

  Params p_;
  Plan plan_;
  bool has_plan_{false};
  std::vector<Target> targets_;  ///< Tramos de crucero: ruta + vertical de la zona a altura de crucero

  State state_{State::PREFLIGHT};
  State previous_{State::PREFLIGHT};
  Result result_{Result::NONE};
  std::string cause_{"inicio"};
  uint32_t transitions_{0};

  bool start_requested_{false};
  bool abort_requested_{false};
  bool aborted_{false};
  Inputs last_inputs_{};
  bool have_inputs_{false};

  std::size_t target_index_{0};
  double distance_to_target_{0.0};
  double state_entry_s_{0.0};
  double leg_start_s_{0.0};
  double leg_timeout_{0.0};
  double last_command_s_{-1e9};
  double mode_bad_since_s_{-1.0};

  // Confirmación de la orden GOTO mediante la consigna de PX4
  bool need_goto_{false};
  bool goto_confirmed_{false};
  int goto_sent_count_{0};
  double goto_sent_s_{0.0};
  double sp_mismatch_since_s_{-1.0};

  // Suelta con payload_manager: se pide una sola vez, cuando PX4 ya confirmó el Hold sobre la zona
  bool drop_requested_{false};
  bool drop_request_pending_{false};
  double drop_requested_s_{0.0};
  DropResult drop_result_{DropResult::NONE};
};

}  // namespace drone_mission
