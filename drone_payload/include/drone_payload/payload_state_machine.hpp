// drone_payload — máquina de estados de la suelta de carga (SR-PLD-002, SR-PLD-004, ADR-006, ADR-007). IDAL C.
//
// Clase C++ pura, sin ROS: recibe eventos (inicio, confirmación del piloto, ack de PX4) y el estado
// del vehículo, y devuelve la orden a enviar. El nodo payload_manager solo traduce entre DDS y esta clase.
//
// Es el miembro ROS 2 de la función de suelta (ADR-007). El otro es el módulo drop_guard de PX4, que
// vuelve a comprobar la zona por su cuenta: aquí nunca se asume que PX4 va a vetar la orden.
//
// Secuencia: WAITING_CONFIRMATION -> RELEASING -> VERIFYING -> resultado. La orden de apertura se envía
// UNA sola vez por suelta; no se reintenta (una apertura no es idempotente en sentido de seguridad).
#pragma once

#include <cstdint>
#include <string>

#include "drone_core/geo.hpp"
#include "drone_core/ops_config.hpp"

namespace drone_payload
{

enum class Phase : uint8_t {
  IDLE = 0,
  WAITING_CONFIRMATION,
  RELEASING,
  VERIFYING,
};

/** Coincide con los RESULT_* de DropPayload.action y los LAST_* de PayloadState.msg. */
enum class Outcome : uint8_t {
  NONE = 0,
  RELEASED,
  DENIED,
  TIMEOUT,
  NOT_RELEASED,
};

const char * to_string(Phase p);
const char * to_string(Outcome o);

/** Nombre del motivo de rechazo de drop_guard (result_param2 del ack; drop_guard::Reason). */
const char * drop_guard_reason_name(int32_t reason);

struct Params {
  double confirm_timeout_s{120.0};   ///< Espera máxima de la confirmación del piloto (SIM-05)
  double ack_timeout_s{2.0};         ///< Espera del ack de PX4 tras la orden de apertura
  double verify_timeout_s{3.0};      ///< TBD-1 (SR-PLD-004): tiempo máximo para ver la carga liberada
  double position_timeout_s{1.0};    ///< Antigüedad máxima de la posición
};

struct Inputs {
  double now_s{0.0};
  bool armed{false};
  bool pos_valid{false};             ///< lat/lon y altura válidas en el último mensaje recibido
  double pos_age_s{1e9};
  drone_core::LatLon pos;
  double alt_amsl_m{0.0};
  double eph_m{0.0};
  double epv_m{0.0};
  bool home_valid{false};
  double home_alt_amsl_m{0.0};
  bool payload_valid{false};         ///< El sensor de carga tiene una lectura válida
  bool payload_present{false};       ///< Hay carga en el mecanismo
  bool dg_valid{false};              ///< drop_guard_status recibido recientemente
  int32_t dg_zone_hash{0};           ///< DG_ZONE_HASH en uso en PX4
};

enum class CommandType : uint8_t {
  NONE = 0,
  RELEASE,                           ///< VEHICLE_CMD_DO_GRIPPER con GRIPPER_ACTION_RELEASE
};

struct Command {
  CommandType type{CommandType::NONE};
};

class PayloadStateMachine
{
public:
  explicit PayloadStateMachine(const Params & params);

  /** Inicia una suelta sobre la zona. Solo en IDLE. zone_hash es el DG_ZONE_HASH esperado en PX4. */
  bool start(const drone_core::DropZone & zone, int32_t zone_hash, double now_s, std::string & message);

  /** Confirmación del piloto (ADR-006). Solo se acepta en WAITING_CONFIRMATION; se evalúa en el siguiente step. */
  bool confirm(std::string & message);

  /** Cancela la suelta antes de ordenar la apertura. Una vez ordenada ya no se puede deshacer. */
  bool cancel(std::string & message);

  /** Resultado del ack de PX4 a la orden de apertura. Solo cuenta en RELEASING. */
  void on_ack(bool accepted, int32_t reason);

  /** Avanza la máquina con las entradas actuales y devuelve la orden a enviar (o NONE). */
  Command step(const Inputs & in);

  Phase phase() const {return phase_;}
  /** Resultado de la última suelta terminada; NONE mientras hay una en curso. */
  Outcome outcome() const {return outcome_;}
  /** Último resultado desde el arranque (no se borra al iniciar otra suelta). */
  Outcome last_outcome() const {return last_outcome_;}
  const std::string & message() const {return message_;}
  const std::string & zone_id() const {return zone_.id;}
  /** DG_ZONE_HASH esperado en PX4 para la zona de la suelta en curso o de la última. */
  int32_t zone_hash() const {return zone_hash_;}
  /** true si se ha ordenado la apertura en la suelta actual o en la última terminada. */
  bool release_commanded() const {return release_commanded_;}

private:
  void finish(Outcome o, const std::string & message);
  std::string release_blocker(const Inputs & in) const;

  Params p_;
  drone_core::DropZone zone_;
  int32_t zone_hash_{0};

  Phase phase_{Phase::IDLE};
  Outcome outcome_{Outcome::NONE};
  Outcome last_outcome_{Outcome::NONE};
  std::string message_;

  double entry_s_{0.0};              ///< Instante de entrada en WAITING_CONFIRMATION
  double command_s_{0.0};            ///< Instante de la orden de apertura
  bool confirm_pending_{false};
  bool release_commanded_{false};
  bool ack_received_{false};
  bool ack_accepted_{false};
  int32_t ack_reason_{0};
};

}  // namespace drone_payload
