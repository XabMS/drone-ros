// payload_manager — gestiona la suelta de carga (SR-PLD-002, SR-PLD-004, ADR-006, ADR-007).
//
// Traduce entre PX4 (uXRCE-DDS), mission_manager y drone_payload::PayloadStateMachine. No decide nada por sí mismo.
//
// Parámetros: ops_dir, city (zona de suelta), sensor_mode ("sim"), sim_stuck (simula una carga atascada, SIM-07)
// y los de la máquina de estados.
// Acción:    /drone/payload/drop (R-02, drone_interfaces/DropPayload), la llama mission_manager.
// Servicios: ~/confirm (R-04, confirmación del piloto), ~/load_payload (solo sensor_mode "sim": repone la carga).
// Publica:   /drone/payload/state (R-03) y /fmu/in/vehicle_command (DO_GRIPPER RELEASE).
//
// Sensor de carga (SR-PLD-004): sin hardware, en modo "sim" se deduce de drop_guard_status.release_count. El
// microinterruptor real (GPIO del companion) sustituirá a esta clase de entrada sin tocar la máquina de estados.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "drone_core/ops_config.hpp"
#include "drone_interfaces/action/drop_payload.hpp"
#include "drone_interfaces/msg/payload_state.hpp"
#include "drone_interfaces/srv/confirm_drop.hpp"
#include "drone_payload/payload_state_machine.hpp"
#include "px4_msgs/msg/drop_guard_status.hpp"
#include "px4_msgs/msg/home_position.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace drone_payload
{

using DropPayload = drone_interfaces::action::DropPayload;
using GoalHandle = rclcpp_action::ServerGoalHandle<DropPayload>;

class PayloadManager : public rclcpp::Node
{
public:
  PayloadManager()
  : Node("payload_manager"), sm_(read_params())
  {
    const std::string ops_dir = declare_parameter<std::string>("ops_dir", "/ops");
    const std::string city = declare_parameter<std::string>("city", "toulouse");
    sensor_mode_ = declare_parameter<std::string>("sensor_mode", "sim");
    declare_parameter<bool>("sim_stuck", false);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 1.0);
    const auto t_status = declare_parameter<std::string>("topic_vehicle_status", "/fmu/out/vehicle_status_v1");
    const auto t_gpos = declare_parameter<std::string>("topic_global_position", "/fmu/out/vehicle_global_position");
    const auto t_home = declare_parameter<std::string>("topic_home_position", "/fmu/out/home_position_v1");
    const auto t_ack = declare_parameter<std::string>("topic_command_ack", "/fmu/out/vehicle_command_ack");
    const auto t_dg = declare_parameter<std::string>("topic_drop_guard", "/fmu/out/drop_guard_status");

    load_ops(ops_dir, city);

    const auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      t_status, px4_qos, [this](const px4_msgs::msg::VehicleStatus & m) {
        armed_ = (m.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
        status_rx_s_ = now_s();
      });
    gpos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      t_gpos, px4_qos, [this](const px4_msgs::msg::VehicleGlobalPosition & m) {
        gpos_ = m;
        gpos_rx_s_ = now_s();
      });
    home_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
      t_home, px4_qos, [this](const px4_msgs::msg::HomePosition & m) {home_ = m; have_home_ = true;});
    dg_sub_ = create_subscription<px4_msgs::msg::DropGuardStatus>(
      t_dg, px4_qos, [this](const px4_msgs::msg::DropGuardStatus & m) {
        dg_ = m;
        dg_rx_s_ = now_s();
        if (!have_dg_ || m.release_count < baseline_release_count_) {
          baseline_release_count_ = m.release_count;   // primer estado, o PX4 reiniciado
        }
        have_dg_ = true;
      });
    // Solo interesan los acks de DO_GRIPPER; PX4 los publica sin sufijo de versión (MESSAGE_VERSION = 0).
    ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      t_ack, rclcpp::QoS(10).best_effort(), [this](const px4_msgs::msg::VehicleCommandAck & m) {
        if (m.command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER) {
          return;
        }
        sm_.on_ack(m.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED, m.result_param2);
      });

    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort().transient_local());
    state_pub_ = create_publisher<drone_interfaces::msg::PayloadState>(
      "/drone/payload/state", rclcpp::QoS(1).reliable().transient_local());

    action_srv_ = rclcpp_action::create_server<DropPayload>(
      this, "/drone/payload/drop",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const DropPayload::Goal> goal) {
        return on_goal(*goal);
      },
      [this](const std::shared_ptr<GoalHandle>) {
        std::string msg;
        const bool ok = sm_.cancel(msg);
        RCLCPP_INFO(get_logger(), "Cancelación: %s (%s)", ok ? "aceptada" : "rechazada", msg.c_str());
        return ok ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle> gh) {goal_ = gh;});

    confirm_srv_ = create_service<drone_interfaces::srv::ConfirmDrop>(
      "~/confirm",
      [this](const std::shared_ptr<drone_interfaces::srv::ConfirmDrop::Request> req,
      std::shared_ptr<drone_interfaces::srv::ConfirmDrop::Response> res) {
        std::string msg;
        if (goal_ && req->drop_zone_id != sm_.zone_id()) {
          res->accepted = false;
          msg = "la zona no coincide con la suelta en curso";
        } else {
          res->accepted = sm_.confirm(msg);
        }
        res->message = msg;
        RCLCPP_INFO(get_logger(), "Confirmación del piloto para '%s': %s (%s)", req->drop_zone_id.c_str(),
          res->accepted ? "aceptada" : "rechazada", msg.c_str());
      });

    load_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/load_payload",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (sensor_mode_ != "sim" || !have_dg_) {
          res->success = false;
          res->message = "solo con sensor_mode sim y drop_guard_status recibido";
        } else {
          baseline_release_count_ = dg_.release_count;
          res->success = true;
          res->message = "carga repuesta";
        }
      });

    timer_ = create_wall_timer(100ms, [this]() {step();});
    RCLCPP_INFO(get_logger(), "payload_manager listo (ciudad '%s', sensor '%s').", city.c_str(), sensor_mode_.c_str());
  }

private:
  Params read_params()
  {
    Params p;
    p.confirm_timeout_s = declare_parameter<double>("confirm_timeout_s", p.confirm_timeout_s);
    p.ack_timeout_s = declare_parameter<double>("ack_timeout_s", p.ack_timeout_s);
    p.verify_timeout_s = declare_parameter<double>("verify_timeout_s", p.verify_timeout_s);
    p.position_timeout_s = declare_parameter<double>("position_timeout_s", p.position_timeout_s);
    return p;
  }

  double now_s() {return now().seconds();}

  void load_ops(const std::string & ops_dir, const std::string & city)
  {
    const auto r = drone_core::load_ops_config(ops_dir, city);
    if (!r.ok) {
      for (const auto & e : r.errors) {
        RCLCPP_ERROR(get_logger(), "Configuración '%s': %s", city.c_str(), e.c_str());
      }
      RCLCPP_ERROR(get_logger(), "Sin configuración válida: se rechazarán las sueltas.");
      return;
    }
    const auto problems = drone_core::validate_ops_config(r.config);
    if (!problems.empty()) {
      for (const auto & e : problems) {
        RCLCPP_ERROR(get_logger(), "Configuración '%s': %s", city.c_str(), e.c_str());
      }
      RCLCPP_ERROR(get_logger(), "Sin configuración válida: se rechazarán las sueltas.");
      return;
    }
    ops_ = r.config;
    ops_ok_ = true;
  }

  rclcpp_action::GoalResponse on_goal(const DropPayload::Goal & goal)
  {
    if (!ops_ok_) {
      RCLCPP_WARN(get_logger(), "Suelta rechazada: sin configuración válida.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const drone_core::DropZone * zone = ops_.find_drop_zone(goal.drop_zone_id);
    if (zone == nullptr) {
      RCLCPP_WARN(get_logger(), "Suelta rechazada: la zona '%s' no existe.", goal.drop_zone_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::string msg;
    // DG_ZONE_HASH es un int32 con el CRC32 de la zona (drone_core::drop_zone_hash)
    const auto hash = static_cast<int32_t>(drone_core::drop_zone_hash(*zone));
    if (!sm_.start(*zone, hash, now_s(), msg)) {
      RCLCPP_WARN(get_logger(), "Suelta rechazada: %s.", msg.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Suelta en '%s': %s.", zone->id.c_str(), msg.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  Inputs build_inputs()
  {
    Inputs in;
    in.now_s = now_s();
    in.armed = (status_rx_s_ > 0.0) && (in.now_s - status_rx_s_ < input_timeout_s_) && armed_;
    in.pos_valid = (gpos_rx_s_ > 0.0) && gpos_.lat_lon_valid && gpos_.alt_valid;
    in.pos_age_s = (gpos_rx_s_ > 0.0) ? in.now_s - gpos_rx_s_ : 1e9;
    in.pos = drone_core::LatLon{gpos_.lat, gpos_.lon};
    in.alt_amsl_m = static_cast<double>(gpos_.alt);
    in.eph_m = static_cast<double>(gpos_.eph);
    in.epv_m = static_cast<double>(gpos_.epv);
    in.home_valid = have_home_ && home_.valid_alt;
    in.home_alt_amsl_m = static_cast<double>(home_.alt);
    in.dg_valid = have_dg_ && (in.now_s - dg_rx_s_ < input_timeout_s_);
    in.dg_zone_hash = dg_.zone_hash;
    read_payload_sensor(in);
    return in;
  }

  /** Sensor de carga. "sim": hay carga hasta que drop_guard autoriza una apertura, salvo carga atascada. */
  void read_payload_sensor(Inputs & in)
  {
    if (sensor_mode_ != "sim" || !have_dg_) {
      in.payload_valid = false;
      return;
    }
    in.payload_valid = true;
    const bool released = dg_.release_count > baseline_release_count_;
    const bool stuck = get_parameter("sim_stuck").as_bool();
    in.payload_present = !released || stuck;
  }

  void send(const Command & c)
  {
    if (c.type != CommandType::RELEASE) {
      return;
    }
    px4_msgs::msg::VehicleCommand v{};
    v.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER;
    v.param1 = 0.0f;   // instancia del gripper
    v.param2 = static_cast<float>(px4_msgs::msg::VehicleCommand::GRIPPER_ACTION_RELEASE);
    v.target_system = 1;
    v.target_component = 1;
    v.source_system = 1;
    v.source_component = 191;  // companion (DOC-06 §4)
    v.from_external = true;
    v.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    cmd_pub_->publish(v);
    RCLCPP_INFO(get_logger(), "DO_GRIPPER RELEASE enviado a PX4 (drop_guard decide).");
  }

  void step()
  {
    const Inputs in = build_inputs();
    const Phase before = sm_.phase();
    send(sm_.step(in));
    const Phase after = sm_.phase();
    if (after != before) {
      RCLCPP_INFO(get_logger(), "%s -> %s%s%s", to_string(before), to_string(after),
        sm_.outcome() != Outcome::NONE ? ": " : "", sm_.outcome() != Outcome::NONE ? sm_.message().c_str() : "");
    }
    drive_goal();
    if (after != before || ++publish_divider_ >= 5) {   // 2 Hz + en cada cambio
      publish_divider_ = 0;
      publish_state(in);
    }
  }

  /** Feedback mientras dura la suelta y resultado al terminar. */
  void drive_goal()
  {
    if (!goal_) {
      return;
    }
    if (sm_.phase() != Phase::IDLE) {
      auto fb = std::make_shared<DropPayload::Feedback>();
      fb->phase = static_cast<uint8_t>(sm_.phase());   // Phase 1..3 = PHASE_WAITING_CONFIRMATION..PHASE_VERIFYING
      goal_->publish_feedback(fb);
      return;
    }
    auto result = std::make_shared<DropPayload::Result>();
    result->message = sm_.message();
    if (goal_->is_canceling()) {
      goal_->canceled(result);
    } else if (sm_.outcome() != Outcome::NONE) {
      result->result = static_cast<uint8_t>(sm_.outcome());
      goal_->succeed(result);   // la acción "termina bien" aunque el resultado sea DENIED: el motivo va en result
    } else {
      return;   // aún sin resultado (goal recién aceptado)
    }
    goal_.reset();
  }

  void publish_state(const Inputs & in)
  {
    drone_interfaces::msg::PayloadState m;
    m.stamp = now();
    m.payload_present = in.payload_valid && in.payload_present;
    if (!have_dg_) {
      m.mechanism = drone_interfaces::msg::PayloadState::MECHANISM_UNKNOWN;
    } else if (dg_.release_count > baseline_release_count_) {
      m.mechanism = drone_interfaces::msg::PayloadState::MECHANISM_OPEN;
    } else {
      m.mechanism = drone_interfaces::msg::PayloadState::MECHANISM_CLOSED;
    }
    m.last_result = static_cast<uint8_t>(sm_.last_outcome());
    state_pub_->publish(m);
  }

  PayloadStateMachine sm_;
  drone_core::OpsConfig ops_;
  bool ops_ok_{false};
  std::string sensor_mode_;
  double input_timeout_s_{1.0};

  bool armed_{false};
  double status_rx_s_{0.0};
  px4_msgs::msg::VehicleGlobalPosition gpos_;
  double gpos_rx_s_{0.0};
  px4_msgs::msg::HomePosition home_;
  bool have_home_{false};
  px4_msgs::msg::DropGuardStatus dg_;
  double dg_rx_s_{0.0};
  bool have_dg_{false};
  uint32_t baseline_release_count_{0};
  int publish_divider_{0};

  std::shared_ptr<GoalHandle> goal_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gpos_sub_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<px4_msgs::msg::DropGuardStatus>::SharedPtr dg_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<drone_interfaces::msg::PayloadState>::SharedPtr state_pub_;
  rclcpp_action::Server<DropPayload>::SharedPtr action_srv_;
  rclcpp::Service<drone_interfaces::srv::ConfirmDrop>::SharedPtr confirm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_payload

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_payload::PayloadManager>());
  rclcpp::shutdown();
  return 0;
}
