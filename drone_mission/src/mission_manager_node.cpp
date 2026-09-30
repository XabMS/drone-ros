// mission_manager — ejecuta una misión de reparto con la máquina de estados del ConOps §5 (SR-MSN-001).
//
// Traduce entre PX4 (uXRCE-DDS) y drone_mission::MissionStateMachine. No decide nada por sí mismo.
//
// Parámetros principales: ops_dir, missions_dir, mission_id y los de la máquina de estados.
// Publica:   /drone/mission/state (R-01) y /fmu/in/vehicle_command.
// Servicio:  ~/command (drone_interfaces/MissionCommand: START, ABORT).
// Suelta:    en DROP pide la suelta a payload_manager con la acción /drone/payload/drop (R-02). Con
//            simulated_drop:=true no la usa y espera drop_wait_s (comportamiento de S2).

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "drone_core/mission.hpp"
#include "drone_core/ops_config.hpp"
#include "drone_interfaces/action/drop_payload.hpp"
#include "drone_interfaces/msg/active_config.hpp"
#include "drone_interfaces/msg/mission_state.hpp"
#include "drone_interfaces/msg/payload_state.hpp"
#include "drone_interfaces/srv/mission_command.hpp"
#include "drone_mission/mission_state_machine.hpp"
#include "px4_msgs/msg/drop_guard_status.hpp"
#include "px4_msgs/msg/home_position.hpp"
#include "px4_msgs/msg/position_setpoint_triplet.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace drone_mission
{

using DropPayload = drone_interfaces::action::DropPayload;
using DropGoalHandle = rclcpp_action::ClientGoalHandle<DropPayload>;

/** Valores de drop_guard::Reason y del estado del módulo que interesan al prevuelo (DropGuardStatus.msg). */
constexpr uint8_t kDropGuardReasonDisabled = 1;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager()
  : Node("mission_manager"), sm_(read_params())
  {
    const std::string ops_dir = declare_parameter<std::string>("ops_dir", "/ops");
    const std::string missions_dir = declare_parameter<std::string>("missions_dir", "/missions");
    const std::string mission_id = declare_parameter<std::string>("mission_id", "tls_demo_01");
    status_timeout_s_ = declare_parameter<double>("status_timeout_s", 1.0);
    const auto t_status = declare_parameter<std::string>("topic_vehicle_status", "/fmu/out/vehicle_status_v1");
    const auto t_gpos = declare_parameter<std::string>("topic_global_position", "/fmu/out/vehicle_global_position");
    const auto t_home = declare_parameter<std::string>("topic_home_position", "/fmu/out/home_position_v1");
    const auto t_land = declare_parameter<std::string>("topic_land_detected", "/fmu/out/vehicle_land_detected");
    const auto t_sp = declare_parameter<std::string>("topic_setpoint_triplet", "/fmu/out/position_setpoint_triplet");
    const auto t_dg = declare_parameter<std::string>("topic_drop_guard", "/fmu/out/drop_guard_status");
    payload_state_timeout_s_ = declare_parameter<double>("payload_state_timeout_s", 2.0);

    load_mission(ops_dir, missions_dir, mission_id);

    const auto px4_qos = rclcpp::QoS(1).best_effort().transient_local();
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      t_status, px4_qos, [this](const px4_msgs::msg::VehicleStatus & m) {
        status_ = m;
        status_rx_s_ = now_s();
      });
    gpos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      t_gpos, px4_qos, [this](const px4_msgs::msg::VehicleGlobalPosition & m) {gpos_ = m; have_gpos_ = true;});
    home_sub_ = create_subscription<px4_msgs::msg::HomePosition>(
      t_home, px4_qos, [this](const px4_msgs::msg::HomePosition & m) {home_ = m; have_home_ = true;});
    land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      t_land, px4_qos, [this](const px4_msgs::msg::VehicleLandDetected & m) {landed_ = m.landed;});
    sp_sub_ = create_subscription<px4_msgs::msg::PositionSetpointTriplet>(
      t_sp, px4_qos, [this](const px4_msgs::msg::PositionSetpointTriplet & m) {sp_ = m; have_sp_ = true;});
    dg_sub_ = create_subscription<px4_msgs::msg::DropGuardStatus>(
      t_dg, px4_qos, [this](const px4_msgs::msg::DropGuardStatus & m) {dg_ = m; dg_rx_s_ = now_s();});
    payload_state_sub_ = create_subscription<drone_interfaces::msg::PayloadState>(
      "/drone/payload/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](const drone_interfaces::msg::PayloadState &) {payload_rx_s_ = now_s();});
    drop_client_ = rclcpp_action::create_client<DropPayload>(this, "/drone/payload/drop");
    config_sub_ = create_subscription<drone_interfaces::msg::ActiveConfig>(
      "/drone/config/active", rclcpp::QoS(1).reliable().transient_local(),
      [this](const drone_interfaces::msg::ActiveConfig & m) {active_ = m; have_active_ = true;});

    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort().transient_local());
    state_pub_ = create_publisher<drone_interfaces::msg::MissionState>(
      "/drone/mission/state", rclcpp::QoS(1).reliable().transient_local());

    cmd_srv_ = create_service<drone_interfaces::srv::MissionCommand>(
      "~/command",
      [this](const std::shared_ptr<drone_interfaces::srv::MissionCommand::Request> req,
      std::shared_ptr<drone_interfaces::srv::MissionCommand::Response> res) {
        std::string msg;
        if (req->command == drone_interfaces::srv::MissionCommand::Request::START) {
          res->accepted = sm_.request_start(msg);
        } else if (req->command == drone_interfaces::srv::MissionCommand::Request::ABORT) {
          res->accepted = sm_.request_abort(msg);
        } else {
          res->accepted = false;
          msg = "orden desconocida";
        }
        res->message = msg;
        RCLCPP_INFO(get_logger(), "Orden %u: %s (%s)", req->command, res->accepted ? "aceptada" : "rechazada",
          msg.c_str());
      });

    timer_ = create_wall_timer(100ms, [this]() {step();});
    RCLCPP_INFO(get_logger(), "mission_manager listo. Misión '%s'.", mission_id.c_str());
  }

private:
  Params read_params()
  {
    Params p;
    p.acceptance_radius_m = declare_parameter<double>("acceptance_radius_m", p.acceptance_radius_m);
    p.alt_tolerance_m = declare_parameter<double>("alt_tolerance_m", p.alt_tolerance_m);
    p.arming_timeout_s = declare_parameter<double>("arming_timeout_s", p.arming_timeout_s);
    p.takeoff_timeout_s = declare_parameter<double>("takeoff_timeout_s", p.takeoff_timeout_s);
    p.leg_speed_mps = declare_parameter<double>("leg_speed_mps", p.leg_speed_mps);
    p.leg_timeout_margin_s = declare_parameter<double>("leg_timeout_margin_s", p.leg_timeout_margin_s);
    p.simulated_drop = declare_parameter<bool>("simulated_drop", p.simulated_drop);
    p.drop_wait_s = declare_parameter<double>("drop_wait_s", p.drop_wait_s);
    p.drop_timeout_s = declare_parameter<double>("drop_timeout_s", p.drop_timeout_s);
    p.command_resend_s = declare_parameter<double>("command_resend_s", p.command_resend_s);
    p.mode_grace_s = declare_parameter<double>("mode_grace_s", p.mode_grace_s);
    p.setpoint_tolerance_m = declare_parameter<double>("setpoint_tolerance_m", p.setpoint_tolerance_m);
    p.setpoint_alt_tolerance_m = declare_parameter<double>("setpoint_alt_tolerance_m", p.setpoint_alt_tolerance_m);
    p.setpoint_mismatch_s = declare_parameter<double>("setpoint_mismatch_s", p.setpoint_mismatch_s);
    p.goto_attempts = static_cast<int>(declare_parameter<int64_t>("goto_attempts", p.goto_attempts));
    return p;
  }

  void load_mission(const std::string & ops_dir, const std::string & missions_dir, const std::string & id)
  {
    const drone_core::MissionLoadResult m = drone_core::load_mission(missions_dir, id);
    if (!m.ok) {
      for (const auto & e : m.errors) {
        RCLCPP_ERROR(get_logger(), "%s", e.c_str());
      }
      return;
    }
    const drone_core::OpsLoadResult c = drone_core::load_ops_config(ops_dir, m.plan.city);
    if (!c.ok) {
      RCLCPP_ERROR(get_logger(), "Configuración de '%s' no válida: la misión no se carga", m.plan.city.c_str());
      return;
    }
    const drone_core::MissionValidation v = drone_core::validate_mission(m.plan, c.config);
    for (const auto & w : v.warnings) {
      RCLCPP_WARN(get_logger(), "Aviso: %s", w.c_str());
    }
    if (!v.ok) {
      RCLCPP_ERROR(get_logger(), "Misión '%s' rechazada:", id.c_str());
      for (const auto & e : v.errors) {
        RCLCPP_ERROR(get_logger(), "  - %s", e.c_str());
      }
      return;
    }
    Plan plan;
    plan.mission_id = m.plan.id;
    plan.route = m.plan.route;
    plan.drop_center = v.drop_zone.center;
    plan.cruise_alt_m = m.plan.cruise_alt_m;
    plan.drop_alt_m = 0.5 * (v.drop_zone.alt_min_m + v.drop_zone.alt_max_m);
    std::string msg;
    if (sm_.set_plan(plan, msg)) {
      drop_zone_id_ = v.drop_zone.id;
      // DG_ZONE_HASH es un int32 con el CRC32 de la zona (drone_core::drop_zone_hash)
      drop_zone_hash_ = static_cast<int32_t>(drone_core::drop_zone_hash(v.drop_zone));
      loaded_ops_hash_ = c.config.ops_hash;
      loaded_city_ = c.config.city;
      mission_id_ = plan.mission_id;
      RCLCPP_INFO(get_logger(), "Misión '%s' válida: %.0f m ida + vuelta, suelta en '%s' a %.1f m, hash de ciudad 0x%08X",
        id.c_str(), v.total_m, v.drop_zone.id.c_str(), plan.drop_alt_m, loaded_ops_hash_);
    }
  }

  double now_s() {return now().seconds();}

  Inputs build_inputs()
  {
    Inputs in;
    in.now_s = now_s();
    in.status_valid = (status_rx_s_ > 0.0) && (in.now_s - status_rx_s_ < status_timeout_s_);
    in.armed = (status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    in.nav_state = status_.nav_state;
    in.failsafe = status_.failsafe;
    in.preflight_checks_pass = status_.pre_flight_checks_pass;
    in.landed = landed_;
    in.pos_valid = have_gpos_ && gpos_.lat_lon_valid && gpos_.alt_valid;
    in.pos = LatLon{gpos_.lat, gpos_.lon};
    in.alt_amsl_m = gpos_.alt;
    in.home_valid = have_home_ && home_.valid_hpos && home_.valid_alt;
    in.home = LatLon{home_.lat, home_.lon};
    in.home_alt_amsl_m = home_.alt;
    in.config_ok = have_active_ && active_.valid && sm_.has_plan() && active_.city == loaded_city_ &&
      active_.ops_hash == loaded_ops_hash_;
    in.setpoint_valid = have_sp_ && sp_.current.valid;
    in.setpoint = LatLon{sp_.current.lat, sp_.current.lon};
    in.setpoint_alt_amsl_m = sp_.current.alt;
    in.payload_ready = drop_client_->action_server_is_ready() && payload_rx_s_ > 0.0 &&
      (in.now_s - payload_rx_s_ < payload_state_timeout_s_);
    // drop_guard activo (DG_ENABLE) y con la zona de esta misión (DG_ZONE_HASH): el piloto lo verifica en prevuelo
    in.drop_guard_ok = (dg_rx_s_ > 0.0) && (in.now_s - dg_rx_s_ < status_timeout_s_) &&
      dg_.state != px4_msgs::msg::DropGuardStatus::STATE_ARMED_FAULT &&
      dg_.reason != kDropGuardReasonDisabled && dg_.zone_hash == drop_zone_hash_;
    in.drop_result = drop_result_;
    return in;
  }

  /** Pide la suelta a payload_manager. El resultado llega por drop_result_ (lo lee build_inputs). */
  void request_drop()
  {
    drop_result_ = DropResult::NONE;
    drop_cancel_sent_ = false;
    drop_goal_.reset();
    if (!drop_client_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "payload_manager no disponible: no se puede pedir la suelta.");
      drop_result_ = DropResult::UNAVAILABLE;
      return;
    }
    DropPayload::Goal goal;
    goal.drop_zone_id = drop_zone_id_;
    auto opts = rclcpp_action::Client<DropPayload>::SendGoalOptions();
    opts.goal_response_callback = [this](const DropGoalHandle::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "payload_manager rechazó la suelta en '%s'.", drop_zone_id_.c_str());
          drop_result_ = DropResult::UNAVAILABLE;
        } else {
          drop_goal_ = gh;
        }
      };
    opts.result_callback = [this](const DropGoalHandle::WrappedResult & r) {
        drop_goal_.reset();
        if (r.code == rclcpp_action::ResultCode::CANCELED) {
          RCLCPP_INFO(get_logger(), "Suelta cancelada en payload_manager.");
          return;   // la cancelamos nosotros al salir de DROP: no hay resultado que registrar
        }
        const uint8_t v = (r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result) ? r.result->result : 0;
        drop_result_ = (v >= 1 && v <= 4) ? static_cast<DropResult>(v) : DropResult::UNAVAILABLE;
        RCLCPP_INFO(get_logger(), "Resultado de la suelta: %s (%s)", to_string(drop_result_),
          r.result ? r.result->message.c_str() : "sin mensaje");
      };
    drop_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(get_logger(), "Suelta pedida a payload_manager en '%s'.", drop_zone_id_.c_str());
  }

  void send(const Command & c)
  {
    if (c.type == CommandType::NONE) {
      return;
    }
    if (c.type == CommandType::DROP) {
      request_drop();
      return;
    }
    px4_msgs::msg::VehicleCommand v{};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    switch (c.type) {
      case CommandType::ARM:
        v.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        v.param1 = 1.0f;
        break;
      case CommandType::TAKEOFF:
        v.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
        v.param4 = nan;                                   // rumbo actual
        v.param5 = std::numeric_limits<double>::quiet_NaN();  // posición actual
        v.param6 = std::numeric_limits<double>::quiet_NaN();
        v.param7 = static_cast<float>(c.alt_amsl_m);
        break;
      case CommandType::GOTO:
        v.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
        v.param1 = -1.0f;  // velocidad por defecto
        v.param2 = 1.0f;   // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: pasa a Hold si hace falta
        v.param3 = 0.0f;
        v.param4 = nan;    // sin cambio de rumbo
        v.param5 = c.pos.lat_deg;
        v.param6 = c.pos.lon_deg;
        v.param7 = static_cast<float>(c.alt_amsl_m);
        break;
      case CommandType::RTL:
        v.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
        break;
      case CommandType::DROP:
      case CommandType::NONE:
        return;
    }
    v.target_system = 1;
    v.target_component = 1;
    v.source_system = 1;
    v.source_component = 191;  // companion (DOC-06 §4)
    v.from_external = true;
    v.timestamp = static_cast<uint64_t>(now().nanoseconds() / 1000);
    cmd_pub_->publish(v);
  }

  void step()
  {
    const Inputs in = build_inputs();
    const uint32_t before = sm_.transition_count();
    send(sm_.step(in));

    // Si la misión sale de DROP con la suelta en curso (aborto, contingencia), se pide cancelarla. Si
    // payload_manager ya ordenó la apertura la rechazará y su resultado llegará tarde.
    if (drop_goal_ && sm_.state() != State::DROP && !drop_cancel_sent_) {
      drop_cancel_sent_ = true;
      drop_client_->async_cancel_goal(drop_goal_);
      RCLCPP_INFO(get_logger(), "Fuera de DROP: se pide cancelar la suelta.");
    }

    const bool changed = sm_.transition_count() != before;
    if (changed) {
      RCLCPP_INFO(get_logger(), "%s -> %s: %s", to_string(sm_.previous_state()), to_string(sm_.state()),
        sm_.cause().c_str());
    }
    if (changed || ++publish_divider_ >= 5) {  // 2 Hz + en cada transición
      publish_divider_ = 0;
      publish_state(in);
    }
  }

  void publish_state(const Inputs & in)
  {
    drone_interfaces::msg::MissionState m;
    m.stamp = now();
    m.state = static_cast<uint8_t>(sm_.state());
    m.previous_state = static_cast<uint8_t>(sm_.previous_state());
    m.state_name = to_string(sm_.state());
    m.cause = sm_.cause();
    m.mission_id = mission_id_;
    m.waypoint_index = static_cast<uint16_t>(sm_.waypoint_index());
    m.waypoint_count = static_cast<uint16_t>(sm_.waypoint_count());
    m.distance_to_target_m = static_cast<float>(sm_.distance_to_target_m());
    m.preflight_blockers = sm_.preflight_blockers(in);
    m.ready_to_start = m.preflight_blockers.empty() &&
      (sm_.state() == State::PREFLIGHT || sm_.state() == State::COMPLETED);
    m.result = static_cast<uint8_t>(sm_.result());
    m.drop_result = static_cast<uint8_t>(sm_.drop_result());
    state_pub_->publish(m);
  }

  MissionStateMachine sm_;
  std::string mission_id_;
  std::string loaded_city_;
  uint32_t loaded_ops_hash_{0};
  double status_timeout_s_{1.0};
  int publish_divider_{0};

  px4_msgs::msg::VehicleStatus status_{};
  double status_rx_s_{0.0};
  px4_msgs::msg::VehicleGlobalPosition gpos_{};
  bool have_gpos_{false};
  px4_msgs::msg::HomePosition home_{};
  bool have_home_{false};
  bool landed_{true};
  px4_msgs::msg::PositionSetpointTriplet sp_{};
  bool have_sp_{false};
  drone_interfaces::msg::ActiveConfig active_{};
  bool have_active_{false};
  px4_msgs::msg::DropGuardStatus dg_{};
  double dg_rx_s_{0.0};
  double payload_rx_s_{0.0};
  double payload_state_timeout_s_{2.0};
  std::string drop_zone_id_;
  int32_t drop_zone_hash_{0};
  DropResult drop_result_{DropResult::NONE};
  DropGoalHandle::SharedPtr drop_goal_;
  bool drop_cancel_sent_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gpos_sub_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr sp_sub_;
  rclcpp::Subscription<drone_interfaces::msg::ActiveConfig>::SharedPtr config_sub_;
  rclcpp::Subscription<px4_msgs::msg::DropGuardStatus>::SharedPtr dg_sub_;
  rclcpp::Subscription<drone_interfaces::msg::PayloadState>::SharedPtr payload_state_sub_;
  rclcpp_action::Client<DropPayload>::SharedPtr drop_client_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<drone_interfaces::msg::MissionState>::SharedPtr state_pub_;
  rclcpp::Service<drone_interfaces::srv::MissionCommand>::SharedPtr cmd_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_mission::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
