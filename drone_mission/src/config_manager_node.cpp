// config_manager — carga y valida la configuración de operación de una ciudad (AR-019, SR-MSN-002)
// y la publica en /drone/config/active (R-05) con QoS RELIABLE + TRANSIENT_LOCAL.
//
// Parámetros: ops_dir (directorio de ops/), city (nombre de la ciudad).
// Servicio:   ~/reload (std_srvs/Trigger) vuelve a leer los ficheros.

#include <memory>
#include <string>

#include "drone_core/ops_config.hpp"
#include "drone_interfaces/msg/active_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace drone_mission
{

class ConfigManager : public rclcpp::Node
{
public:
  ConfigManager()
  : Node("config_manager")
  {
    ops_dir_ = declare_parameter<std::string>("ops_dir", "/ops");
    city_ = declare_parameter<std::string>("city", "toulouse");

    pub_ = create_publisher<drone_interfaces::msg::ActiveConfig>(
      "/drone/config/active", rclcpp::QoS(1).reliable().transient_local());
    reload_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reload",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = load_and_publish();
        res->message = res->success ? "configuración válida" : "configuración NO válida (ver log)";
      });

    load_and_publish();
  }

private:
  bool load_and_publish()
  {
    const drone_core::OpsLoadResult r = drone_core::load_ops_config(ops_dir_, city_);

    drone_interfaces::msg::ActiveConfig msg;
    msg.stamp = now();
    msg.city = r.config.city.empty() ? city_ : r.config.city;
    msg.version = r.config.version;
    msg.ops_hash = r.config.ops_hash;
    msg.contingency_hash = r.config.contingency_hash;
    msg.valid = r.ok;
    msg.errors = r.errors;
    for (const auto & dz : r.config.drop_zones) {
      msg.drop_zone_ids.push_back(dz.id);
    }
    pub_->publish(msg);

    if (r.ok) {
      RCLCPP_INFO(get_logger(), "Configuración '%s' v%s válida: hash 0x%08X, contingencia 0x%08X, %zu zonas de suelta",
        msg.city.c_str(), msg.version.c_str(), msg.ops_hash, msg.contingency_hash, r.config.drop_zones.size());
      for (const auto & dz : r.config.drop_zones) {
        RCLCPP_INFO(get_logger(), "  zona '%s': DG_ZONE_HASH = %d", dz.id.c_str(),
          static_cast<int32_t>(drone_core::drop_zone_hash(dz)));
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Configuración '%s' NO válida (%zu errores):", city_.c_str(), r.errors.size());
      for (const auto & e : r.errors) {
        RCLCPP_ERROR(get_logger(), "  - %s", e.c_str());
      }
    }
    return r.ok;
  }

  std::string ops_dir_;
  std::string city_;
  rclcpp::Publisher<drone_interfaces::msg::ActiveConfig>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
};

}  // namespace drone_mission

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_mission::ConfigManager>());
  rclcpp::shutdown();
  return 0;
}
