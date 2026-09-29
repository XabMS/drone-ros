// drone_core — misiones de reparto (DOC-06 F-03) y su validación contra la configuración (SR-MSN-002).
#pragma once

#include <string>
#include <vector>

#include "drone_core/geo.hpp"
#include "drone_core/ops_config.hpp"

namespace drone_core
{

struct MissionPlan {
  std::string id;
  std::string city;
  std::string drop_zone_id;
  double cruise_alt_m{0.0};   ///< Sobre el hub
  double payload_kg{0.0};
  std::vector<LatLon> route;  ///< Waypoints intermedios de ida (sin hub ni zona de suelta)
};

struct MissionLoadResult {
  bool ok{false};
  std::vector<std::string> errors;
  MissionPlan plan;
};

/** Carga missions/<id>.yaml. No lanza excepciones. */
MissionLoadResult load_mission(const std::string & yaml_path);

/** Carga <missions_dir>/<id>.yaml. */
MissionLoadResult load_mission(const std::string & missions_dir, const std::string & id);

struct MissionValidation {
  bool ok{false};
  std::vector<std::string> errors;
  std::vector<std::string> warnings;  ///< p. ej. cruza una zona que exige autorización
  DropZone drop_zone;
  std::vector<LatLon> legs;           ///< hub, route..., centro de la zona de suelta
  double outbound_m{0.0};
  double total_m{0.0};                ///< ida + vuelta en línea recta al hub
};

/** Masa máxima de la carga (AR-001). */
constexpr double kMaxPayloadKg = 1.0;

/**
 * Valida la misión contra la configuración de su ciudad (SR-MSN-002):
 * ciudad correcta, zona de suelta existente, altura de crucero en rango, carga ≤ 1 kg,
 * cada tramo dentro del volumen operacional y fuera de zonas prohibidas (ida y vuelta),
 * longitud total ≤ max_route_m. Cruzar una zona con autorización da un aviso, no un error.
 */
MissionValidation validate_mission(const MissionPlan & plan, const OpsConfig & cfg);

}  // namespace drone_core
