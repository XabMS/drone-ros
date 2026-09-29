// drone_core — configuración de la operación por ciudad (DOC-06 F-01 y F-02, AR-019).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "drone_core/geo.hpp"

namespace drone_core
{

struct PolygonZone {
  std::string id;
  std::vector<LatLon> polygon;
  std::string authority;  ///< Solo para role = authorization_zone
};

struct Corridor {
  std::string id;
  std::vector<LatLon> line;
  double width_m{0.0};
};

/** Zona de suelta circular: coincide con drop_guard v1 (DG_LAT_E7, DG_LON_E7, DG_RADIUS, DG_ALT_*). */
struct DropZone {
  std::string id;
  LatLon center;
  double radius_m{0.0};
  double alt_min_m{0.0};  ///< Sobre el hub
  double alt_max_m{0.0};
};

struct NamedPoint {
  std::string id;
  LatLon point;
};

struct OpsConfig {
  std::string city;
  std::string version;
  std::string hub_name;
  LatLon hub;
  double hub_alt_m{0.0};           ///< AMSL
  double max_height_agl_m{120.0};
  double max_route_m{0.0};         ///< Longitud máxima ida + vuelta
  std::vector<LatLon> operational_volume;
  std::vector<LatLon> contingency_volume;
  std::vector<Corridor> corridors;
  std::vector<DropZone> drop_zones;
  std::vector<PolygonZone> no_fly;
  std::vector<PolygonZone> authorization_zones;
  std::vector<NamedPoint> emergency_landing;

  uint32_t ops_hash{0};          ///< CRC32 del YAML + GeoJSON tal como están en disco
  uint32_t contingency_hash{0};  ///< CRC32 del volumen de contingencia (lo usa también el FTS, SR-FTS-006)

  const DropZone * find_drop_zone(const std::string & id) const;
};

struct OpsLoadResult {
  bool ok{false};
  std::vector<std::string> errors;
  OpsConfig config;
};

/** Carga ops/<ciudad>.yaml y el GeoJSON que referencia (ruta relativa al YAML). No lanza excepciones. */
OpsLoadResult load_ops_config(const std::string & yaml_path);

/** Carga el directorio de operación y la ciudad: <ops_dir>/<city>.yaml. */
OpsLoadResult load_ops_config(const std::string & ops_dir, const std::string & city);

/** Comprobaciones de coherencia (SR-MSN-002, SR-GND-003). Vacío = válida. */
std::vector<std::string> validate_ops_config(const OpsConfig & cfg);

/** CRC-32 (IEEE 802.3, el de zip). */
uint32_t crc32(const std::string & data);

/** Hash de una zona de suelta para DG_ZONE_HASH (drop_guard). */
uint32_t drop_zone_hash(const DropZone & dz);

}  // namespace drone_core
