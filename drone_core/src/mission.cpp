#include "drone_core/mission.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <exception>
#include <set>

#include "drone_core/geometry.hpp"

namespace drone_core
{

MissionLoadResult load_mission(const std::string & missions_dir, const std::string & id)
{
  return load_mission(missions_dir + "/" + id + ".yaml");
}

MissionLoadResult load_mission(const std::string & yaml_path)
{
  MissionLoadResult r;
  try {
    const YAML::Node y = YAML::LoadFile(yaml_path);
    MissionPlan & m = r.plan;
    m.id = y["id"].as<std::string>();
    m.city = y["city"].as<std::string>();
    m.drop_zone_id = y["drop_zone"].as<std::string>();
    m.cruise_alt_m = y["cruise_alt_m"].as<double>();
    m.payload_kg = y["payload_kg"].as<double>();
    if (y["route"]) {
      for (const auto & wp : y["route"]) {
        m.route.push_back(LatLon{wp["lat"].as<double>(), wp["lon"].as<double>()});
      }
    }
    r.ok = true;
  } catch (const std::exception & e) {
    r.errors.push_back("No se puede cargar la misión " + yaml_path + ": " + e.what());
  }
  return r;
}

MissionValidation validate_mission(const MissionPlan & plan, const OpsConfig & cfg)
{
  MissionValidation v;
  auto & e = v.errors;

  if (plan.city != cfg.city) {
    e.push_back("La misión es para '" + plan.city + "' y la configuración activa es '" + cfg.city + "'");
  }
  if (!(plan.payload_kg >= 0.0 && plan.payload_kg <= kMaxPayloadKg)) {
    e.push_back("Carga fuera de [0, 1] kg (AR-001)");
  }
  if (!(plan.cruise_alt_m >= 10.0 && plan.cruise_alt_m <= cfg.max_height_agl_m)) {
    e.push_back("Altura de crucero fuera de [10, max_height_agl_m] m (AR-005)");
  }
  const DropZone * dz = cfg.find_drop_zone(plan.drop_zone_id);
  if (dz == nullptr) {
    e.push_back("Zona de suelta '" + plan.drop_zone_id + "' no existe en la configuración");
    return v;
  }
  v.drop_zone = *dz;

  v.legs.push_back(cfg.hub);
  for (const LatLon & p : plan.route) {
    if (!is_valid(p)) {
      e.push_back("Waypoint con coordenadas no válidas");
      return v;
    }
    v.legs.push_back(p);
  }
  v.legs.push_back(dz->center);

  const LocalProjection proj(cfg.hub);
  const Polygon2 op = project(cfg.operational_volume, proj);
  std::vector<Polygon2> nofly;
  for (const PolygonZone & z : cfg.no_fly) {
    nofly.push_back(project(z.polygon, proj));
  }
  std::vector<Polygon2> auth;
  for (const PolygonZone & z : cfg.authorization_zones) {
    auth.push_back(project(z.polygon, proj));
  }

  // Tramos de ida + tramo de vuelta en línea recta (RTL de PX4) desde la zona de suelta al hub.
  std::vector<std::pair<LatLon, LatLon>> segments;
  for (std::size_t i = 1; i < v.legs.size(); ++i) {
    segments.emplace_back(v.legs[i - 1], v.legs[i]);
  }
  segments.emplace_back(dz->center, cfg.hub);

  std::set<std::string> auth_crossed;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const bool is_return = (i + 1 == segments.size());
    char name[48];
    if (is_return) {
      std::snprintf(name, sizeof(name), "tramo de vuelta");
    } else {
      std::snprintf(name, sizeof(name), "tramo %zu", i + 1);
    }
    Point2 a;
    Point2 b;
    proj.to_local(segments[i].first, a.x, a.y);
    proj.to_local(segments[i].second, b.x, b.y);
    if (!segment_inside_polygon(a, b, op)) {
      e.push_back(std::string(name) + " sale del volumen operacional");
    }
    for (std::size_t k = 0; k < nofly.size(); ++k) {
      if (segment_intersects_polygon(a, b, nofly[k])) {
        e.push_back(std::string(name) + " cruza la zona prohibida '" + cfg.no_fly[k].id + "'");
      }
    }
    for (std::size_t k = 0; k < auth.size(); ++k) {
      if (segment_intersects_polygon(a, b, auth[k])) {
        auth_crossed.insert(cfg.authorization_zones[k].id);
      }
    }
  }
  for (const std::string & id : auth_crossed) {
    v.warnings.push_back("La ruta cruza la zona '" + id + "', que exige autorización");
  }

  v.outbound_m = polyline_length_m(v.legs);
  v.total_m = v.outbound_m + haversine_m(dz->center, cfg.hub);
  if (v.total_m > cfg.max_route_m) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Ruta de %.0f m supera el máximo de %.0f m", v.total_m, cfg.max_route_m);
    e.push_back(buf);
  }
  v.ok = e.empty();
  return v;
}

}  // namespace drone_core
