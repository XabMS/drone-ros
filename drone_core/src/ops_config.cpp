#include "drone_core/ops_config.hpp"

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "drone_core/geometry.hpp"

namespace drone_core
{

namespace
{

bool read_file(const std::string & path, std::string & out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

std::string dir_of(const std::string & path)
{
  const auto pos = path.find_last_of('/');
  return (pos == std::string::npos) ? std::string(".") : path.substr(0, pos);
}

std::string fmt7(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.7f", v);
  return buf;
}

std::vector<LatLon> parse_ring(const nlohmann::json & ring)
{
  std::vector<LatLon> out;
  for (const auto & c : ring) {
    // GeoJSON: [longitud, latitud]
    out.push_back(LatLon{c.at(1).get<double>(), c.at(0).get<double>()});
  }
  return out;
}

std::string prop_str(const nlohmann::json & props, const char * key)
{
  return (props.contains(key) && props[key].is_string()) ? props[key].get<std::string>() : std::string();
}

double prop_num(const nlohmann::json & props, const char * key, double def)
{
  return (props.contains(key) && props[key].is_number()) ? props[key].get<double>() : def;
}

void parse_geojson(const std::string & text, OpsConfig & cfg, std::vector<std::string> & errors)
{
  const nlohmann::json doc = nlohmann::json::parse(text);
  if (!doc.contains("features") || !doc["features"].is_array()) {
    errors.push_back("GeoJSON sin 'features'");
    return;
  }
  for (const auto & f : doc["features"]) {
    const auto & props = f.at("properties");
    const auto & geom = f.at("geometry");
    const std::string role = prop_str(props, "role");
    const std::string id = prop_str(props, "id");
    const std::string type = geom.at("type").get<std::string>();
    const auto & coords = geom.at("coordinates");

    if (role == "operational_volume" && type == "Polygon") {
      cfg.operational_volume = parse_ring(coords.at(0));
      cfg.max_height_agl_m = std::fmin(cfg.max_height_agl_m, prop_num(props, "max_alt_m", 120.0));
    } else if (role == "contingency_volume" && type == "Polygon") {
      cfg.contingency_volume = parse_ring(coords.at(0));
    } else if (role == "corridor" && type == "LineString") {
      cfg.corridors.push_back(Corridor{id, parse_ring(coords), prop_num(props, "width_m", 0.0)});
    } else if (role == "drop_zone" && type == "Point") {
      DropZone dz;
      dz.id = id;
      dz.center = LatLon{coords.at(1).get<double>(), coords.at(0).get<double>()};
      dz.radius_m = prop_num(props, "radius_m", NAN);
      dz.alt_min_m = prop_num(props, "drop_alt_min_m", NAN);
      dz.alt_max_m = prop_num(props, "drop_alt_max_m", NAN);
      cfg.drop_zones.push_back(dz);
    } else if (role == "no_fly" && type == "Polygon") {
      cfg.no_fly.push_back(PolygonZone{id, parse_ring(coords.at(0)), ""});
    } else if (role == "authorization_zone" && type == "Polygon") {
      cfg.authorization_zones.push_back(PolygonZone{id, parse_ring(coords.at(0)), prop_str(props, "authority")});
    } else if (role == "emergency_landing" && type == "Point") {
      cfg.emergency_landing.push_back(
        NamedPoint{id, LatLon{coords.at(1).get<double>(), coords.at(0).get<double>()}});
    } else {
      errors.push_back("Feature no reconocida: role='" + role + "', tipo " + type);
    }
  }
}

bool ring_valid(const std::vector<LatLon> & ring)
{
  if (ring.size() < 3) {
    return false;
  }
  for (const LatLon & p : ring) {
    if (!is_valid(p)) {
      return false;
    }
  }
  return true;
}

}  // namespace

const DropZone * OpsConfig::find_drop_zone(const std::string & id) const
{
  for (const DropZone & dz : drop_zones) {
    if (dz.id == id) {
      return &dz;
    }
  }
  return nullptr;
}

uint32_t crc32(const std::string & data)
{
  static const std::array<uint32_t, 256> table = [] {
      std::array<uint32_t, 256> t{};
      for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
          c = (c & 1U) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
        }
        t[i] = c;
      }
      return t;
    }();
  uint32_t crc = 0xFFFFFFFFU;
  for (unsigned char ch : data) {
    crc = table[(crc ^ ch) & 0xFFU] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFU;
}

uint32_t drop_zone_hash(const DropZone & dz)
{
  const std::string canon = dz.id + "|" + fmt7(dz.center.lat_deg) + "|" + fmt7(dz.center.lon_deg) + "|" +
    fmt7(dz.radius_m) + "|" + fmt7(dz.alt_min_m) + "|" + fmt7(dz.alt_max_m);
  return crc32(canon);
}

OpsLoadResult load_ops_config(const std::string & ops_dir, const std::string & city)
{
  return load_ops_config(ops_dir + "/" + city + ".yaml");
}

OpsLoadResult load_ops_config(const std::string & yaml_path)
{
  OpsLoadResult r;
  std::string yaml_text;
  if (!read_file(yaml_path, yaml_text)) {
    r.errors.push_back("No se puede leer " + yaml_path);
    return r;
  }

  std::string geojson_text;
  try {
    const YAML::Node y = YAML::Load(yaml_text);
    OpsConfig & c = r.config;
    c.city = y["city"].as<std::string>();
    c.version = y["version"] ? y["version"].as<std::string>() : std::string("sin versión");
    c.hub_name = y["hub"]["name"] ? y["hub"]["name"].as<std::string>() : std::string();
    c.hub = LatLon{y["hub"]["lat"].as<double>(), y["hub"]["lon"].as<double>()};
    c.hub_alt_m = y["hub"]["alt_m"].as<double>();
    c.max_height_agl_m = y["limits"]["max_height_agl_m"].as<double>();
    c.max_route_m = y["limits"]["max_route_m"] ? y["limits"]["max_route_m"].as<double>() : 0.0;

    if (!y["geojson"]) {
      r.errors.push_back("Falta la clave 'geojson' en " + yaml_path);
      return r;
    }
    const std::string gj_path = dir_of(yaml_path) + "/" + y["geojson"].as<std::string>();
    if (!read_file(gj_path, geojson_text)) {
      r.errors.push_back("No se puede leer " + gj_path);
      return r;
    }
    parse_geojson(geojson_text, c, r.errors);
  } catch (const std::exception & e) {
    r.errors.push_back(std::string("Error de formato: ") + e.what());
    return r;
  }

  OpsConfig & c = r.config;
  c.ops_hash = crc32(yaml_text + geojson_text);
  std::string canon;
  for (const LatLon & p : c.contingency_volume) {
    canon += fmt7(p.lat_deg) + "," + fmt7(p.lon_deg) + ";";
  }
  c.contingency_hash = crc32(canon);

  for (const std::string & e : validate_ops_config(c)) {
    r.errors.push_back(e);
  }
  r.ok = r.errors.empty();
  return r;
}

std::vector<std::string> validate_ops_config(const OpsConfig & c)
{
  std::vector<std::string> e;
  if (c.city.empty()) {
    e.push_back("Ciudad vacía");
  }
  if (!is_valid(c.hub) || !std::isfinite(c.hub_alt_m)) {
    e.push_back("Hub no válido");
  }
  if (!(c.max_height_agl_m > 0.0 && c.max_height_agl_m <= 120.0)) {
    e.push_back("max_height_agl_m debe estar en (0, 120] m (AR-005)");
  }
  if (!(c.max_route_m > 0.0)) {
    e.push_back("max_route_m debe ser positivo");
  }
  if (!ring_valid(c.operational_volume)) {
    e.push_back("Volumen operacional ausente o no válido");
    return e;  // el resto de comprobaciones dependen de él
  }
  if (!ring_valid(c.contingency_volume)) {
    e.push_back("Volumen de contingencia ausente o no válido");
    return e;
  }

  const LocalProjection proj(c.hub);
  const Polygon2 op = project(c.operational_volume, proj);
  const Polygon2 cont = project(c.contingency_volume, proj);

  Point2 hub_local;
  proj.to_local(c.hub, hub_local.x, hub_local.y);
  if (!point_in_polygon(hub_local, op)) {
    e.push_back("El hub está fuera del volumen operacional");
  }
  for (const Point2 & v : op) {
    if (!point_in_polygon(v, cont)) {
      e.push_back("El volumen operacional sale del volumen de contingencia");
      break;
    }
  }
  if (c.drop_zones.empty()) {
    e.push_back("No hay zonas de suelta");
  }
  for (const DropZone & dz : c.drop_zones) {
    const std::string who = "Zona de suelta '" + dz.id + "': ";
    if (dz.id.empty()) {
      e.push_back("Zona de suelta sin id");
    }
    if (!is_valid(dz.center)) {
      e.push_back(who + "centro no válido");
      continue;
    }
    if (!(dz.radius_m >= 1.0 && dz.radius_m <= 100.0)) {
      e.push_back(who + "radio fuera de [1, 100] m (DG_RADIUS)");
    }
    if (!(dz.alt_min_m >= 5.0 && dz.alt_max_m <= 120.0 && dz.alt_min_m < dz.alt_max_m)) {
      e.push_back(who + "banda de altura no válida (5 ≤ min < max ≤ 120)");
    }
    Point2 ctr;
    proj.to_local(dz.center, ctr.x, ctr.y);
    if (!circle_inside_polygon(ctr, dz.radius_m, op)) {
      e.push_back(who + "no está entera dentro del volumen operacional");
    }
    for (const PolygonZone & nf : c.no_fly) {
      if (point_in_polygon(ctr, project(nf.polygon, proj))) {
        e.push_back(who + "está dentro de la zona prohibida '" + nf.id + "'");
      }
    }
  }
  for (const PolygonZone & nf : c.no_fly) {
    if (!ring_valid(nf.polygon)) {
      e.push_back("Zona prohibida '" + nf.id + "' no válida");
    } else if (point_in_polygon(hub_local, project(nf.polygon, proj))) {
      e.push_back("El hub está dentro de la zona prohibida '" + nf.id + "'");
    }
  }
  for (const NamedPoint & p : c.emergency_landing) {
    Point2 q;
    proj.to_local(p.point, q.x, q.y);
    if (!is_valid(p.point) || !point_in_polygon(q, op)) {
      e.push_back("Punto de aterrizaje de emergencia '" + p.id + "' fuera del volumen operacional");
    }
  }
  return e;
}

}  // namespace drone_core
