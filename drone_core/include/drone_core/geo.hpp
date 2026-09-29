// drone_core — utilidades geodésicas (sin dependencias de ROS).
// Convenciones (DOC-06 §1): grados WGS84, metros, NED en PX4 y ENU en ROS 2.
// La conversión NED <-> ENU vive SOLO aquí.
#pragma once

namespace drone_core
{

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

struct LatLon {
  double lat_deg{0.0};
  double lon_deg{0.0};
};

/** true si latitud y longitud son finitas y están en rango. */
bool is_valid(const LatLon & p);

/** Distancia de gran círculo (haversine) en metros. */
double haversine_m(const LatLon & a, const LatLon & b);

/**
 * Proyección local equirectangular alrededor de un origen.
 * Error < 0,1 % hasta ~10 km del origen: suficiente para el radio de operación (≤ 5 km).
 * Ejes: x = este, y = norte (ENU plano).
 */
class LocalProjection
{
public:
  explicit LocalProjection(const LatLon & origin);

  void to_local(const LatLon & p, double & east_m, double & north_m) const;
  LatLon to_global(double east_m, double north_m) const;
  const LatLon & origin() const {return origin_;}

private:
  LatLon origin_;
  double cos_lat0_;
};

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/** NED (x norte, y este, z abajo) -> ENU (x este, y norte, z arriba). */
Vec3 ned_to_enu(const Vec3 & ned);

/** ENU -> NED. */
Vec3 enu_to_ned(const Vec3 & enu);

}  // namespace drone_core
