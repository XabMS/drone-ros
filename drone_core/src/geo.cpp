#include "drone_core/geo.hpp"

#include <cmath>

namespace drone_core
{

bool is_valid(const LatLon & p)
{
  return std::isfinite(p.lat_deg) && std::isfinite(p.lon_deg) &&
         (p.lat_deg >= -90.0) && (p.lat_deg <= 90.0) &&
         (p.lon_deg >= -180.0) && (p.lon_deg <= 180.0);
}

double haversine_m(const LatLon & a, const LatLon & b)
{
  const double p0 = a.lat_deg * kDegToRad;
  const double p1 = b.lat_deg * kDegToRad;
  const double dp = (b.lat_deg - a.lat_deg) * kDegToRad;
  const double dl = (b.lon_deg - a.lon_deg) * kDegToRad;
  const double s = std::sin(dp / 2.0) * std::sin(dp / 2.0) +
    std::cos(p0) * std::cos(p1) * std::sin(dl / 2.0) * std::sin(dl / 2.0);
  return 2.0 * kEarthRadiusM * std::asin(std::sqrt(s));
}

LocalProjection::LocalProjection(const LatLon & origin)
: origin_(origin), cos_lat0_(std::cos(origin.lat_deg * kDegToRad))
{
}

void LocalProjection::to_local(const LatLon & p, double & east_m, double & north_m) const
{
  double dlon = p.lon_deg - origin_.lon_deg;
  if (dlon > 180.0) {
    dlon -= 360.0;
  } else if (dlon < -180.0) {
    dlon += 360.0;
  }
  east_m = dlon * kDegToRad * cos_lat0_ * kEarthRadiusM;
  north_m = (p.lat_deg - origin_.lat_deg) * kDegToRad * kEarthRadiusM;
}

LatLon LocalProjection::to_global(double east_m, double north_m) const
{
  LatLon p;
  p.lat_deg = origin_.lat_deg + (north_m / kEarthRadiusM) / kDegToRad;
  p.lon_deg = origin_.lon_deg + (east_m / (kEarthRadiusM * cos_lat0_)) / kDegToRad;
  return p;
}

Vec3 ned_to_enu(const Vec3 & ned)
{
  return Vec3{ned.y, ned.x, -ned.z};
}

Vec3 enu_to_ned(const Vec3 & enu)
{
  return Vec3{enu.y, enu.x, -enu.z};
}

}  // namespace drone_core
