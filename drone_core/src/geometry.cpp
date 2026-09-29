#include "drone_core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace drone_core
{

namespace
{

constexpr double kEps = 1e-9;

double cross(const Point2 & o, const Point2 & a, const Point2 & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

bool on_segment(const Point2 & p, const Point2 & a, const Point2 & b)
{
  return std::fabs(cross(a, b, p)) <= kEps &&
         p.x >= std::min(a.x, b.x) - kEps && p.x <= std::max(a.x, b.x) + kEps &&
         p.y >= std::min(a.y, b.y) - kEps && p.y <= std::max(a.y, b.y) + kEps;
}

int sign(double v)
{
  return (v > kEps) ? 1 : ((v < -kEps) ? -1 : 0);
}

/** Número de vértices sin el de cierre repetido. */
std::size_t vertex_count(const Polygon2 & poly)
{
  std::size_t n = poly.size();
  if (n > 1 && std::fabs(poly.front().x - poly.back().x) <= kEps &&
    std::fabs(poly.front().y - poly.back().y) <= kEps)
  {
    --n;
  }
  return n;
}

/** Corte "propio": los segmentos se cruzan en un punto interior de ambos. */
bool segments_cross_properly(const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  const int d1 = sign(cross(c, d, a));
  const int d2 = sign(cross(c, d, b));
  const int d3 = sign(cross(a, b, c));
  const int d4 = sign(cross(a, b, d));
  return (d1 * d2 < 0) && (d3 * d4 < 0);
}

}  // namespace

bool point_in_polygon(const Point2 & p, const Polygon2 & poly)
{
  const std::size_t n = vertex_count(poly);
  if (n < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2 & a = poly[i];
    const Point2 & b = poly[j];
    if (on_segment(p, a, b)) {
      return true;
    }
    if (((a.y > p.y) != (b.y > p.y)) &&
      (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x))
    {
      inside = !inside;
    }
  }
  return inside;
}

bool segments_intersect(const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  if (segments_cross_properly(a, b, c, d)) {
    return true;
  }
  return on_segment(a, c, d) || on_segment(b, c, d) || on_segment(c, a, b) || on_segment(d, a, b);
}

bool segment_inside_polygon(const Point2 & a, const Point2 & b, const Polygon2 & poly)
{
  if (!point_in_polygon(a, poly) || !point_in_polygon(b, poly)) {
    return false;
  }
  const std::size_t n = vertex_count(poly);
  for (std::size_t i = 0; i < n; ++i) {
    const Point2 & c = poly[i];
    const Point2 & d = poly[(i + 1) % n];
    if (segments_cross_properly(a, b, c, d)) {
      return false;
    }
  }
  // Polígonos cóncavos: el punto medio también debe estar dentro (evita atajos por fuera
  // que pasan justo por un vértice sin corte "propio").
  const Point2 mid{(a.x + b.x) / 2.0, (a.y + b.y) / 2.0};
  return point_in_polygon(mid, poly);
}

bool segment_intersects_polygon(const Point2 & a, const Point2 & b, const Polygon2 & poly)
{
  if (point_in_polygon(a, poly) || point_in_polygon(b, poly)) {
    return true;
  }
  const std::size_t n = vertex_count(poly);
  for (std::size_t i = 0; i < n; ++i) {
    if (segments_intersect(a, b, poly[i], poly[(i + 1) % n])) {
      return true;
    }
  }
  return false;
}

bool circle_inside_polygon(const Point2 & c, double radius, const Polygon2 & poly)
{
  if (!point_in_polygon(c, poly)) {
    return false;
  }
  constexpr int kSamples = 16;
  for (int k = 0; k < kSamples; ++k) {
    const double ang = 2.0 * kPi * static_cast<double>(k) / kSamples;
    const Point2 q{c.x + radius * std::cos(ang), c.y + radius * std::sin(ang)};
    if (!point_in_polygon(q, poly)) {
      return false;
    }
  }
  return true;
}

Polygon2 project(const std::vector<LatLon> & poly, const LocalProjection & proj)
{
  Polygon2 out;
  out.reserve(poly.size());
  for (const LatLon & p : poly) {
    Point2 q;
    proj.to_local(p, q.x, q.y);
    out.push_back(q);
  }
  return out;
}

double polyline_length_m(const std::vector<LatLon> & line)
{
  double total = 0.0;
  for (std::size_t i = 1; i < line.size(); ++i) {
    total += haversine_m(line[i - 1], line[i]);
  }
  return total;
}

}  // namespace drone_core
