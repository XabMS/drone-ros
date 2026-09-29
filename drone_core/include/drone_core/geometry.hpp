// drone_core — geometría plana en coordenadas locales (x este, y norte, metros).
#pragma once

#include <vector>

#include "drone_core/geo.hpp"

namespace drone_core
{

struct Point2 {
  double x{0.0};
  double y{0.0};
};

using Polygon2 = std::vector<Point2>;

/** Punto dentro del polígono (el borde cuenta como dentro). Polígonos simples, abiertos o cerrados. */
bool point_in_polygon(const Point2 & p, const Polygon2 & poly);

/** true si los segmentos ab y cd se cortan o se tocan. */
bool segments_intersect(const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d);

/** true si el segmento ab queda entero dentro del polígono (extremos dentro y sin cruzar el borde). */
bool segment_inside_polygon(const Point2 & a, const Point2 & b, const Polygon2 & poly);

/** true si el segmento ab toca el polígono (algún extremo dentro o corta el borde). */
bool segment_intersects_polygon(const Point2 & a, const Point2 & b, const Polygon2 & poly);

/** true si el círculo (centro, radio) queda dentro del polígono (se comprueban 16 puntos del borde). */
bool circle_inside_polygon(const Point2 & c, double radius, const Polygon2 & poly);

/** Proyecta un polígono geográfico a coordenadas locales. */
Polygon2 project(const std::vector<LatLon> & poly, const LocalProjection & proj);

/** Longitud de una polilínea geográfica (suma de haversines). */
double polyline_length_m(const std::vector<LatLon> & line);

}  // namespace drone_core
