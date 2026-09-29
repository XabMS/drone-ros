// Tests de drone_core. Se ejecutan con el directorio test/ como directorio de trabajo.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "drone_core/geo.hpp"
#include "drone_core/geometry.hpp"
#include "drone_core/mission.hpp"
#include "drone_core/ops_config.hpp"

using namespace drone_core;

namespace
{

bool contains(const std::vector<std::string> & v, const std::string & needle)
{
  for (const auto & s : v) {
    if (s.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

OpsConfig load_testville()
{
  const OpsLoadResult r = load_ops_config("ops", "testville");
  EXPECT_TRUE(r.ok);
  for (const auto & e : r.errors) {
    ADD_FAILURE() << e;
  }
  return r.config;
}

MissionPlan load_plan(const std::string & id)
{
  const MissionLoadResult r = load_mission("missions", id);
  EXPECT_TRUE(r.ok) << (r.errors.empty() ? "" : r.errors.front());
  return r.plan;
}

}  // namespace

// --- geo -------------------------------------------------------------------

TEST(Geo, HaversineOneDegreeOfLatitude)
{
  EXPECT_NEAR(haversine_m({43.0, 1.0}, {44.0, 1.0}), 111194.9, 0.5);
}

TEST(Geo, ProjectionRoundTrip)
{
  const LocalProjection proj({43.6045, 1.4440});
  double e, n;
  proj.to_local({43.6100, 1.4500}, e, n);
  const LatLon back = proj.to_global(e, n);
  EXPECT_NEAR(back.lat_deg, 43.6100, 1e-10);
  EXPECT_NEAR(back.lon_deg, 1.4500, 1e-10);
  EXPECT_NEAR(std::hypot(e, n), haversine_m({43.6045, 1.4440}, {43.6100, 1.4500}), 1.0);
}

TEST(Geo, ProjectionAcrossAntimeridian)
{
  const LocalProjection proj({0.0, 179.9999});
  double e, n;
  proj.to_local({0.0, -179.9999}, e, n);
  EXPECT_NEAR(e, 22.2, 0.1);
}

TEST(Geo, NedEnuConversion)
{
  const Vec3 enu = ned_to_enu({10.0, 20.0, -5.0});  // 10 m norte, 20 m este, 5 m arriba
  EXPECT_DOUBLE_EQ(enu.x, 20.0);
  EXPECT_DOUBLE_EQ(enu.y, 10.0);
  EXPECT_DOUBLE_EQ(enu.z, 5.0);
  const Vec3 ned = enu_to_ned(enu);
  EXPECT_DOUBLE_EQ(ned.x, 10.0);
  EXPECT_DOUBLE_EQ(ned.y, 20.0);
  EXPECT_DOUBLE_EQ(ned.z, -5.0);
}

TEST(Geo, Validity)
{
  EXPECT_TRUE(is_valid({43.6, 1.44}));
  EXPECT_FALSE(is_valid({91.0, 0.0}));
  EXPECT_FALSE(is_valid({0.0, -181.0}));
  EXPECT_FALSE(is_valid({std::numeric_limits<double>::quiet_NaN(), 0.0}));
}

// --- geometría -------------------------------------------------------------

TEST(Geometry, PointInSquareIncludingBorder)
{
  const Polygon2 sq{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(point_in_polygon({5, 5}, sq));
  EXPECT_TRUE(point_in_polygon({10, 5}, sq));   // borde
  EXPECT_TRUE(point_in_polygon({0, 0}, sq));    // vértice
  EXPECT_FALSE(point_in_polygon({10.01, 5}, sq));
  EXPECT_FALSE(point_in_polygon({5, 5}, Polygon2{{0, 0}, {1, 1}}));  // degenerado
}

TEST(Geometry, ClosedRingEqualsOpenRing)
{
  const Polygon2 open{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  const Polygon2 closed{{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
  EXPECT_EQ(point_in_polygon({5, 5}, open), point_in_polygon({5, 5}, closed));
  EXPECT_EQ(point_in_polygon({15, 5}, open), point_in_polygon({15, 5}, closed));
}

TEST(Geometry, SegmentsIntersectOrTouch)
{
  EXPECT_TRUE(segments_intersect({0, 0}, {10, 10}, {0, 10}, {10, 0}));
  EXPECT_TRUE(segments_intersect({0, 0}, {5, 5}, {5, 5}, {10, 0}));   // se tocan en un extremo
  EXPECT_FALSE(segments_intersect({0, 0}, {1, 1}, {2, 2}, {3, 0}));
  EXPECT_TRUE(segments_intersect({0, 0}, {4, 0}, {2, 0}, {6, 0}));    // colineales solapados
}

TEST(Geometry, SegmentInsideConcavePolygon)
{
  // Forma de U: el hueco central (x 3..7, y 3..10) está fuera.
  const Polygon2 u{{0, 0}, {10, 0}, {10, 10}, {7, 10}, {7, 3}, {3, 3}, {3, 10}, {0, 10}};
  EXPECT_TRUE(segment_inside_polygon({1, 1}, {9, 1}, u));
  EXPECT_FALSE(segment_inside_polygon({1, 8}, {9, 8}, u));  // cruza el hueco
  EXPECT_FALSE(segment_inside_polygon({1, 1}, {5, 5}, u));  // termina en el hueco
}

TEST(Geometry, SegmentInsideRejectsShortcutThroughReflexVertex)
{
  // Atajo que pasa justo por los vértices del hueco sin corte "propio".
  const Polygon2 u{{0, 0}, {10, 0}, {10, 10}, {7, 10}, {7, 3}, {3, 3}, {3, 10}, {0, 10}};
  EXPECT_FALSE(segment_inside_polygon({3, 10}, {7, 10}, u));
}

TEST(Geometry, SegmentIntersectsPolygon)
{
  const Polygon2 sq{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(segment_intersects_polygon({-5, 5}, {15, 5}, sq));  // lo atraviesa sin extremos dentro
  EXPECT_TRUE(segment_intersects_polygon({5, 5}, {20, 20}, sq));
  EXPECT_FALSE(segment_intersects_polygon({-5, -5}, {-1, 20}, sq));
}

TEST(Geometry, CircleInsidePolygon)
{
  const Polygon2 sq{{0, 0}, {100, 0}, {100, 100}, {0, 100}};
  EXPECT_TRUE(circle_inside_polygon({50, 50}, 10.0, sq));
  EXPECT_FALSE(circle_inside_polygon({95, 50}, 10.0, sq));
  EXPECT_FALSE(circle_inside_polygon({150, 50}, 1.0, sq));
}

TEST(Geometry, PolylineLength)
{
  EXPECT_NEAR(polyline_length_m({{43.0, 1.0}, {43.0, 1.0}}), 0.0, 1e-9);
  EXPECT_NEAR(polyline_length_m({{43.0, 1.0}, {44.0, 1.0}, {43.0, 1.0}}), 2 * 111194.9, 1.0);
}

// --- configuración por ciudad ----------------------------------------------

TEST(OpsConfig, Crc32KnownVector)
{
  EXPECT_EQ(crc32("123456789"), 0xCBF43926U);
  EXPECT_EQ(crc32(""), 0x00000000U);
}

TEST(OpsConfig, LoadsTestvilleAndIsValid)
{
  const OpsConfig c = load_testville();
  EXPECT_EQ(c.city, "testville");
  EXPECT_EQ(c.version, "test-1");
  EXPECT_EQ(c.drop_zones.size(), 1U);
  EXPECT_EQ(c.no_fly.size(), 1U);
  EXPECT_EQ(c.authorization_zones.size(), 1U);
  EXPECT_EQ(c.authorization_zones[0].authority, "ATC ficticio");
  EXPECT_EQ(c.corridors.size(), 1U);
  EXPECT_EQ(c.emergency_landing.size(), 1U);
  EXPECT_NE(c.ops_hash, 0U);
  EXPECT_NE(c.contingency_hash, 0U);
  ASSERT_NE(c.find_drop_zone("dz_01"), nullptr);
  EXPECT_EQ(c.find_drop_zone("no_existe"), nullptr);
}

TEST(OpsConfig, HashesAreStable)
{
  const OpsConfig a = load_testville();
  const OpsConfig b = load_testville();
  EXPECT_EQ(a.ops_hash, b.ops_hash);
  EXPECT_EQ(a.contingency_hash, b.contingency_hash);
}

TEST(OpsConfig, DropZoneHashChangesWithAnyField)
{
  const OpsConfig c = load_testville();
  DropZone dz = *c.find_drop_zone("dz_01");
  const uint32_t h = drop_zone_hash(dz);
  EXPECT_EQ(h, drop_zone_hash(dz));
  dz.radius_m += 1.0;
  EXPECT_NE(h, drop_zone_hash(dz));
}

TEST(OpsConfig, MissingFileMalformedAndUnknownRole)
{
  EXPECT_FALSE(load_ops_config("ops", "no_existe").ok);
  const OpsLoadResult m = load_ops_config("ops", "malformed");
  EXPECT_FALSE(m.ok);
  EXPECT_TRUE(contains(m.errors, "formato"));
  const OpsLoadResult b = load_ops_config("ops", "badrole");
  EXPECT_FALSE(b.ok);
  EXPECT_TRUE(contains(b.errors, "helipuerto"));
}

TEST(OpsConfig, ValidationCatchesInconsistencies)
{
  const OpsConfig base = load_testville();
  EXPECT_TRUE(validate_ops_config(base).empty());

  OpsConfig c = base;
  c.hub = {43.6200, 1.4440};  // fuera del volumen operacional
  EXPECT_TRUE(contains(validate_ops_config(c), "hub está fuera"));

  c = base;
  c.drop_zones[0].center = {43.6143, 1.4500};  // a ~20 m del borde norte con radio 50
  c.drop_zones[0].radius_m = 50.0;
  EXPECT_TRUE(contains(validate_ops_config(c), "no está entera dentro"));

  c = base;
  c.drop_zones[0].radius_m = 0.5;
  EXPECT_TRUE(contains(validate_ops_config(c), "radio"));

  c = base;
  c.drop_zones[0].alt_min_m = 40.0;  // min > max
  EXPECT_TRUE(contains(validate_ops_config(c), "banda"));

  c = base;
  c.drop_zones[0].center = {43.6050, 1.4490};  // dentro de la zona prohibida
  EXPECT_TRUE(contains(validate_ops_config(c), "prohibida"));

  c = base;
  c.contingency_volume = c.operational_volume;
  c.contingency_volume[1].lon_deg -= 0.01;  // un vértice del operacional queda fuera
  EXPECT_TRUE(contains(validate_ops_config(c), "contingencia"));

  c = base;
  c.emergency_landing[0].point = {43.7000, 1.4400};
  EXPECT_TRUE(contains(validate_ops_config(c), "emergencia"));

  c = base;
  c.max_height_agl_m = 150.0;
  EXPECT_TRUE(contains(validate_ops_config(c), "AR-005"));

  c = base;
  c.operational_volume.resize(2);
  EXPECT_TRUE(contains(validate_ops_config(c), "Volumen operacional"));

  c = base;
  c.drop_zones.clear();
  EXPECT_TRUE(contains(validate_ops_config(c), "No hay zonas"));
}

// --- misiones --------------------------------------------------------------

TEST(Mission, GoodMissionIsValid)
{
  const OpsConfig c = load_testville();
  const MissionValidation v = validate_mission(load_plan("good"), c);
  EXPECT_TRUE(v.ok);
  for (const auto & e : v.errors) {
    ADD_FAILURE() << e;
  }
  EXPECT_TRUE(v.warnings.empty());
  ASSERT_EQ(v.legs.size(), 3U);  // hub, waypoint, zona de suelta
  EXPECT_EQ(v.drop_zone.id, "dz_01");
  EXPECT_GT(v.total_m, v.outbound_m);
  EXPECT_NEAR(v.outbound_m, 389.2 + 520.0, 30.0);
}

TEST(Mission, RouteThroughNoFlyIsRejected)
{
  const MissionValidation v = validate_mission(load_plan("through_nofly"), load_testville());
  EXPECT_FALSE(v.ok);
  EXPECT_TRUE(contains(v.errors, "nf_hospital"));
}

TEST(Mission, RouteThroughAuthorizationZoneOnlyWarns)
{
  const MissionValidation v = validate_mission(load_plan("through_ctr"), load_testville());
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(contains(v.warnings, "ctr_test"));
}

TEST(Mission, ParameterChecks)
{
  const OpsConfig c = load_testville();
  const MissionPlan base = load_plan("good");

  MissionPlan p = base;
  p.city = "donostia";
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "donostia"));

  p = base;
  p.payload_kg = 1.5;
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "AR-001"));

  p = base;
  p.cruise_alt_m = 130.0;
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "AR-005"));

  p = base;
  p.drop_zone_id = "dz_99";
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "dz_99"));

  p = base;
  p.route.push_back({std::numeric_limits<double>::quiet_NaN(), 1.44});
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "no válidas"));

  p = base;
  p.route = {{43.6200, 1.4440}};  // fuera del volumen operacional
  EXPECT_TRUE(contains(validate_mission(p, c).errors, "sale del volumen"));
}

TEST(Mission, RouteLongerThanLimitIsRejected)
{
  OpsConfig c = load_testville();
  c.max_route_m = 1000.0;
  EXPECT_TRUE(contains(validate_mission(load_plan("good"), c).errors, "supera"));
}

TEST(Mission, ReturnLegIsCheckedToo)
{
  // Zona prohibida justo en la diagonal de vuelta zona de suelta -> hub.
  OpsConfig c = load_testville();
  c.no_fly[0].polygon = {{43.6070, 1.4465}, {43.6070, 1.4475}, {43.6078, 1.4475}, {43.6078, 1.4465}};
  const MissionValidation v = validate_mission(load_plan("good"), c);
  EXPECT_TRUE(contains(v.errors, "tramo de vuelta"));
}

TEST(Mission, MissingFile)
{
  const MissionLoadResult r = load_mission("missions", "no_existe");
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.errors.empty());
}

// --- caminos de error adicionales (cobertura de sentencias al 100 %) ---------

TEST(Geo, ProjectionWrapsNegativeLongitudeDifference)
{
  const LocalProjection proj({0.0, -179.9999});
  double e, n;
  proj.to_local({0.0, 179.9999}, e, n);
  EXPECT_NEAR(e, -22.2, 0.1);
}

TEST(Geometry, PolylineOfOnePointHasZeroLength)
{
  EXPECT_DOUBLE_EQ(polyline_length_m({{43.0, 1.0}}), 0.0);
  EXPECT_DOUBLE_EQ(polyline_length_m({}), 0.0);
}

TEST(OpsConfig, GeojsonReferenceProblems)
{
  const OpsLoadResult a = load_ops_config("ops", "nogeojson");
  EXPECT_FALSE(a.ok);
  EXPECT_TRUE(contains(a.errors, "geojson"));
  const OpsLoadResult b = load_ops_config("ops", "missinggj");
  EXPECT_FALSE(b.ok);
  EXPECT_TRUE(contains(b.errors, "no_existe.geojson"));
  const OpsLoadResult c = load_ops_config("ops", "nofeatures");
  EXPECT_FALSE(c.ok);
  EXPECT_TRUE(contains(c.errors, "features"));
}

TEST(OpsConfig, LoadByFullPathWithoutDirectory)
{
  // Ruta sin '/': el GeoJSON se busca en el directorio actual.
  EXPECT_FALSE(load_ops_config(std::string("testville.yaml")).ok);
}

TEST(OpsConfig, MoreValidationBranches)
{
  const OpsConfig base = load_testville();

  OpsConfig c = base;
  c.city.clear();
  c.hub = {std::numeric_limits<double>::quiet_NaN(), 1.44};
  c.max_route_m = 0.0;
  auto e = validate_ops_config(c);
  EXPECT_TRUE(contains(e, "Ciudad vacía"));
  EXPECT_TRUE(contains(e, "Hub no válido"));
  EXPECT_TRUE(contains(e, "max_route_m"));

  c = base;
  c.contingency_volume.clear();
  EXPECT_TRUE(contains(validate_ops_config(c), "contingencia ausente"));

  c = base;
  c.drop_zones[0].id.clear();
  EXPECT_TRUE(contains(validate_ops_config(c), "sin id"));

  c = base;
  c.drop_zones[0].center = {std::numeric_limits<double>::quiet_NaN(), 1.45};
  EXPECT_TRUE(contains(validate_ops_config(c), "centro no válido"));

  c = base;
  c.no_fly[0].polygon.resize(2);
  EXPECT_TRUE(contains(validate_ops_config(c), "no válida"));

  c = base;
  c.no_fly[0].polygon = {{43.6040, 1.4430}, {43.6040, 1.4450}, {43.6050, 1.4450}, {43.6050, 1.4430}};
  EXPECT_TRUE(contains(validate_ops_config(c), "hub está dentro"));
}

TEST(Mission, MissionWithoutRouteGoesStraightToDropZone)
{
  const MissionValidation v = validate_mission(load_plan("noroute"), load_testville());
  EXPECT_TRUE(v.ok);
  EXPECT_EQ(v.legs.size(), 2U);
}

TEST(OpsConfig, PolygonWithNonFiniteVertexIsInvalid)
{
  OpsConfig c = load_testville();
  c.operational_volume[2].lat_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(contains(validate_ops_config(c), "Volumen operacional"));
}
