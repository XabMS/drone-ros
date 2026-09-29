// drone_validate — valida la configuración de una ciudad y, opcionalmente, una misión (SR-GND-003).
//   drone_validate <ops_dir> <ciudad> [<missions_dir> <mission_id>]
// Código de salida 0 si todo es válido.

#include <cstdio>
#include <string>

#include "drone_core/mission.hpp"
#include "drone_core/ops_config.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 5) {
    std::fprintf(stderr, "uso: %s <ops_dir> <ciudad> [<missions_dir> <mission_id>]\n", argv[0]);
    return 2;
  }
  const drone_core::OpsLoadResult c = drone_core::load_ops_config(argv[1], argv[2]);
  std::printf("Ciudad '%s' v%s: %s\n", argv[2], c.config.version.c_str(), c.ok ? "VÁLIDA" : "NO VÁLIDA");
  for (const auto & e : c.errors) {
    std::printf("  ERROR: %s\n", e.c_str());
  }
  if (!c.ok) {
    return 1;
  }
  std::printf("  hash 0x%08X, contingencia 0x%08X\n", c.config.ops_hash, c.config.contingency_hash);
  for (const auto & dz : c.config.drop_zones) {
    std::printf("  zona '%s': centro %.7f, %.7f, r %.1f m, h [%.1f, %.1f] m, DG_ZONE_HASH %d\n", dz.id.c_str(),
      dz.center.lat_deg, dz.center.lon_deg, dz.radius_m, dz.alt_min_m, dz.alt_max_m,
      static_cast<int>(drone_core::drop_zone_hash(dz)));
  }
  if (argc == 3) {
    return 0;
  }
  const drone_core::MissionLoadResult m = drone_core::load_mission(argv[3], argv[4]);
  if (!m.ok) {
    std::printf("Misión '%s': NO SE PUEDE CARGAR\n  %s\n", argv[4], m.errors.front().c_str());
    return 1;
  }
  const drone_core::MissionValidation v = drone_core::validate_mission(m.plan, c.config);
  std::printf("Misión '%s': %s (ida %.0f m, total %.0f m)\n", argv[4], v.ok ? "VÁLIDA" : "NO VÁLIDA",
    v.outbound_m, v.total_m);
  for (const auto & w : v.warnings) {
    std::printf("  AVISO: %s\n", w.c_str());
  }
  for (const auto & e : v.errors) {
    std::printf("  ERROR: %s\n", e.c_str());
  }
  return v.ok ? 0 : 1;
}
