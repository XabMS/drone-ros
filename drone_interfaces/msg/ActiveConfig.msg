# R-05 /drone/config/active — configuración de operación en uso (config_manager). QoS RELIABLE, TRANSIENT_LOCAL.
builtin_interfaces/Time stamp
string city
string version
uint32 ops_hash            # CRC32 de <ciudad>.yaml + <ciudad>.geojson
uint32 contingency_hash    # CRC32 del volumen de contingencia (SR-FTS-006)
bool valid
string[] errors
string[] drop_zone_ids
