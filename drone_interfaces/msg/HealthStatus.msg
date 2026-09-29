# R-04 /drone/health/status — salud del companion y del FTS (health_monitor, hito S4).
builtin_interfaces/Time stamp
NodeHealth[] nodes
uint8 FTS_UNKNOWN = 0
uint8 FTS_DISARMED = 1
uint8 FTS_ARMED = 2
uint8 FTS_TRIGGERED = 3
uint8 FTS_FAULT = 4
uint8 fts_state
bool fts_healthy
