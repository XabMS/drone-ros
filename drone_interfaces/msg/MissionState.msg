# R-01 /drone/mission/state — estado de la misión (mission_manager). QoS RELIABLE, TRANSIENT_LOCAL.
builtin_interfaces/Time stamp

uint8 STATE_PREFLIGHT = 0
uint8 STATE_ARMING = 1
uint8 STATE_TAKEOFF = 2
uint8 STATE_CRUISE = 3
uint8 STATE_APPROACH = 4
uint8 STATE_DROP = 5
uint8 STATE_RETURN = 6
uint8 STATE_CONTINGENCY = 7
uint8 STATE_COMPLETED = 8

uint8 state
uint8 previous_state
string state_name
string cause                   # Motivo de la última transición

string mission_id
uint16 waypoint_index          # Tramo actual (0 = primer waypoint)
uint16 waypoint_count
float32 distance_to_target_m   # NaN si no aplica

bool ready_to_start
string[] preflight_blockers    # Motivos por los que aún no se puede iniciar

uint8 RESULT_NONE = 0
uint8 RESULT_COMPLETED = 1
uint8 RESULT_ABORTED = 2
uint8 RESULT_CONTINGENCY = 3
uint8 result

uint8 DROP_RESULT_NONE = 0
uint8 DROP_RESULT_RELEASED = 1
uint8 DROP_RESULT_DENIED = 2
uint8 DROP_RESULT_TIMEOUT = 3
uint8 DROP_RESULT_NOT_RELEASED = 4
uint8 DROP_RESULT_UNAVAILABLE = 5   # payload_manager rechazó el objetivo, no estaba o no respondió a tiempo
uint8 drop_result              # Resultado de la suelta de esta misión (NONE hasta que hay respuesta)
