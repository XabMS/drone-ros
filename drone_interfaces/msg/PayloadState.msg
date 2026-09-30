# R-03 /drone/payload/state — estado de la carga (payload_manager, hito S3).
builtin_interfaces/Time stamp
bool payload_present
uint8 MECHANISM_UNKNOWN = 0
uint8 MECHANISM_CLOSED = 1
uint8 MECHANISM_OPEN = 2
uint8 mechanism
uint8 LAST_NONE = 0
uint8 LAST_RELEASED = 1
uint8 LAST_DENIED = 2
uint8 LAST_TIMEOUT = 3
uint8 LAST_NOT_RELEASED = 4
uint8 last_result

# Suelta en curso: el puente con tierra (drone_gcs_bridge) la anuncia al piloto y valida su confirmación con estos campos.
uint8 PHASE_IDLE = 0
uint8 PHASE_WAITING_CONFIRMATION = 1
uint8 PHASE_RELEASING = 2
uint8 PHASE_VERIFYING = 3
uint8 phase
string zone_id                 # Zona de la suelta en curso o de la última (vacío si aún no hubo ninguna)
int32 zone_hash                # DG_ZONE_HASH de esa zona: es lo que el piloto ve y confirma
