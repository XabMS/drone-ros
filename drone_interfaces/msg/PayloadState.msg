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
