int32 value 0

int32 UNSET = 0
int32 OK = 1
int32 NOK = 2  # convenience flag, has to be set in all failure cases (in addition to some other NOK flag)
int32 NOK_ALREADY_IN_REQUESTED_MODE = 4  # binary flags for various reasons why MO change failed
int32 NOK_ROBOT_UNAVAILABLE = 8  # e.g., robot controller failed or otherwise cannot connect to robot
int32 NOK_CURRENTLY_EXECUTING_ACTION = 16
int32 NOK_INVALID_MODE = 32  # requested mode does not exist
int32 NOK_FORBIDDEN_TRANSITION = 64  # transition from current to the requested mode is forbidden
int32 NOK_RESOURCE_KEY_ERROR = 128
