from config import *

def has_error(status):
    return bool(status & (
        STATUS_ERROR_SENSOR |
        STATUS_ERROR_LOW_BAT |
        STATUS_ERROR_CRITICAL
    ))
