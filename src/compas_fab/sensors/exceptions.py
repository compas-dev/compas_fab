class ProtocolError(IOError):
    """Exception raised for protocol errors."""
    pass


class SensorTimeoutError(Exception):
    """Exception raised for sensor timeout errors."""
    pass
