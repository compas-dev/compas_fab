class SerialSensor(object):
    """Base class for all sensors with a serial
    interface.

    Args:
        serial (:obj:`serial.Serial`): Instance of the serial
            port used to communicate with the sensor.

    Examples:
        >>> from serial import Serial                           # doctest: +SKIP
        >>> with Serial('COM5', 57600, timeout=1) as serial:    # doctest: +SKIP
        ...     sensor = SerialSensor(serial)                   # doctest: +SKIP
    """
    def __init__(self, serial):
        self.serial = serial

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass
