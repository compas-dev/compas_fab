import time

from compas_fab.sensors import SerialSensor
from compas_fab.sensors.exceptions import ProtocolError
from compas_fab.sensors.exceptions import SensorTimeoutError

__all__ = ['PosCon3D', 'PosConCM']


class PosCon3D(SerialSensor):
    """Provides an interface for the `Baumer PosCon3D edge measurement sensor
    <http://www.baumer.com/us-en/products/distance-measurement/light-section-sensors/poscon-3d-edge-measurement/>`_.

    The sensor has different interfaces to retrieve its data. This
    class provides access to the serial interface (RS-485). This
    class is a context manager type, so it's best used in combination
    with the ``with`` statement to ensure resource deallocation. The
    protocol of the sensor when operated via RS-485 indicates that
    access to it must be locked programmatically before starting operations
    and unlocked on completion. This is handled automatically if you use
    this class on a ``with`` statement, otherwise, the methods ``begin()`` and
    ``end()`` must be invoked by hand.

    Attributes
    ----------
    serial : :obj:`serial.Serial`
        Instance of the serial port used to communicate with the sensor.
    address : :obj:`int`
        PosCon3D sensors have an address assigned, which defaults to 1.
        There's also a broadcast address (``PosCon3D.BROADCAST_ADDRESS``)
        that can be used to query the address of the sensor connected to
        the RS-485 bus. Only one sensor can be in the bus when using the
        broadcast address to query for sensor's address.

    Examples
    --------
    >>> from serial import Serial                                                   # doctest: +SKIP
    >>> with Serial('COM5', 57600, timeout=1) as serial:                            # doctest: +SKIP
    >>>     with PosCon3D(serial, PosCon3D.BROADCAST_ADDRESS) as broadcast_query:   # doctest: +SKIP
    ...         addr = broadcast_query.get_address()                                # doctest: +SKIP
    ...                                                                             # doctest: +SKIP
    ...     with PosCon3D(serial, addr) as sensor:                                  # doctest: +SKIP
    ...         sensor.set_measurement_type('Edge L rise')                          # doctest: +SKIP
    ...         sensor.set_precision(2)                                             # doctest: +SKIP
    ...         data = sensor.get_measurement()                                     # doctest: +SKIP
    """
    FRAME_HEAD = '{%s,%s,%s'
    FRAME_TAIL = '%s%s}'
    BROADCAST_ADDRESS = 0
    MEASUREMENT_TYPES = ('Edge L rise', 'Edge L fall', 'Edge R rise',
                         'Edge R fall', 'Width', 'Center width', 'Gap', 'Center gap')
    QUALITY = {
        0: 'Valid',
        1: 'Low signal',
        2: 'No edge',
        3: 'Low signal, no edge',
        4: 'No signal'}
    ERROR_CODES = {
        '000': 'No error',
        '001': 'False checksum',
        '002': 'False command',
        '003': 'False frame',
        '004': 'False value or parameter',
        '005': 'Missed command 000 to begin RS-485 control',
        '006': 'Out of range',
        '007': 'Buffer overflow',
        '010': 'All outputs Off',
        '020': 'Display Off',
        '99': 'Argument out of Range',
        '100': 'Distance out of Range (see FSP)',
        '101': 'Angle out of Range (see FSP)',
        '102': 'Flatness out of Range (see FSP)',
        '103': 'Length out of Range (see FSP)',
        '200': 'Fatal Error (Reset sensor, Power Off / On)'}

    def __init__(self, serial, address):
        super(PosCon3D, self).__init__(serial)
        self.address = address

    def __enter__(self):
        self.begin()
        return self

    def __exit__(self, *args):
        self.end()

    def begin(self):
        """Locks the sensor to start RS-485 communication.

        Note
        ----
        This method only needs to be called if not using
        a ``with`` statement to handle lifetime of the `PosCon3D` instance.
        """
        return self.send_command(self.address, '000', '1')

    def end(self):
        """Unlocks the sensor from RS-485 communication.

        Note
        ----
        This method only needs to be called if not using
        a ``with`` statement to handle lifetime of the `PosCon3D` instance.
        """
        return self.send_command(self.address, '000', '0')

    def send_command(self, address, command, data=None):
        """Sends a command to the sensor's address specified. The command
        can optionally contain a data string.

        This method is mostly for internal use, as the higher-level API is
        exposed via dedicated methods.

        Parameters
        ----------
        address : :obj:`int`
            PosCon3D sensors have an address assigned, which defaults to 1.
            There's also a broadcast address (``PosCon3D.BROADCAST_ADDRESS``)
            that can be used to query the address of the sensor connected to
            the RS-485 bus. Only one sensor can be in the bus when using the
            broadcast address to query for sensor's address.
        command : :obj:`string`
            A string indicating the command number to be executed.
        data : :obj:`string`
            An optional string of data that is sent together with
            the command.

        Returns
        -------
        `list` or value
            Result of the command. It can be a list or a single value depending on the operation.
        """
        cmd = self.format_command(address, command, data)

        # Python 2 vs 3
        if hasattr(cmd, 'decode'):
            self.serial.write(cmd)
            result = self.serial.readline()
        else:
            self.serial.write(cmd.encode('ascii'))
            result = self.serial.readline().decode('ascii')

        if result:
            frame_head = result[:-4]
            checksum = result[-4:-1]
            expected = self.calculate_checksum(frame_head)
            if expected != checksum:
                raise ProtocolError('Invalid response, checksum mismatch. Expected=%s, Got=%s' % (expected, checksum))

            expected_frame_head = self.FRAME_HEAD % (address, command, '')
            if not result.startswith(expected_frame_head):
                raise ProtocolError('Invalid response, command/address mismatch. Expected to start with="%s", Got="%s"' % (expected_frame_head, result))

            return self.get_payload(result)

        return None

    def format_command(self, address, command, data=None):
        """Formats the command."""
        data = data + ',' if data else ''
        frame = self.FRAME_HEAD % (address, command, data)
        return self.FRAME_TAIL % (frame, self.calculate_checksum(frame))

    def calculate_checksum(self, command):
        """Checks that message is complete."""
        code_points = [ord(c) for c in list(command)]
        checksum = 0
        for i in code_points:
            checksum = checksum ^ i

        return str(checksum).zfill(3)

    def get_payload(self, result):
        """Gets payload."""
        data = result.split(',')[2:-1]
        if not data:
            return None
        elif len(data) == 1:
            return data[0]
        else:
            if data[0] == 'E':
                raise ProtocolError(self.ERROR_CODES[str(data[1])])

            return data

    def get_address(self):
        """Gets the address of the RS-485 sensors currently connected to the bus. This command
        is only really useful when this class is initialized with the broadcast address,
        with the purpose of retrieving the address of a sensor connected.

        Returns
        -------
        `int`
            Address of the PosCon3D sensor connected to the RS-485 bus.

        Note
        ----
        Only one PosCon3D sensor can be connected to the bus for this operation to succeed.
        """
        return int(self.send_command(self.address, '013'))

    def set_measurement_type(self, measurement_type):
        """Defines the measurement type to use.

        =================== =========== ======
        Measurement type    Function    Value
        =================== =========== ======
        "Edge L rise"       Edge        0
        "Edge L fall"       Edge        1
        "Edge R rise"       Edge        2
        "Edge R fall"       Edge        3
        "Width"             Width       4
        "Center width"      Width       5
        "Gap"               Gap         6
        "Center gap"        Gap         7
        =================== =========== ======

        Parameters
        ----------
        measurement_type : :obj:`string`
            Measurement type.

        """
        if measurement_type not in self.MEASUREMENT_TYPES:
            raise ProtocolError('Unsupported measure type, must be one of ' + str(self.MEASUREMENT_TYPES))

        return self.send_command(self.address, '020', str(self.MEASUREMENT_TYPES.index(measurement_type)))

    def set_precision(self, precision):
        """Defines the precision the sensor will use to determine edges:

        ======== ============ ===============
        Value    Precision    Function values
        ======== ============ ===============
        ``0``    Standard     Median=off, Moving Average=off
        ``1``    High         Median=7, Moving Average=16
        ``2``    Very High    Median=15, Moving Average=128
        ======== ============ ===============

        Parameters
        ----------
        precision : :obj:`int`
            Sensor precision to use.

        Note
        ----
        The higher the precision, the slower the measurement gets.
        """
        if precision < 0 or precision > 2:
            raise ProtocolError('Precision must be 0 (standard), 1 (high) or 2 (very high)')
        return self.send_command(self.address, '040', str(precision))

    def set_edge_height(self, height):
        """Defines the minimum height of an edge to be detected.

        Parameters
        ----------
        height : :obj:`float`
            Minimum edge height.
        """
        return self.send_command(self.address, '042', str(height))

    def get_measurement(self):
        """Retrieves the current measurement of the sensor according to the current settings.

        Returns
        -------
        `tuple`
            The current measurement and additionally a value indicating the quality of the measured value.
        """
        result = self.send_command(self.address, '031')

        if len(result) != 2:
            raise ProtocolError('Unexpected result: ' + str(result))

        value = result[0]
        quality = int(result[1])

        # If Valid
        if quality == 0:
            value = float(value)

        return (value, self.QUALITY[quality])

    def get_live_monitor_data(self):
        """Retrieves the distance to the surface in the center of the laser beam and the
        angle at which it's found.

        Returns
        -------
        `list`
            angle and distance to the reference surface.

        Note
        ----
        This function is designed to aid in the installation of the sensor at an angle.
        """
        result = self.send_command(self.address, '093')
        if len(result) != 2:
            raise ProtocolError('Unexpected result: ' + str(result))

        return map(float, result)

    def reset(self):
        """Resets the sensor to factory settings."""
        self.send_command(self.address, '003')

    def activate_flex_mount(self, reference_thickness):
        """Activates the FLEX Mount feature of the sensor to allow positioning it on an
        angled installation. The reference thickness is only required if the surface is
        uneven and an additional leveling auxiliary plate as been added."""
        result = self.send_command(self.address, '062', str(reference_thickness))
        return map(float, result)

    def deactivate_flex_mount(self):
        """Deactivates the FLEX Mount feature."""
        self.send_command(self.address, '063')

    def set_flex_mount(self, angle, distance):
        """Sets the FLEX Mount feature to a specific angle and distance."""
        result = self.send_command(self.address, '060', '%.2f,%.2f' % (angle, distance))
        return map(float, result)

    def adjust_to_dark_object(self, is_dark_object):
        """Adjusts the sensor to detect darker or lighter surfaces."""
        data = '1' if is_dark_object else '0'
        return self.send_command(self.address, '044', data)


class PosConCM(SerialSensor):
    """Provides an interface for the `Baumer PosConCM round objects measurement sensor
    <http://www.baumer.com/us-en/products/distance-measurement/light-section-sensors/poscon-cm-measurement-of-round-objects/>`_.

    The sensor has different interfaces to retrieve its data. This
    class provides access to the serial interface (RS-485).
    This class is a context manager type, so it's best used in combination
    with the ``with`` statement to ensure resource deallocation.
    The protocol of the sensor when operated via RS-485 indicates that
    access to it must be locked programmatically before starting operations
    and unlocked on completion. This is handled automatically if you use
    this class on a ``with`` statement, otherwise, the methods ``begin()`` and
    ``end()`` must be invoked by hand.

    Attributes
    ----------
    serial : :obj:`serial.Serial`
        Instance of the serial port used to communicate with the sensor.
    address : :obj:`int`
        PosConCM sensors have an address assigned, which defaults to 1.
        There's also a broadcast address (``PosConCM.BROADCAST_ADDRESS``)
        that can be used to query the address of the sensor connected to
        the RS-485 bus. Only one sensor can be in the bus when using the
        broadcast address to query for sensor's address.

    Examples
    --------
    >>> from serial import Serial                                                   # doctest: +SKIP
    >>> with Serial('COM5', 57600, parity=PARITY_EVEN, timeout=1) as serial:                            # doctest: +SKIP
    >>>     with PosConCM(serial, PosConCM.BROADCAST_ADDRESS) as broadcast_query:   # doctest: +SKIP
    ...         addr = broadcast_query.get_address()                                # doctest: +SKIP
    ...                                                                             # doctest: +SKIP
    ...     with PosConCM(serial, addr) as sensor:                                  # doctest: +SKIP
    ...         sensor.set_measurement_type('X-Center')                             # doctest: +SKIP
    ...         sensor.set_precision(2)                                             # doctest: +SKIP
    ...         data = sensor.get_measurement()                                     # doctest: +SKIP
    """

    FRAME_HEAD = ':%s%s;%s;'
    FRAME_TAIL = '%s%s\r\n'
    BROADCAST_ADDRESS = 0
    MEASUREMENT_TYPES = {
        'Diameter': 28,
        'X_center': 29,
        'Z_center': 30,
        'X_left': 31,
        'X_right': 32,
        'Z_top': 33}
    QUALITY = {
        0: 'Valid',
        1: 'Low signal',
        2: 'No edge',
        3: 'Low signal, no edge',
        4: 'No signal'}
    ERROR_CODES = {
        '1': 'Wrong message type',
        '2': 'Wrong payload format',
        '3': 'Wrong argument',
        '4': 'Wrong argument count',
        '5': 'Not enough data',
        '6': 'Index do not exist',
        '7': 'Index locked',
        '8': 'Access not allowed',
        '9': 'Not enough memory for encoding',
        '10': 'Not possible to encode argument',
        '11': 'Application specific error',
        '12': 'Wrong state'}

    def __init__(self, serial, address):
        super(PosConCM, self).__init__(serial)
        self.address = address

    def __enter__(self):
        self.begin()
        return self

    def __exit__(self, *args):
        self.end()

    def begin(self):
        """Locks the sensor to start RS-485 communication.

        Note
        ----
        This method only needs to be called if not using
        a ``with`` statement to handle lifetime of the `PosConCM` instance.
        """
        return self.send_command(self.address, 'W010', '0')

    def end(self):
        """Unlocks the sensor from RS-485 communication.

        Note
        ----
        This method only needs to be called if not using
        a ``with`` statement to handle lifetime of the `PosConCM` instance.
        """
        return self.send_command(self.address, 'W010', '1')

    def format_command(self, address, command, data=None):
        """Formats the command."""
        data = data or ''
        frame = self.FRAME_HEAD % (str(address).zfill(2), command, data)
        return self.FRAME_TAIL % (frame, self.calculate_checksum(frame))

    def calculate_checksum(self, command):
        """Checks that message is complete."""
        return '****'

    def get_payload(self, result):
        """Gets payload."""
        frame_head = result[:-6]
        result_type = frame_head[3]

        if result_type == 'a':
            raise SensorTimeoutError('Sensor has not completed reading')

        if result_type == 'E':
            error_index = frame_head.split(';')
            raise ProtocolError('Application error, Result=%s' % frame_head +
                                'Error type: ' + str(self.ERROR_CODES[str(error_index[1])]))

        if result_type == 'B':
            raise ProtocolError('Sensor is busy, Result=%s' % frame_head)

        return result[5:-6].split(';')

    def send_command(self, address, command, data=None):
        """Sends a command to the sensor's address specified. The command
        can optionally contain a data string.
        This method is mostly for internal use, as the higher-level API is
        exposed via dedicated methods.

        Parameters
        ----------
        address : :obj:`int`
            PosConCM sensors have an address assigned, which defaults to 1.
            There's also a broadcast address (``PosConCM.BROADCAST_ADDRESS``)
            that can be used to query the address of the sensor connected to
            the RS-485 bus. Only one sensor can be in the bus when using the
            broadcast address to query for sensor's address.
        command : :obj:`string`
            A string indicating the command number to be executed with the
            W or R in front, depending on Writting or Reading.
        data : :obj:`string`
            An optional string of data that is sent together with the command.

        Returns
        -------
        `list` or value
            Result of the command. It can be a list or a single value depending on the operation.
        """
        for i in range(2):
            cmd = self.format_command(address, command, data)

            # Python 2 vs 3
            if hasattr(cmd, 'decode'):
                self.serial.write(cmd)
                result = self.serial.readline()
            else:
                self.serial.write(cmd.encode('ascii'))
                result = self.serial.readline().decode('ascii')

            if result:
                try:
                    return self.get_payload(result)
                except SensorTimeoutError:
                    time.sleep(0.5)
                    continue

            return None

    def get_address(self):
        """Gets the address of the RS-485 sensors currently connected to the bus. This command
        is only really useful when this class is initialized with the broadcast address,
        with the purpose of retrieving the address of a sensor connected.

        Returns
        -------
        `int`
            Address of the PosConCM sensor connected to the RS-485 bus.

        Note
        ----
        Only one PosConCM sensor can be connected to the bus for this operation to succeed.
        """
        result = self.send_command(self.address, 'R005')
        return int(result[0])

    def set_measurement_type(self, measurement_type):
        """Defines the measurement type to use.

        ===================  ======
        Measurement type     Value
        ===================  ======
        "Diameter"           28
        "X-Center position"  29
        "Z-Center position"  30
        "X-Left position"    31
        "X-Right position"   32
        "Z-Top position"     33
        ===================  ======

        Parameters
        ----------
        measurement_type : :obj:`string`
            Measurement type.
        """
        if measurement_type not in self.MEASUREMENT_TYPES:
            raise ProtocolError('Unsupported measure type, must be one of ' + str(self.MEASUREMENT_TYPES))

        return self.send_command(self.address, 'W020', str(self.MEASUREMENT_TYPES[measurement_type]))

    def set_precision(self, precision):
        """Defines the precision the sensor will use to determine edges:

        ======== ============ ===============================
        Value    Precision    Function values
        ======== ============ ===============================
        ``0``    Standard     Median=off,  Moving Average=off
        ``1``    High         Median=7,    Moving Average=16
        ``2``    Very High    Median=15,   Moving Average=128
        ======== ============ ===============================

        Parameters
        ----------
        precision : :obj:`int`
            Sensor precision to use.

        Note
        -----
        The higher the precision, the slower the measurement gets.
        """
        if precision < 0 or precision > 2:
            raise ProtocolError('Precision must be 0 (standard), 1 (high) or 2 (very high)')
        return self.send_command(self.address, 'W033', str(precision))

    def get_measurement(self):
        """Retrieves the current measurement of the sensor according to the current settings.

        Returns
        -------
        `tuple`
            The current measurement and additionally a value indicating the quality of the measured value.
        """
        result = self.send_command(self.address, 'R021')

        if len(result) != 3:
            raise ProtocolError('Unexpected result: ' + str(result))

        value = result[0]
        quality = int(result[1])

        # If Valid
        if quality == 0:
            value = float(value)

        return (value, self.QUALITY[quality])

    def activate_flex_mount(self):
        """Activates the FLEX Mount feature of the sensor to allow positioning it on an
        angled installation. The reference thickness is only required if the surface is
        uneven and an additional leveling auxiliary plate as been added."""
        result = self.send_command(self.address, 'W035', '1')
        return map(float, result)

    def deactivate_flex_mount(self):
        """Deactivates the FLEX Mount feature."""
        result = self.send_command(self.address, 'W035', '0')
        return map(float, result)

    def set_flex_mount(self, angle, distance):
        """Sets the FLEX Mount feature to a specific angle and distance."""
        data = '{:.2f};{:.2f}'.format(angle, distance)
        return self.send_command(self.address, 'W036', data)

    def teach_flex_mount(self, reference_thickness):
        """Sets the FLEX Mount feature to a specific angle and distance."""
        return self.send_command(self.address, 'W037', str(reference_thickness))

    def adjust_to_dark_object(self, is_dark_object):
        """Adjusts the sensor to detect darker or lighter surfaces."""
        data = '1' if is_dark_object else '0'
        result = self.send_command(self.address, 'W032', data)
        return map(float, result)

    def reset(self):
        """Resets the sensor to factory settings."""
        self.send_command(self.address, 'W202', '0')
