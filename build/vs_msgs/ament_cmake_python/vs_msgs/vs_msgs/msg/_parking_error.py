# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vs_msgs:msg/ParkingError.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ParkingError(type):
    """Metaclass of message 'ParkingError'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('vs_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vs_msgs.msg.ParkingError')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__parking_error
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__parking_error
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__parking_error
            cls._TYPE_SUPPORT = module.type_support_msg__msg__parking_error
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__parking_error

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ParkingError(metaclass=Metaclass_ParkingError):
    """Message class 'ParkingError'."""

    __slots__ = [
        '_x_error',
        '_y_error',
        '_distance_error',
    ]

    _fields_and_field_types = {
        'x_error': 'float',
        'y_error': 'float',
        'distance_error': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x_error = kwargs.get('x_error', float())
        self.y_error = kwargs.get('y_error', float())
        self.distance_error = kwargs.get('distance_error', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.x_error != other.x_error:
            return False
        if self.y_error != other.y_error:
            return False
        if self.distance_error != other.distance_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_error(self):
        """Message field 'x_error'."""
        return self._x_error

    @x_error.setter
    def x_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_error = value

    @builtins.property
    def y_error(self):
        """Message field 'y_error'."""
        return self._y_error

    @y_error.setter
    def y_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_error = value

    @builtins.property
    def distance_error(self):
        """Message field 'distance_error'."""
        return self._distance_error

    @distance_error.setter
    def distance_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'distance_error' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'distance_error' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._distance_error = value
