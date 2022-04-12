# generated from rosidl_generator_py/resource/_idl.py.em
# with input from add_on_msgs:srv/GetPrimAttribute.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetPrimAttribute_Request(type):
    """Metaclass of message 'GetPrimAttribute_Request'."""

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
            module = import_type_support('add_on_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'add_on_msgs.srv.GetPrimAttribute_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_prim_attribute__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_prim_attribute__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_prim_attribute__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_prim_attribute__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_prim_attribute__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetPrimAttribute_Request(metaclass=Metaclass_GetPrimAttribute_Request):
    """Message class 'GetPrimAttribute_Request'."""

    __slots__ = [
        '_path',
        '_attribute',
    ]

    _fields_and_field_types = {
        'path': 'string',
        'attribute': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.path = kwargs.get('path', str())
        self.attribute = kwargs.get('attribute', str())

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
        if self.path != other.path:
            return False
        if self.attribute != other.attribute:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'path' field must be of type 'str'"
        self._path = value

    @property
    def attribute(self):
        """Message field 'attribute'."""
        return self._attribute

    @attribute.setter
    def attribute(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'attribute' field must be of type 'str'"
        self._attribute = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_GetPrimAttribute_Response(type):
    """Metaclass of message 'GetPrimAttribute_Response'."""

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
            module = import_type_support('add_on_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'add_on_msgs.srv.GetPrimAttribute_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_prim_attribute__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_prim_attribute__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_prim_attribute__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_prim_attribute__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_prim_attribute__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetPrimAttribute_Response(metaclass=Metaclass_GetPrimAttribute_Response):
    """Message class 'GetPrimAttribute_Response'."""

    __slots__ = [
        '_value',
        '_type',
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'value': 'string',
        'type': 'string',
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.value = kwargs.get('value', str())
        self.type = kwargs.get('type', str())
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.value != other.value:
            return False
        if self.type != other.type:
            return False
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def value(self):
        """Message field 'value'."""
        return self._value

    @value.setter
    def value(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'value' field must be of type 'str'"
        self._value = value

    @property  # noqa: A003
    def type(self):  # noqa: A003
        """Message field 'type'."""
        return self._type

    @type.setter  # noqa: A003
    def type(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'type' field must be of type 'str'"
        self._type = value

    @property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_GetPrimAttribute(type):
    """Metaclass of service 'GetPrimAttribute'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('add_on_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'add_on_msgs.srv.GetPrimAttribute')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_prim_attribute

            from add_on_msgs.srv import _get_prim_attribute
            if _get_prim_attribute.Metaclass_GetPrimAttribute_Request._TYPE_SUPPORT is None:
                _get_prim_attribute.Metaclass_GetPrimAttribute_Request.__import_type_support__()
            if _get_prim_attribute.Metaclass_GetPrimAttribute_Response._TYPE_SUPPORT is None:
                _get_prim_attribute.Metaclass_GetPrimAttribute_Response.__import_type_support__()


class GetPrimAttribute(metaclass=Metaclass_GetPrimAttribute):
    from add_on_msgs.srv._get_prim_attribute import GetPrimAttribute_Request as Request
    from add_on_msgs.srv._get_prim_attribute import GetPrimAttribute_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
