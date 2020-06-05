# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from walker_arm/TaskServiceRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TaskServiceRequest(genpy.Message):
  _md5sum = "2578414395b78e4280493cdc6999b7cd"
  _type = "walker_arm/TaskServiceRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string taskName
bool taskEnable
bool[] useJointOTG
bool[] useCartOTG
bool[] securityDection
bool[] collisionDetection

"""
  __slots__ = ['taskName','taskEnable','useJointOTG','useCartOTG','securityDection','collisionDetection']
  _slot_types = ['string','bool','bool[]','bool[]','bool[]','bool[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       taskName,taskEnable,useJointOTG,useCartOTG,securityDection,collisionDetection

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TaskServiceRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.taskName is None:
        self.taskName = ''
      if self.taskEnable is None:
        self.taskEnable = False
      if self.useJointOTG is None:
        self.useJointOTG = []
      if self.useCartOTG is None:
        self.useCartOTG = []
      if self.securityDection is None:
        self.securityDection = []
      if self.collisionDetection is None:
        self.collisionDetection = []
    else:
      self.taskName = ''
      self.taskEnable = False
      self.useJointOTG = []
      self.useCartOTG = []
      self.securityDection = []
      self.collisionDetection = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.taskName
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_B().pack(self.taskEnable))
      length = len(self.useJointOTG)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.useJointOTG))
      length = len(self.useCartOTG)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.useCartOTG))
      length = len(self.securityDection)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.securityDection))
      length = len(self.collisionDetection)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.collisionDetection))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.taskName = str[start:end].decode('utf-8')
      else:
        self.taskName = str[start:end]
      start = end
      end += 1
      (self.taskEnable,) = _get_struct_B().unpack(str[start:end])
      self.taskEnable = bool(self.taskEnable)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.useJointOTG = struct.unpack(pattern, str[start:end])
      self.useJointOTG = list(map(bool, self.useJointOTG))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.useCartOTG = struct.unpack(pattern, str[start:end])
      self.useCartOTG = list(map(bool, self.useCartOTG))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.securityDection = struct.unpack(pattern, str[start:end])
      self.securityDection = list(map(bool, self.securityDection))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.collisionDetection = struct.unpack(pattern, str[start:end])
      self.collisionDetection = list(map(bool, self.collisionDetection))
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.taskName
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_get_struct_B().pack(self.taskEnable))
      length = len(self.useJointOTG)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.useJointOTG.tostring())
      length = len(self.useCartOTG)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.useCartOTG.tostring())
      length = len(self.securityDection)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.securityDection.tostring())
      length = len(self.collisionDetection)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.collisionDetection.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.taskName = str[start:end].decode('utf-8')
      else:
        self.taskName = str[start:end]
      start = end
      end += 1
      (self.taskEnable,) = _get_struct_B().unpack(str[start:end])
      self.taskEnable = bool(self.taskEnable)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.useJointOTG = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.useJointOTG = list(map(bool, self.useJointOTG))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.useCartOTG = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.useCartOTG = list(map(bool, self.useCartOTG))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.securityDection = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.securityDection = list(map(bool, self.securityDection))
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.collisionDetection = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.collisionDetection = list(map(bool, self.collisionDetection))
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from walker_arm/TaskServiceResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class TaskServiceResponse(genpy.Message):
  _md5sum = "b543fbd3518c791be28589b850702201"
  _type = "walker_arm/TaskServiceResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
bool result
string message

"""
  __slots__ = ['result','message']
  _slot_types = ['bool','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,message

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TaskServiceResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = False
      if self.message is None:
        self.message = ''
    else:
      self.result = False
      self.message = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      buff.write(_get_struct_B().pack(self.result))
      _x = self.message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message = str[start:end].decode('utf-8')
      else:
        self.message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      buff.write(_get_struct_B().pack(self.result))
      _x = self.message
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 1
      (self.result,) = _get_struct_B().unpack(str[start:end])
      self.result = bool(self.result)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.message = str[start:end].decode('utf-8')
      else:
        self.message = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class TaskService(object):
  _type          = 'walker_arm/TaskService'
  _md5sum = '4fad11411fd801536b613979f3a27f21'
  _request_class  = TaskServiceRequest
  _response_class = TaskServiceResponse
