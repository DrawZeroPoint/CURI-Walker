# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cruiser_msgs/JointResult.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class JointResult(genpy.Message):
  _md5sum = "60682833e270feaa2ef1ee50b6380b52"
  _type = "cruiser_msgs/JointResult"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string id
time stamp
int32 errCode
float64[16] joints

string[] name

# Joints index to control in array
uint32[] jointIndex

# Corresponding joints postion
# unit - radian;
# example - [0.54, 1.22, 1.39]
float64[] position

# Corresponding joints max speed
float64[] speed

# Corresponding joints motion time
# unit - millisecond
int64[] duration

float64[] current
"""
  __slots__ = ['id','stamp','errCode','joints','name','jointIndex','position','speed','duration','current']
  _slot_types = ['string','time','int32','float64[16]','string[]','uint32[]','float64[]','float64[]','int64[]','float64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,stamp,errCode,joints,name,jointIndex,position,speed,duration,current

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(JointResult, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = ''
      if self.stamp is None:
        self.stamp = genpy.Time()
      if self.errCode is None:
        self.errCode = 0
      if self.joints is None:
        self.joints = [0.] * 16
      if self.name is None:
        self.name = []
      if self.jointIndex is None:
        self.jointIndex = []
      if self.position is None:
        self.position = []
      if self.speed is None:
        self.speed = []
      if self.duration is None:
        self.duration = []
      if self.current is None:
        self.current = []
    else:
      self.id = ''
      self.stamp = genpy.Time()
      self.errCode = 0
      self.joints = [0.] * 16
      self.name = []
      self.jointIndex = []
      self.position = []
      self.speed = []
      self.duration = []
      self.current = []

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
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2Ii().pack(_x.stamp.secs, _x.stamp.nsecs, _x.errCode))
      buff.write(_get_struct_16d().pack(*self.joints))
      length = len(self.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.jointIndex)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.jointIndex))
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.position))
      length = len(self.speed)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.speed))
      length = len(self.duration)
      buff.write(_struct_I.pack(length))
      pattern = '<%sq'%length
      buff.write(struct.pack(pattern, *self.duration))
      length = len(self.current)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.pack(pattern, *self.current))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8')
      else:
        self.id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.stamp.secs, _x.stamp.nsecs, _x.errCode,) = _get_struct_2Ii().unpack(str[start:end])
      start = end
      end += 128
      self.joints = _get_struct_16d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.jointIndex = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.position = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.speed = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sq'%length
      start = end
      end += struct.calcsize(pattern)
      self.duration = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.current = struct.unpack(pattern, str[start:end])
      self.stamp.canon()
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
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_2Ii().pack(_x.stamp.secs, _x.stamp.nsecs, _x.errCode))
      buff.write(self.joints.tostring())
      length = len(self.name)
      buff.write(_struct_I.pack(length))
      for val1 in self.name:
        length = len(val1)
        if python3 or type(val1) == unicode:
          val1 = val1.encode('utf-8')
          length = len(val1)
        buff.write(struct.pack('<I%ss'%length, length, val1))
      length = len(self.jointIndex)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.jointIndex.tostring())
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.position.tostring())
      length = len(self.speed)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.speed.tostring())
      length = len(self.duration)
      buff.write(_struct_I.pack(length))
      pattern = '<%sq'%length
      buff.write(self.duration.tostring())
      length = len(self.current)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.current.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.stamp is None:
        self.stamp = genpy.Time()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8')
      else:
        self.id = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.stamp.secs, _x.stamp.nsecs, _x.errCode,) = _get_struct_2Ii().unpack(str[start:end])
      start = end
      end += 128
      self.joints = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=16)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.name = []
      for i in range(0, length):
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1 = str[start:end].decode('utf-8')
        else:
          val1 = str[start:end]
        self.name.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.jointIndex = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.position = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.speed = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sq'%length
      start = end
      end += struct.calcsize(pattern)
      self.duration = numpy.frombuffer(str[start:end], dtype=numpy.int64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      end += struct.calcsize(pattern)
      self.current = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      self.stamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_16d = None
def _get_struct_16d():
    global _struct_16d
    if _struct_16d is None:
        _struct_16d = struct.Struct("<16d")
    return _struct_16d
_struct_2Ii = None
def _get_struct_2Ii():
    global _struct_2Ii
    if _struct_2Ii is None:
        _struct_2Ii = struct.Struct("<2Ii")
    return _struct_2Ii
