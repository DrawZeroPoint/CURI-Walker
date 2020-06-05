# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from cruiser_msgs/cruiserLocomotionStatus.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class cruiserLocomotionStatus(genpy.Message):
  _md5sum = "e713c1a9d9ca1e3ec2ae4bd0d8ac35e3"
  _type = "cruiser_msgs/cruiserLocomotionStatus"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 status
float32 vx
float32 vy
float32 vz

"""
  __slots__ = ['status','vx','vy','vz']
  _slot_types = ['uint32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       status,vx,vy,vz

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(cruiserLocomotionStatus, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.status is None:
        self.status = 0
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.vz is None:
        self.vz = 0.
    else:
      self.status = 0
      self.vx = 0.
      self.vy = 0.
      self.vz = 0.

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
      _x = self
      buff.write(_get_struct_I3f().pack(_x.status, _x.vx, _x.vy, _x.vz))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.status, _x.vx, _x.vy, _x.vz,) = _get_struct_I3f().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_I3f().pack(_x.status, _x.vx, _x.vy, _x.vz))
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
      _x = self
      start = end
      end += 16
      (_x.status, _x.vx, _x.vy, _x.vz,) = _get_struct_I3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_I3f = None
def _get_struct_I3f():
    global _struct_I3f
    if _struct_I3f is None:
        _struct_I3f = struct.Struct("<I3f")
    return _struct_I3f
