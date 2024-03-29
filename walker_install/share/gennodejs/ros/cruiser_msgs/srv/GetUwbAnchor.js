// Auto-generated. Do not edit!

// (in-package cruiser_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let UwbAnchor = require('../msg/UwbAnchor.js');

//-----------------------------------------------------------

class GetUwbAnchorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetUwbAnchorRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUwbAnchorRequest
    let len;
    let data = new GetUwbAnchorRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cruiser_msgs/GetUwbAnchorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Get the uwb stations as a cruiser_msgs/UwbAnchors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUwbAnchorRequest(null);
    return resolved;
    }
};

class GetUwbAnchorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.anchor = null;
    }
    else {
      if (initObj.hasOwnProperty('anchor')) {
        this.anchor = initObj.anchor
      }
      else {
        this.anchor = new UwbAnchor();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetUwbAnchorResponse
    // Serialize message field [anchor]
    bufferOffset = UwbAnchor.serialize(obj.anchor, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetUwbAnchorResponse
    let len;
    let data = new GetUwbAnchorResponse(null);
    // Deserialize message field [anchor]
    data.anchor = UwbAnchor.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += UwbAnchor.getMessageSize(object.anchor);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cruiser_msgs/GetUwbAnchorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f4b95f067596a6c57f174e4ed94b6a5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    cruiser_msgs/UwbAnchor anchor
    
    
    ================================================================================
    MSG: cruiser_msgs/UwbAnchor
    geometry_msgs/PoseStamped[] anchors
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetUwbAnchorResponse(null);
    if (msg.anchor !== undefined) {
      resolved.anchor = UwbAnchor.Resolve(msg.anchor)
    }
    else {
      resolved.anchor = new UwbAnchor()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetUwbAnchorRequest,
  Response: GetUwbAnchorResponse,
  md5sum() { return '5f4b95f067596a6c57f174e4ed94b6a5'; },
  datatype() { return 'cruiser_msgs/GetUwbAnchor'; }
};
