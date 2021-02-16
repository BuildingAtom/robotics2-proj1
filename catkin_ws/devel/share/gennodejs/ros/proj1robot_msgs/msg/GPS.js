// Auto-generated. Do not edit!

// (in-package proj1robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GPS {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.beacon00 = null;
      this.beacon01 = null;
      this.beacon10 = null;
      this.beacon11 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('beacon00')) {
        this.beacon00 = initObj.beacon00
      }
      else {
        this.beacon00 = 0.0;
      }
      if (initObj.hasOwnProperty('beacon01')) {
        this.beacon01 = initObj.beacon01
      }
      else {
        this.beacon01 = 0.0;
      }
      if (initObj.hasOwnProperty('beacon10')) {
        this.beacon10 = initObj.beacon10
      }
      else {
        this.beacon10 = 0.0;
      }
      if (initObj.hasOwnProperty('beacon11')) {
        this.beacon11 = initObj.beacon11
      }
      else {
        this.beacon11 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GPS
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [beacon00]
    bufferOffset = _serializer.float64(obj.beacon00, buffer, bufferOffset);
    // Serialize message field [beacon01]
    bufferOffset = _serializer.float64(obj.beacon01, buffer, bufferOffset);
    // Serialize message field [beacon10]
    bufferOffset = _serializer.float64(obj.beacon10, buffer, bufferOffset);
    // Serialize message field [beacon11]
    bufferOffset = _serializer.float64(obj.beacon11, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GPS
    let len;
    let data = new GPS(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [beacon00]
    data.beacon00 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beacon01]
    data.beacon01 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beacon10]
    data.beacon10 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beacon11]
    data.beacon11 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'proj1robot_msgs/GPS';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a88cfec89b2d3899365b3c2815c9280';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # A basic message for the local GPS beacons.
    
    Header header       # Reference shenanigans
    
    float64 beacon00    # Distance [m] from the beacon at [0,0,4]
    float64 beacon01    # Distance [m] from the beacon at [0,10,4]
    float64 beacon10    # Distance [m] from the beacon at [10,0,4]
    float64 beacon11    # Distance [m] from the beacon at [10,10,4]
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GPS(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.beacon00 !== undefined) {
      resolved.beacon00 = msg.beacon00;
    }
    else {
      resolved.beacon00 = 0.0
    }

    if (msg.beacon01 !== undefined) {
      resolved.beacon01 = msg.beacon01;
    }
    else {
      resolved.beacon01 = 0.0
    }

    if (msg.beacon10 !== undefined) {
      resolved.beacon10 = msg.beacon10;
    }
    else {
      resolved.beacon10 = 0.0
    }

    if (msg.beacon11 !== undefined) {
      resolved.beacon11 = msg.beacon11;
    }
    else {
      resolved.beacon11 = 0.0
    }

    return resolved;
    }
};

module.exports = GPS;
