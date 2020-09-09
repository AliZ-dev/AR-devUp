// Auto-generated. Do not edit!

// (in-package arm_controllers.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ControllerJointState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.name = null;
      this.command = null;
      this.command_dot = null;
      this.state = null;
      this.state_dot = null;
      this.error = null;
      this.error_dot = null;
      this.effort_command = null;
      this.effort_feedforward = null;
      this.effort_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = [];
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = [];
      }
      if (initObj.hasOwnProperty('command_dot')) {
        this.command_dot = initObj.command_dot
      }
      else {
        this.command_dot = [];
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = [];
      }
      if (initObj.hasOwnProperty('state_dot')) {
        this.state_dot = initObj.state_dot
      }
      else {
        this.state_dot = [];
      }
      if (initObj.hasOwnProperty('error')) {
        this.error = initObj.error
      }
      else {
        this.error = [];
      }
      if (initObj.hasOwnProperty('error_dot')) {
        this.error_dot = initObj.error_dot
      }
      else {
        this.error_dot = [];
      }
      if (initObj.hasOwnProperty('effort_command')) {
        this.effort_command = initObj.effort_command
      }
      else {
        this.effort_command = [];
      }
      if (initObj.hasOwnProperty('effort_feedforward')) {
        this.effort_feedforward = initObj.effort_feedforward
      }
      else {
        this.effort_feedforward = [];
      }
      if (initObj.hasOwnProperty('effort_feedback')) {
        this.effort_feedback = initObj.effort_feedback
      }
      else {
        this.effort_feedback = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerJointState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _arraySerializer.string(obj.name, buffer, bufferOffset, null);
    // Serialize message field [command]
    bufferOffset = _arraySerializer.float64(obj.command, buffer, bufferOffset, null);
    // Serialize message field [command_dot]
    bufferOffset = _arraySerializer.float64(obj.command_dot, buffer, bufferOffset, null);
    // Serialize message field [state]
    bufferOffset = _arraySerializer.float64(obj.state, buffer, bufferOffset, null);
    // Serialize message field [state_dot]
    bufferOffset = _arraySerializer.float64(obj.state_dot, buffer, bufferOffset, null);
    // Serialize message field [error]
    bufferOffset = _arraySerializer.float64(obj.error, buffer, bufferOffset, null);
    // Serialize message field [error_dot]
    bufferOffset = _arraySerializer.float64(obj.error_dot, buffer, bufferOffset, null);
    // Serialize message field [effort_command]
    bufferOffset = _arraySerializer.float64(obj.effort_command, buffer, bufferOffset, null);
    // Serialize message field [effort_feedforward]
    bufferOffset = _arraySerializer.float64(obj.effort_feedforward, buffer, bufferOffset, null);
    // Serialize message field [effort_feedback]
    bufferOffset = _arraySerializer.float64(obj.effort_feedback, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerJointState
    let len;
    let data = new ControllerJointState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [command]
    data.command = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [command_dot]
    data.command_dot = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [state]
    data.state = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [state_dot]
    data.state_dot = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [error]
    data.error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [error_dot]
    data.error_dot = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort_command]
    data.effort_command = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort_feedforward]
    data.effort_feedforward = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort_feedback]
    data.effort_feedback = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.name.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.command.length;
    length += 8 * object.command_dot.length;
    length += 8 * object.state.length;
    length += 8 * object.state_dot.length;
    length += 8 * object.error.length;
    length += 8 * object.error_dot.length;
    length += 8 * object.effort_command.length;
    length += 8 * object.effort_feedforward.length;
    length += 8 * object.effort_feedback.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arm_controllers/ControllerJointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2a6814d254fca29f6f31d3dc29611cc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    string[] name
    float64[] command
    float64[] command_dot
    float64[] state
    float64[] state_dot
    float64[] error
    float64[] error_dot
    float64[] effort_command
    float64[] effort_feedforward
    float64[] effort_feedback
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
    const resolved = new ControllerJointState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = []
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = []
    }

    if (msg.command_dot !== undefined) {
      resolved.command_dot = msg.command_dot;
    }
    else {
      resolved.command_dot = []
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = []
    }

    if (msg.state_dot !== undefined) {
      resolved.state_dot = msg.state_dot;
    }
    else {
      resolved.state_dot = []
    }

    if (msg.error !== undefined) {
      resolved.error = msg.error;
    }
    else {
      resolved.error = []
    }

    if (msg.error_dot !== undefined) {
      resolved.error_dot = msg.error_dot;
    }
    else {
      resolved.error_dot = []
    }

    if (msg.effort_command !== undefined) {
      resolved.effort_command = msg.effort_command;
    }
    else {
      resolved.effort_command = []
    }

    if (msg.effort_feedforward !== undefined) {
      resolved.effort_feedforward = msg.effort_feedforward;
    }
    else {
      resolved.effort_feedforward = []
    }

    if (msg.effort_feedback !== undefined) {
      resolved.effort_feedback = msg.effort_feedback;
    }
    else {
      resolved.effort_feedback = []
    }

    return resolved;
    }
};

module.exports = ControllerJointState;
