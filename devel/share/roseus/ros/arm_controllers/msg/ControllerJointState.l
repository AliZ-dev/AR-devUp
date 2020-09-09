;; Auto-generated. Do not edit!


(when (boundp 'arm_controllers::ControllerJointState)
  (if (not (find-package "ARM_CONTROLLERS"))
    (make-package "ARM_CONTROLLERS"))
  (shadow 'ControllerJointState (find-package "ARM_CONTROLLERS")))
(unless (find-package "ARM_CONTROLLERS::CONTROLLERJOINTSTATE")
  (make-package "ARM_CONTROLLERS::CONTROLLERJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ControllerJointState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arm_controllers::ControllerJointState
  :super ros::object
  :slots (_header _name _command _command_dot _state _state_dot _error _error_dot _effort_command _effort_feedforward _effort_feedback ))

(defmethod arm_controllers::ControllerJointState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:command __command) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:command_dot __command_dot) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:state __state) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:state_dot __state_dot) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error __error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error_dot __error_dot) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort_command __effort_command) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort_feedforward __effort_feedforward) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort_feedback __effort_feedback) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _command __command)
   (setq _command_dot __command_dot)
   (setq _state __state)
   (setq _state_dot __state_dot)
   (setq _error __error)
   (setq _error_dot __error_dot)
   (setq _effort_command __effort_command)
   (setq _effort_feedforward __effort_feedforward)
   (setq _effort_feedback __effort_feedback)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:command_dot
   (&optional __command_dot)
   (if __command_dot (setq _command_dot __command_dot)) _command_dot)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:state_dot
   (&optional __state_dot)
   (if __state_dot (setq _state_dot __state_dot)) _state_dot)
  (:error
   (&optional __error)
   (if __error (setq _error __error)) _error)
  (:error_dot
   (&optional __error_dot)
   (if __error_dot (setq _error_dot __error_dot)) _error_dot)
  (:effort_command
   (&optional __effort_command)
   (if __effort_command (setq _effort_command __effort_command)) _effort_command)
  (:effort_feedforward
   (&optional __effort_feedforward)
   (if __effort_feedforward (setq _effort_feedforward __effort_feedforward)) _effort_feedforward)
  (:effort_feedback
   (&optional __effort_feedback)
   (if __effort_feedback (setq _effort_feedback __effort_feedback)) _effort_feedback)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float64[] _command
    (* 8    (length _command)) 4
    ;; float64[] _command_dot
    (* 8    (length _command_dot)) 4
    ;; float64[] _state
    (* 8    (length _state)) 4
    ;; float64[] _state_dot
    (* 8    (length _state_dot)) 4
    ;; float64[] _error
    (* 8    (length _error)) 4
    ;; float64[] _error_dot
    (* 8    (length _error_dot)) 4
    ;; float64[] _effort_command
    (* 8    (length _effort_command)) 4
    ;; float64[] _effort_feedforward
    (* 8    (length _effort_feedforward)) 4
    ;; float64[] _effort_feedback
    (* 8    (length _effort_feedback)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _command
     (write-long (length _command) s)
     (dotimes (i (length _command))
       (sys::poke (elt _command i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _command_dot
     (write-long (length _command_dot) s)
     (dotimes (i (length _command_dot))
       (sys::poke (elt _command_dot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _state
     (write-long (length _state) s)
     (dotimes (i (length _state))
       (sys::poke (elt _state i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _state_dot
     (write-long (length _state_dot) s)
     (dotimes (i (length _state_dot))
       (sys::poke (elt _state_dot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _error
     (write-long (length _error) s)
     (dotimes (i (length _error))
       (sys::poke (elt _error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _error_dot
     (write-long (length _error_dot) s)
     (dotimes (i (length _error_dot))
       (sys::poke (elt _error_dot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort_command
     (write-long (length _effort_command) s)
     (dotimes (i (length _effort_command))
       (sys::poke (elt _effort_command i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort_feedforward
     (write-long (length _effort_feedforward) s)
     (dotimes (i (length _effort_feedforward))
       (sys::poke (elt _effort_feedforward i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort_feedback
     (write-long (length _effort_feedback) s)
     (dotimes (i (length _effort_feedback))
       (sys::poke (elt _effort_feedback i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _command (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _command i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _command_dot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _command_dot (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _command_dot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _state_dot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state_dot (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _state_dot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _error_dot
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _error_dot (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _error_dot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort_command
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort_command (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort_command i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort_feedforward
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort_feedforward (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort_feedforward i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort_feedback
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort_feedback (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort_feedback i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get arm_controllers::ControllerJointState :md5sum-) "e2a6814d254fca29f6f31d3dc29611cc")
(setf (get arm_controllers::ControllerJointState :datatype-) "arm_controllers/ControllerJointState")
(setf (get arm_controllers::ControllerJointState :definition-)
      "std_msgs/Header header
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

")



(provide :arm_controllers/ControllerJointState "e2a6814d254fca29f6f31d3dc29611cc")


