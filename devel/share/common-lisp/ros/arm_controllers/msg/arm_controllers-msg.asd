
(cl:in-package :asdf)

(defsystem "arm_controllers-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControllerJointState" :depends-on ("_package_ControllerJointState"))
    (:file "_package_ControllerJointState" :depends-on ("_package"))
  ))