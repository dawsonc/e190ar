
(cl:in-package :asdf)

(defsystem "e190_bot-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :e190_bot-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "path_Service" :depends-on ("_package_path_Service"))
    (:file "_package_path_Service" :depends-on ("_package"))
  ))