
(cl:in-package :asdf)

(defsystem "servo_ctrl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EcatGetPVT" :depends-on ("_package_EcatGetPVT"))
    (:file "_package_EcatGetPVT" :depends-on ("_package"))
    (:file "EcatGetVer" :depends-on ("_package_EcatGetVer"))
    (:file "_package_EcatGetVer" :depends-on ("_package"))
    (:file "EcatLimitSrv" :depends-on ("_package_EcatLimitSrv"))
    (:file "_package_EcatLimitSrv" :depends-on ("_package"))
    (:file "EcatPidSrv" :depends-on ("_package_EcatPidSrv"))
    (:file "_package_EcatPidSrv" :depends-on ("_package"))
    (:file "EcatSetZero" :depends-on ("_package_EcatSetZero"))
    (:file "_package_EcatSetZero" :depends-on ("_package"))
    (:file "SendFtAtCmd" :depends-on ("_package_SendFtAtCmd"))
    (:file "_package_SendFtAtCmd" :depends-on ("_package"))
    (:file "SetBasePower" :depends-on ("_package_SetBasePower"))
    (:file "_package_SetBasePower" :depends-on ("_package"))
    (:file "SetFtOffset" :depends-on ("_package_SetFtOffset"))
    (:file "_package_SetFtOffset" :depends-on ("_package"))
    (:file "SetFtPeriod" :depends-on ("_package_SetFtPeriod"))
    (:file "_package_SetFtPeriod" :depends-on ("_package"))
  ))