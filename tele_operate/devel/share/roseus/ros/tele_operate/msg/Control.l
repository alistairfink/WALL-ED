;; Auto-generated. Do not edit!


(when (boundp 'tele_operate::Control)
  (if (not (find-package "TELE_OPERATE"))
    (make-package "TELE_OPERATE"))
  (shadow 'Control (find-package "TELE_OPERATE")))
(unless (find-package "TELE_OPERATE::CONTROL")
  (make-package "TELE_OPERATE::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html


(defclass tele_operate::Control
  :super ros::object
  :slots (_control ))

(defmethod tele_operate::Control
  (:init
   (&key
    ((:control __control) 0)
    )
   (send-super :init)
   (setq _control (round __control))
   self)
  (:control
   (&optional __control)
   (if __control (setq _control __control)) _control)
  (:serialization-length
   ()
   (+
    ;; int64 _control
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _control
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _control (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _control) (= (length (_control . bv)) 2)) ;; bignum
              (write-long (ash (elt (_control . bv) 0) 0) s)
              (write-long (ash (elt (_control . bv) 1) -1) s))
             ((and (class _control) (= (length (_control . bv)) 1)) ;; big1
              (write-long (elt (_control . bv) 0) s)
              (write-long (if (>= _control 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _control s)(write-long (if (>= _control 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _control
#+(or :alpha :irix6 :x86_64)
      (setf _control (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _control (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get tele_operate::Control :md5sum-) "69939a687db9b3fdb214ee8d790057ba")
(setf (get tele_operate::Control :datatype-) "tele_operate/Control")
(setf (get tele_operate::Control :definition-)
      "int64 control

")



(provide :tele_operate/Control "69939a687db9b3fdb214ee8d790057ba")


