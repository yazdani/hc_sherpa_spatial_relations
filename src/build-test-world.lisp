;;; Copyright (c) 2014, Fereshta Yazdani <yazdani@cs.uni-bremen.de>
;;; All rights reserved.
;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to 
;;;       endorse or promote products derived from this software without 
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :sherpa)
(defvar *joint-states* nil
  "List of current joint states as published by /joint_states.")
(defvar *joint-states-subscriber* nil
  "Subscriber to /joint_states.")

;; (defun start-scenario ()
;;  (start-bullet-with-robots)
;;  (spawn-robot)
;;  (spawn-tree)
;;  (pointing-direction)
;;  s
;; s
;; s

(defun start-myros ()
  (roslisp:ros-info (sherpa-spatial-relations) "START the ROSNODE")
  (roslisp-utilities:startup-ros :anonymous nil))

(defun end-myros ()
  (roslisp:ros-info (sherpa-spatial-relations) "KILL the ROSNODE")
  (roslisp-utilities:shutdown-ros))

;; HERE WE ARE AT THE BEGINNING OF OUT PROGRAMM AND WOULD LIKE TO
;; START THE REASONING. BUT BEFORE STARTING WE HAVE TO BUILD OUR
;; SYSTEM, THAT MEANS WE HAVE TO BUILD THE BULLET WORLD WITH ALL
;; THE IMPORTANT OBJECTS INTO THE WORLD. THE FOLLOWING METHODS
;; ARE SHOWING US, HOW WE CAN START THE BULLET WORLD AND SPAWN
;; ROBOTS AND ENVIRONMENT SPECIFIC OBJECTS INTO THIS WOLD.

(defun start-bullet-with-robots()
  (roslisp:ros-info (sherpa-spatial-relations) "SPAWN ROBOTS INTO WORLD")
  (setf *list* nil)
  (let* ((genius-urdf (cl-urdf:parse-urdf (roslisp:get-param "genius/robot_description")))
         (quad-urdf (cl-urdf:parse-urdf (roslisp:get-param "quad/robot_description")))
         (rover-urdf (cl-urdf:parse-urdf (roslisp:get-param "rover/robot_description"))) 
)
    (setf *list*
          (car 
           (force-ll
            (prolog
             `(and
               (clear-bullet-world)
               (bullet-world ?w)
               (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1))
                               :normal (0 0 1) :constant 0))
               (debug-window ?w)
               (assert (object ?w urdf genius ((0 0 0) (0 0 1 1)) :urdf ,genius-urdf))
               (assert (object ?w urdf quad ((0 1 2) (0 0 0 1)) :urdf ,quad-urdf))
               (assert (object ?w urdf rover ((2 3 0) (0 0 0 1)) :urdf ,rover-urdf))
             )))))))
(defun spawn-tree ()
  (roslisp:ros-info (sherpa-spatial-relations) "SPAWN TREE INTO WORLD")
  (force-ll (prolog `(and (bullet-world ?w)
                          (assert (object ?w mesh tree-1 ((9 -4 0)(0 0 0 1))
                                          :mesh tree1 :mass 0.2 :color (0 0 0)))
                          (assert (object ?w mesh tree-3 ((9 -5 0)(0 0 0 1))
                                          :mesh tree3 :mass 0.2 :color (0 0 0)))
                          (assert (object ?w mesh tree-4 ((6 0 0)(0 0 0 1))
                                          :mesh tree4 :mass 0.2 :color (0 0 0)))
                          (assert (object ?w mesh tree-8 ((6 1 0)(0 0 0 1))
                                          :mesh tree3 :mass 0.2 :color (0 0 0)))
                          (assert (object ?w mesh tree-9 ((6.5 2 0)(0 0 0 1))
                                          :mesh tree2 :mass 0.2 :color (0 0 0)))
                          (assert (object ?w mesh tree-10 ((5.5 3 0)(0 0 0 1))
                                          :mesh tree1 :mass 0.2 :color (0 0 0)))))))

(defun spawn-object ()
"we spawn the subject of interest"
 (force-ll 
   (prolog `(and (bullet-world ?w)
		 (assert (object ?w mesh hat ((7 0 0)(0 0 0 1))
				 :mesh hat :mass 0.2 :color (1 0 0)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;POSITION AND NAMES OF OBJECTS IN WORLD;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-pose ( name)
 (prolog `(joint-state ,name "right_shoulder_joint_x"))                            
)

(defun get-object-pose (name)
  (let* ((lists (force-ll
              (prolog `(and (bullet-world ?w)
                            (object-pose ?w ,name ?pose)))))
         (list (car lists))
         (a-list (assoc '?pose list)))
    (cdr a-list)))

(defun robot-name-extract-from-world (rob-name)
  (let ((list (force-ll 
               (prolog `(and (bullet-world ?w) 
                             (object-type ?w ?robot robot-object))))))
    (loop
      (when (equal (car list) nil) (return))
      (when (equal rob-name (cdr (assoc '?robot (car list)))) (return (cdr (assoc '?robot (car list))))) 
      (setf list (cdr list)))))

;; AT THIS PART OF THIS PROGRAMM WE WILL NOW WORKING WITH COMMAND AND INSTRUCTIONS IN FORM OF
;; HUMAN GESTURES.


(defun pointing-direction ()
  (pointing-into-bullet)
 ; (pointing-into-gazebo)
  (arm-extension-bullet)
;  (arm-extension-gazebo)
)


;; CREATING THE COSTMAP AND GENERATING SAMPLES FOR THE ROBOT 
;; WHERE TO MOVE AND TO NAVIGATE. FOR THE HUMAN WE WILL
;; RESTRICT THE AREA OF THE COSTMAP WITH CONDITIONS.

(defun create-sampling-area-bullet ()
  (let* ((x-val (+ 1 (get-joint-value-bullet "right_shoulder_joint_x"))) ;;move 2 m
         (y-val (get-joint-value-bullet "right_shoulder_joint_y"))
         (z-val (get-joint-value-bullet "right_shoulder_joint_z"))
         (transform (cl-transforms:make-pose (cl-transforms:make-3d-vector x-val y-val z-val)
                                             (cl-transforms:make-quaternion 0 0 0 1))))

         (make-designator 'desig-props:location `((go-to ,transform)))))
                     
(defun create-sampling-area-gazebo ())                    












;FILE POINTING-GESTURES.LISP
;(poniting-into-gazebo)
;(poniting-into-bullet)

;;;;;;;;;;;;;;CREATE DESIGNATORS;;;;;;;;;;;;;;;;;;;;;;
(defun make-obj-desig-close-to-tree ()
 (format t "create object desig for tree ~%")
 (let* ((transform (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 0 0)
                                                  (cl-transforms:make-quaternion 0 0 0 1))))
         (make-designator 'desig-props:location `((right-of ,transform)))))

(defun create-object-costmap ())
;; instead of giving the exactly pose, we can look for the object of interest into the environment
;; and call the pose.
 ;; (let* ((transform (cl-transforms:make-transform (cl-transforms:make-3d-vector 0 1 2)
   ;;                                               (cl-transforms:make-quaternion 0 0 0 1)))
     ;;     (make-designator 'desig-props:location `((right-of ,transform))))))

(defun make-obj-desig-close-to-robot ()
 (format t "create object desig for robot ~%")
 (let* ((pose (get-object-pose 'hat))
         (desig (make-designator 'desig-props:location `((right-of ,pose)))))
   (reference desig) ))


			      
(defun go-to-obj (obj-pose)
  "this function returns a designator by getting the obj-pose and generating the costmap from the
  view of the object"
  (let ((desig (make-designator 'desig-props:location `((right-of ,obj-pose)))))
    (reference desig)))


(def-fact-group build-test-world ()
  (<- (compute-obj-pose ?desig)
    (once
     (bound ?desig)
     (bullet-world ?w)
     (format "hello~%")
     (desig-solutions ?desig ?solutions)
     (format "1 ~a~%" ?solutions)
     (take 1 ?solutions ?8-solutions) 
     (btr::generate ?poses-on (btr::obj-poses-on 'tree-4 ?8-solutions ?w))
     (member ?solution ?poses-on)
     (assert (object-pose ?w 'tree-4 ?solution)))))



    

;;;;;;;;;;;;;Commented;;;;;;;;;;;;;

;; (defun go-into-direction ()
;;   (roslisp:ros-info (sherpa-spatial-relations) "GO INTO POINTED DIRECTION")
;;  ; (change-tree)
;;   (let* ((transform-1 (cl-transforms:make-transform (cl-transforms:make-3d-vector 6 -0.5 0)
;;                                                     (cl-transforms:make-quaternion 0 0 0 1)))
;;          (desig (make-designator 'desig-props:location `((right-of ,transform-1))))
         
;;          (pose (reference desig)))
;;     (roslisp:ros-info (sherpa-spatial-relations) "RETURN POSITION ~a FOR ROBOT TO GO" pose)
;;     (prolog `(and 
;;               (bullet-world ?w)
;;               (assert (object-pose ?w quad-1 ,pose))))))


;; (defun go-into-default ()
;;   (let* ((transform-1 (cl-transforms:make-transform (cl-transforms:make-3d-vector 6 -0.5 0)
;;                                                     (cl-transforms:make-quaternion 0 0 0 1)))
;;          (desig (make-designator 'desig-props:location `((used-arm ,transform-1))))
         
;;          (pose (reference desig)))
;;     (prolog `(and 
;;               (bullet-world ?w)
;;               (assert (object-pose ?w quad-1 ,pose))))))

;;(defun pointing-into-direction ()
;; (crs:prolog
;; `(assert (btr:joint-state ?w genius(( "right_upper_arm_joint_x" 0.1) ;;0.1
;;                                     ( "right_upper_arm_joint_y" 0.4)  ;;0.0 0.40
;;                                     ( "right_upper_arm_joint_z" 0.500)  ;;0.6 0.500
;;                                     )))))

;; (defun compute-multiple-obj-pose ()
;;   (format t "we are inside compute multiple ~%")
;;   (let ((desig-1 (make-obj-desig-close-to-tree))
;;         (desig-2 (make-obj-desig-close-to-robot)))
;;     (crs:prolog `(compute-obj-pose ,desig))
;;     ))



;;execute a certain position with the joints  
;;(defun pr2-execute-trajectory ()
;;  (crs:prolog
;;   `(assert (btr:joint-state ?w ?robot (("r_shoulder_pan_joint" 0.0)
;;                                        ("r_shoulder_lift_joint" -0.5) 
;;                                        ("r_upper_arm_roll_joint" 0.0)
;;                                        ("r_elbow_flex_joint" 0.0)
;;                                        ("r_forearm_roll_joint" 0.0)
;;                                        ("r_wrist_flex_joint" 0.0)
;;                                        ("r_wrist_roll_joint" 1.5))))))
                                        ;(defun go-into-direction ()
                                        ;(change-tree)
                                        ;(time
                                        ; (compute-multiple-obj-pose 'tree-4)))




;;(defun find-obj-pose (obj-type obj-name)
;; "this is hard coded and give the object-pose back"
;;(setf obj-pose (cdr (assoc '?pose (cdar (force-ll (prolog `(and (bullet-world ?w)
;;					     (object-type ?w ,obj-name ,obj-type)
;;					     (object-pose ?w ,obj-name ?pose))))))))
;; obj-pose)

;;(defun spawn-robot ()
;;  (force-ll 
;;   (prolog `(and (bullet-world ?w)
;;		 (assert (object ?w mesh quad-1 ((0 1 2)(0 0 0 1))
;;				 :mesh btr::quad1 :mass 0.2 :color (1 0 0)))))))


;;(defun pointing-into-direction (object-name)
;;  (cram-language-implementation:top-level
;;    (plan-lib:with-designators
;;        ((des-for-loc (desig-props:location `((right-of tree-1) 
;;                                              (far-from tree-1) 
;;                                              (for ,object-name)))))
;;      (crs:prolog `(assign-robot-pos-to ,des-for-loc ,object-name)))))


;; This Fuction returns a designator 
;;(cpl::def-cram-function find-object-in-world (object-type object-name)
"returns an object designator"
;; (cram-plan-library:with-designators
;;     ((in-world   (desig-props:location `((on ground)
;; (name ,object-name))))
;;      (the-object (desig-props:object `((type ,object-type)
;; 				       (name ,object-name)))))
;;   (reference in-world)
;;   (plan-lib:perceive-object 'cram-plan-library:a the-object)))


;; (defun go-robot-to-direction ()
;;   (go-to-object))

;; (cpl::def-top-level-cram-function go-to-object ()
;;  (cram-projection:with-projection-environment
;;    projection-process-modules::pr2-bullet-projection-environment
;;       (let ((tree (go-robot-from-origin)))
;; 	(go-robot-from-origin-near-tree tree))))



;; (cpl::def-cram-function go-robot-from-origin-near-tree (object-type tree-obj)
;;  (sb-ext:gc :full t)
;;  (let ((robot (find-object-in-world 'btr:household_objects "tree4")))

;;    (sb-ext:gc :full t)
;;    (go-robot-near-target robot tree-obj)
;;    ;;(ecase object-type
;;    ;;(btr::tree4 '(desig-props:left-of)))
;;    (sb-ext:gc :full t)))

;; (cpl::def-cram-fuction go-robot-near-target (object-to-target tree-object
;; 								  sherpa-spatial)
;;   (cram-plan-library:with-designators
;;       ((go-there-loc (location `(,@(loop for property in sherpa-spatial
;; 					collecting `(,property ,tree-object))
;; 				   (near ,tree-object)(for ,object-to-target)
;; 				   (on ???)))))
;;     (plan-knowledge:achieve `(plan-knowledge:loc ,object-to-target ,go-there-loc)))



;; (cpl-impl:def-top-level-cram-function go-to-object ()
;;  (cram-projection:with-projection-environment
;;    projection-process-modules::pr2-bullet-projection-environment
;;       (let ((tree (go-robot-from-origin)))
;; 	(

;; (cpl-impl:def-cram-function go-robot-from-origin-near-object (robot-type tree-obj)
;;   (sb-ext:gc :full t)
;;   (let ((robot (find-object-on-counter object-type "CounterTop205")))
;;     (sb-ext:gc :full t)
;;     (put-object-near-plate obj plate-obj
;;                            (ecase object-type
;;                              (btr::fork '(desig-props:left-of))
;;                              (btr::knife '(desig-props:right-of))
;;                              (btr::mug '(desig-props:right-of desig-props:behind))))
;;     (sb-ext:gc :full t)))



;; ;; (cpl-impl:def-cram-function find-object-in-world (object-name)
;; ;; "Returns an designator."
;; ;;  (cram-plan-library:with-designators 
;; ;;      ((loc-of-object (desig-props:location `((desig-props:name object-name)
;; ;; 					       (desig-props:type object-type))))
;; ;;       (the-object)
;; ;;    (reference loc-of-object)
;; ;;    (plan-lib:perceive-object 'cram-plan-library:a the-object)))) 
;; ;;        ;                                        (name ,object-name

;; ;;       ;(reference loc)

;; (def-fact-group build-test-world()
;;   (<- (assign-robot-pos-to ?desig ?obj-name) 
;;     (btr::bound ?obj-name)
;;     (btr::bound ?desig)
;;     (btr:bullet-world ?world)
;;     (btr::desig-solutions ?desig ?solutions)
;;     (btr::take 6 ?solutions ?6-solutions)
;;     (btr::member ?solution ?6-solutions)
;;     (assert (btr::object-pose-on ?world ?obj-name ?solution))))

;;;;;;;;;;;;;;;;;;;;;;;;;START WITH INCOMPLETE CODE;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;desig for a direction
;;(defun make-location-desig (direction)
;;(make-designator 'desig-props:location '((desig-props:go-to ,direction)
;;                                         (desig-props:close-to avalanche1))))

;define fuction to go in a direction
;;(defun command-robot (direction)
;;(try-solving direction))

;define try-solving function
;;(defun try-solving (direction)
  

;;;;;;;;;;;;;;;;;;;;;;;;START OF DESIG;;;;;;;;;;;;;;;;;;;;;;;;;

;(defun make-loc-desig (obj-desig)
 ; (make-designator `location `((desig-props:right-of object-desig))))
 
;(defun make-obj-desig (obj-name)
 ; (make-designator `object `((desig-props:name obj-name))))

;(defun go-to-direction (loc-desig obj-desig)
 ;(with-designators
  ;   ((desig-for-loc (desig:location `((desig-props:right-of ,obj-desig)))))
  ; (crs:prolog `(assign-direction ,desig-for-loc))))


;; (def-fact-group build-test-world ()
;; (<- (assign-direction ?loc-desig ?obj-name)
;;   (btr:once 
;;    (btr:bound ?loc-desig)
;;    (btr:bullet-world ?w)
;;    (btr:desig-solutions ?loc-desig ?solutions),
;;    (btr:take 6 ?solutions ?6-solutions)
;;    (btr:member ?solution ?6-solutions)
;;    (assert (btr:object-pose-on ?w ?obj-name ?solution)))))

;;    ;  (assert (btr:joint-state ?w ?robot (;;PROLOG METHOD WITH AN ACTION-DESIG)))))))

;; (defun reset-arms-parking ()
;;   (crs:prolog `(and 
;;                 (btr:robot-arms-parking-joint-states ?joint-states)
;;                        (assert (btr:joint-state ?w ?robot ?joint-states)))))



   ;; (defun start-projection ()
;;  ; (right-arm-joint)
;; )

;; (defun make-joint-desig (joint-id)
;;     (make-designator 'object `((desig-props:joint-name ,joint-id))))


;(cpl-impl:def-top-level-cram-function right-arm-joint ()
 ; (cram-projection:with-projection-environment
 ;     projection-process-modules::pr2-bullet-projection-environment
;))

;(cpl-impl:def-cram-fuction fly-close-to-the-cylinder (object-type cylinder-obj)
 ;(format t "Fly to cylinder~%")
;(defun make-trajectory-desig ()
; (cram-designators:make-designator 'action `((joints (list ("" "" "" "" "" "" "" ""))) (values (list; (0.0 0.0 0.0 0.0 0.0 0.0)))))
;)

;(defun make-joint-desig (joint-name)
;  (cram-designators:make-designator 'object `((joint joint-name)))
;)


;;;
;;; STARTING PROLOG
;;;


;;;
;;; EXECUTING A TRAJECTORY OF THE PR2 IN THE GAZEBO SIMULATION
;;;


;;(defun execute-right-arm-trajectory (trajec)
;;  (roslisp:ros-info (sherpa-spatial-relations) "Execute-right")
;;  (let* ((act-cli (actionlib:make-action-client
;;                   "/r_arm_controller/joint_trajectory_action"
;;                   "pr2_controllers_msgs/JointTrajectoryAction"))
;;         (act-goal (actionlib:make-action-goal 
;;                       act-cli
;;                     :trajectory (pr2-manip-pm::remove-trajectory-joints
;;                                  #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint") 
;;                                  trajec :invert t))))
;;    (actionlib:wait-for-server act-cli)
 ;;   (actionlib:call-goal act-cli act-goal))
 ;; (roslisp:ros-info (sherpa-spatial-relations) "Finished"))

;;(defun default-position-to-trajectory ()
;;  (roslisp:ros-info (sherpa-spatial-relations) "Default Position to Trajectory")
 ;; (roslisp:make-msg
 ;;  "trajectory_msgs/JointTrajectory"
 ;;  (stamp header)
 ;;  (roslisp:ros-time)
 ;;  joint_names #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")
 ;;  points (vector
 ;;          (roslisp:make-message
 ;;           "trajectory_msgs/JointTrajectoryPoint"
 ;;           positions #(0.0 0.0 0.0 0.0 0.0 0.0 0.0);(vector position)
 ;;           velocities #(0)
;;            accelerations #(0)
;;            time_from_start 2.0))))

;;(defun position-to-trajectory ()
;;  (roslisp:ros-info (sherpa-spatial-relations) "Position to Trajectory")
;;  (roslisp:make-msg
 ;;  "trajectory_msgs/JointTrajectory"
;;   (stamp header)
;;   (roslisp:ros-time)
;;   joint_names #("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")
;;   points (vector
;;           (roslisp:make-message
;;            "trajectory_msgs/JointTrajectoryPoint"
;;            positions #(0.0 -0.5 0.0 0.0 0.0 0.0 1.5);(vector position)
;;            velocities #(0)
;;            accelerations #(0)
;;            time_from_start 2.0))))

;;(defun right-arm-trajectory () 
;;  (roslisp:ros-info (sherpa-spatial-relations) "Moving right arm")
;;  (execute-right-arm-trajectory (default-position-to-trajectory))
;;  (start-myros)
;;  (execute-right-arm-trajectory (position-to-trajectory)))

