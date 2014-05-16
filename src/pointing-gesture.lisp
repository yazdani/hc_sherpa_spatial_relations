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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;BULLET;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun pointing-into-bullet ()
 (roslisp:ros-info (sherpa-spatial-relations) "POINTING INTO BULLETWORLD")
 (crs:prolog
 `(assert (btr:joint-state ?w genius (( "right_shoulder_joint_x" 0.06) ;;0.1
                                     ( "right_shoulder_joint_y" -0.25)  ;;0.0 0.40
                                     ( "right_shoulder_joint_z" 1.4)  ;;0.6 0.500
                                     ( "left_upper_arm_joint_x" 0.1)
                                     ( "left_upper_arm_joint_y" 3.0)
                                     ( "left_upper_arm_joint_z" -0.5))))))

(defun get-joint-value-bullet (joint-name)
  (cdr 
   (assoc `?value
          (lazy-car
           (prolog 
            `(joint-state ?w genius ,joint-name ?value))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;GAZEBO;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun init-joints ()
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/genius/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-cb))
  (init-human))

(defun get-joint-value (str-name)
  (let* ((joint-states (sherpa-joint-states))
         (joint-state
           (nth (position str-name joint-states
                          :test (lambda (str-name state)
                                  (equal str-name (car state))))
              joint-states)))
    (cdr joint-state)))

(defun joint-states-cb (msg)
  (roslisp:with-fields (name position) msg
    (setf
     *joint-states*
     (loop for i from 0 below (length name)
           for n = (elt name i)
           for p = (elt position i)
           collect (cons n p)))))


(defun sherpa-joint-states ()
  *joint-states*)

(defun default-position-to-trajectory ()
  (roslisp:ros-info (sherpa-spatial-relations) "Position of human arm")
  (roslisp:make-msg "trajectory_msgs/JointTrajectory" 
                    (stamp header)
                    (roslisp:ros-time)
                    joint_names #("right_shoulder_joint_x" "right_shoulder_joint_y" "right_shoulder_joint_z"
                                  )
                    points (vector
                            (roslisp:make-message
                             "trajectory_msgs/JointTrajectoryPoint"
                             positions #(1.5 0.0 -0.1)
                             velocities #(0 0 0)
                             accelerations #(0)
                             time_from_start 2.0
                             ))))

    



(defun execute-right-arm-trajectory (trajec)
  (roslisp:ros-info (sherpa-spatial-relations) "Execute-right")
  (let* ((act-cli (actionlib:make-action-client
                   "genius/right_shoulder_joint_controller/follow_joint_trajectory"
                   "control_msgs/FollowJointTrajectoryAction"))
         (act-goal (actionlib:make-action-goal
                      act-cli
                     :trajectory 
                     (remove-trajectory-joints
                     #("right_shoulder_joint_x" "right_shoulder_joint_y" "right_shoulder_joint_z") 
                                  trajec :invert t))))

         (actionlib:wait-for-server act-cli)

         (actionlib:call-goal act-cli act-goal)
 
)
    (roslisp:ros-info (sherpa-spatial-relations) "Finished"))

(defun pointing-into-gazebo ()
 (roslisp:ros-info (sherpa-spatial-relations) "POINTING INTO GAZEBO")
 ; (init-human)
  (execute-right-arm-trajectory (default-position-to-trajectory))
  (start-myros))

(defun seq-member (item sequence)
  (some (lambda (s)
          (equal item s))
        sequence))

(defun remove-trajectory-joints (joints trajectory &key invert)
  "Removes (or keeps) only the joints that are specified in
  `joints'. If `invert' is NIL, the named joints are removed,
  otherwise, they are kept."
  (roslisp:with-fields ((stamp (stamp header))
                        (joint-names joint_names)
                        (points points))
      trajectory
    (if (not invert)
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (remove-if (lambda (name)
                                  (seq-member name joints))
                                joint-names)
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions
                                             velocities
                                             accelerations
                                             time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               unless (seq-member n joints)
                                                 collecting p))
                          velocities (map 'vector #'identity
                                          (loop for n across joint-names
                                                for p across velocities
                                                unless (seq-member n joints)
                                                  collecting p))
                          accelerations (map 'vector #'identity
                                             (loop for n across joint-names
                                                   for p across accelerations
                                                   unless (seq-member n joints)
                                                     collecting p))
                          time_from_start time_from_start)))
                     points))
        (roslisp:make-message
         "trajectory_msgs/JointTrajectory"
         (stamp header) stamp
         joint_names (map 'vector #'identity
                          (loop for n across joint-names
                                when (seq-member n joints)
                                  collecting n))
         points (map 'vector
                     (lambda (point)
                       (roslisp:with-fields (positions time_from_start)
                           point
                         (roslisp:make-message
                          "trajectory_msgs/JointTrajectoryPoint"
                          positions (map 'vector #'identity
                                         (loop for n across joint-names
                                               for p across positions
                                               when (seq-member n joints)
                                                 collecting p))
                          time_from_start time_from_start)))
                     points)))))


;;;;;;;;;;;;;;;;;;;INIT HUMAN BONES;;;;;;;;;;;;;;;;;;;
(defparameter *bone-hash* (make-hash-table :test 'equal))
(defparameter *bone-list* (list
			    "pelvis_joint"
			    "l5_joint"
			    "l3_joint"
			    "t12_joint"
			    "t8_joint"
			    "neck_joint"
			    "head_joint"
			    "right_upper_arm_joint"
			    "right_lower_arm_joint"
			    "right_hand_joint"
			    "right_shoulder_joint"
			    "right_upper_leg_joint"
			    "right_lower_leg_joint"
			    "right_foot_joint"
			    "right_toe_joint"
			    "left_upper_arm_joint"
			    "left_lower_arm_joint"
			    "left_hand_joint"
			    "left_shoulder_joint"
			    "left_upper_leg_joint"
			    "left_lower_leg_joint"
			    "left_foot_joint"
			    "left_toe_joint"))


(defun change-bone-state (name vector)
  " This function is used to change the state of a single bone"
  (let 
      ((pub  (gethash name *bone-hash*)) 
       (msg  (roslisp:make-msg "trajectory_msgs/JointTrajectory" 
                                               joint_names (vector (format nil "~a_x" name) (format nil "~a_y" name) (format nil "~a_z" name))
                                                 points (vector
           (roslisp:make-message
            "trajectory_msgs/JointTrajectoryPoint"
            positions vector
            velocities #(0 0 0)
            time_from_start 2.0
            )))))
    (roslisp:publish pub msg)))

(defun add-bone (name)
	"Used to add a bone to the hash map of available bones and creates a connection to the subscribed controller"
           (setf 
		(gethash name *bone-hash*) 
		(roslisp:advertise 
			(format nil "~a_controller/command" name) 
			"trajectory_msgs/JointTrajectory")))

(defun init-human ()
  "used to initialize the human controller for usage. creates advertisers for each joint and resets init position
"
    (roslisp:start-ros-node "human_teleop")
    (dolist (b *bone-list*) 
      (print 
       (format t "adding bone ~a" b)) 
      (add-bone b) 
      (change-bone-state b #(0 0 0)))
    (print "all bones initialized"))

;; (defun init-joint ()
;;  (setf *joint-states-subscriber*
;;         (roslisp:subscribe "trajectory_msgs/JointTrajectory"
;;                           "trajectory_msgs/JointTrajectoryPoint"
;;                            #'joint-states-cb)))
 

;; (defun joint-states-cb (msg)
;;   (roslisp:with-fields (name position) msg
;;     (setf
;;      *joint-states*
;;      (loop for i from 0 below (length name)
;;            for n = (elt name i)
;;            for p = (elt position i)
;;            collect (cons n p)))))

;; ;; (defun joint-states ()
;; ;;   *joint-states*
;; ;;   (format t "*joint-states* ~a~%" *joint-states*)))

;; (defun get-joint-value (name)
;;   (let* ((joint-states (joint-states))
;;          (joint-state
;;            (nth (position name joint-states
;;                           :test (lambda (name state)
;;                                   (equal name (car state))))
;;                 joint-states)))
;;     (cdr joint-state)))