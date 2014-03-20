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

(defun square (x)
  (* x x))

;;
;;functions for calculating the links in the world
;;
(defun right-shoulder-to-right-upper-arm-length ()
  (let* ((lower_arm_x (get-joint-value "right_upper_arm_joint_x"))
         (lower_arm_y (get-joint-value "right_upper_arm_joint_y"))
         (lower_arm_z (get-joint-value "right_upper_arm_joint_z"))
         (r_shoulder_x (get-joint-value "right_shoulder_joint_x"))
         (r_shoulder_y (get-joint-value "right_shoulder_joint_y"))
         (r_shoulder_z (get-joint-value "right_shoulder_joint_z"))
         (x-value  (- lower_arm_x r_shoulder_x))
         (y-value  (- lower_arm_y r_shoulder_y))
         (z-value  (- lower_arm_z r_shoulder_z))
         (x-square (square x-value))
         (y-square (square y-value))
         (z-square (square z-value))
         (square-value (+ (+ x-square y-square) z-square)))
    (sqrt square-value)))

(defun right-upper-arm-to-right-lower-arm-length ()
  (let* ((lower_arm_x (get-joint-value "right_upper_arm_joint_x"))
         (lower_arm_y (get-joint-value "right_upper_arm_joint_y"))
         (lower_arm_z (get-joint-value "right_upper_arm_joint_z"))
         (r_shoulder_x (get-joint-value "right_lower_arm_joint_x"))
         (r_shoulder_y (get-joint-value "right_lower_arm_joint_y"))
         (r_shoulder_z (get-joint-value "right_lower_arm_joint_z"))
         (x-value  (- r_shoulder_x  lower_arm_x))
         (y-value  (- r_shoulder_y  lower_arm_y))
         (z-value  (- r_shoulder_z  lower_arm_z))
         (x-square (square x-value))
         (y-square (square y-value))
         (z-square (square z-value))
         (square-value (+ (+ x-square y-square) z-square)))
    (sqrt square-value)))

(defun right-shoulder-to-right-lower-arm-length ()
  (let* ((lower_arm_x (get-joint-value "right_lower_arm_joint_x"))
         (lower_arm_y (get-joint-value "right_lower_arm_joint_y"))
         (lower_arm_z (get-joint-value "right_lower_arm_joint_z"))
         (r_shoulder_x (get-joint-value "right_shoulder_joint_x"))
         (r_shoulder_y (get-joint-value "right_shoulder_joint_y"))
         (r_shoulder_z (get-joint-value "right_shoulder_joint_z"))
         (x-value  (- lower_arm_x r_shoulder_x))
         (y-value  (- lower_arm_y r_shoulder_y))
         (z-value  (- lower_arm_z r_shoulder_z))
         (x-square (square x-value))
         (y-square (square y-value))
         (z-square (square z-value))
         (square-value (+ (+ x-square y-square) z-square)))
    (sqrt square-value)
    ))

;;
;;function for extension the length of the right arm in the world
;;
(defun arm-linear-extension ()
  (+ 2 (right-upper-arm-to-right-lower-arm-length)))

(defun ik-calculate ())

















;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;BULLET;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun arm-length ()
  (let* ((x-shoulder (var-value '?state 
             (lazy-car 
              (prolog `(and (bullet-world ?w)
                            (btr:joint-state ?w genius
                                             "right_shoulder_joint_x" ?state))))))
        (y-shoulder (var-value '?state 
                            (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "right_shoulder_joint_y" ?state))))))
        (z-shoulder (var-value '?state 
                            (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "right_shoulder_joint_z" ?state))))))
        (x-arm (var-value '?state 
                               (lazy-car 
                                (prolog `(and (bullet-world ?w)
                                              (btr:joint-state ?w genius
                                                               "right_upper_arm_joint_x" ?state))))))
        (y-arm (var-value '?state 
                               (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "right_upper_arm_joint_y" ?state))))))
        (z-arm (var-value '?state 
                               (lazy-car 
                                (prolog `(and (bullet-world ?w)
                                              (btr:joint-state ?w genius
                                                               "right_upper_arm_joint_z" ?state))))))
        (x-value  (- x-shoulder x-arm))
        (y-value  (- y-shoulder y-arm))
        (z-value  (- z-shoulder z-arm))
         (x-square (square x-value))
         (y-square (square y-value))
         (z-square (square z-value))
         (square-value (+ (+ x-square y-square) z-square)))
                       
    (sqrt square-value)))

(defun body-length ()
  (let* ((x-shoulder (var-value '?state 
             (lazy-car 
              (prolog `(and (bullet-world ?w)
                            (btr:joint-state ?w genius
                                             "t8_joint_x" ?state))))))
        (y-shoulder (var-value '?state 
                            (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "t8_joint_y" ?state))))))
        (z-shoulder (var-value '?state 
                            (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "t8_joint_z" ?state))))))
        (x-arm (var-value '?state 
                               (lazy-car 
                                (prolog `(and (bullet-world ?w)
                                              (btr:joint-state ?w genius
                                                               "t12_joint_x" ?state))))))
        (y-arm (var-value '?state 
                               (lazy-car 
                             (prolog `(and (bullet-world ?w)
                                           (btr:joint-state ?w genius
                                                            "t12_joint_y" ?state))))))
        (z-arm (var-value '?state 
                               (lazy-car 
                                (prolog `(and (bullet-world ?w)
                                              (btr:joint-state ?w genius
                                                               "t12_joint_z" ?state))))))
        (x-value  (- x-arm x-shoulder))
        (y-value  (- y-shoulder y-arm))
        (z-value  (- z-shoulder z-arm))
         (x-square (square x-value))
         (y-square (square y-value))
         (z-square (square z-value))
         (square-value (+ (+ x-square y-square) z-square)))
                       
    (sqrt square-value)))

