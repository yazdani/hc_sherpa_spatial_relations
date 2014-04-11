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

;;(register-location-validation-function 6 collision-obj-pose-validator)


;; (defun collision-obj-pose-validator (desig pose)
;;   (format t "hi ~%")
;;   (when (typep pose 'cl-transforms:pose)
;;     (let* ((get-value (desig-prop-value desig 'far-from))
;;         ;   (get-pose (get-object-pose get-value))
;;            (get-type (cdr (assoc '?type (car (force-ll
;;                                          (prolog `(and
;;                                                    (bullet-world ?w)
;;                                                    (object-type ?w ,get-value ?type))))))))
;;            (bound-x (/(x-size-object get-value)2))
;;            (bound-y (/(y-size-object get-value)2))
;;            (bound-z (/(z-size-object get-value)2))
;;            (pos-x (cl-transforms:x (cl-transforms:origin pose)))
;;            (pos-y (cl-transforms:y (cl-transforms:origin pose)))
;;            (pos-z (cl-transforms:z (cl-transforms:origin pose))))
;;       (format t "pos-x: ~a~%pos-y: ~a~%pos-z ~a~%bound-x:~a~%bound-y:~a~%bound-z: ~a~%" pos-x pos-y pos-z bound-x bound-y bound-z)
;;       (if (and get-value
;;                get-type)
;;           (if (and               
;;                (and  (<= pos-z bound-z)
;;                      (<= pos-x bound-x))
;;                (<= pos-y bound-y))
;;               :accept
;;               :reject)
;;           :accept))))
