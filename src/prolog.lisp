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

(defmethod costmap-generator-name->score ((name (eql 'sherpa-spatial-generator))) 5)
(defmethod costmap-generator-name->score ((name (eql 'support-object))) 3)
(defmethod costmap-generator-name->score ((name (eql 'sherpa-angle-generator))) 4)

(defmethod costmap-generator-name->score ((name (eql 'visibility-distribution)))
  9)
(defmethod costmap-generator-name->score ((name (eql 'on-the-bounding-box)))
  6)

(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)

(defclass gaussian-generator () ())
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)

(def-fact-group robot-metadata (end-effector-link robot)

  (<- (robot genius)))

(def-fact-group spatial-relations-costmap (desig-costmap)

  (<- (desig-costmap ?designator ?costmap)
    (format "hi1 ~a~%" ?costmap)
    (desig-prop ?designator (go-to ?position))
;   (desig-prop ?designator (far-from ?coll-obj))
    ;;   (desig-prop ?designator (far-from ?obj))
;    (costmap ?costmap)
   (format "hi2~%")
 ;   (costmap-add-function sherpa-spatial-generator
  ;                        (make-spatial-relation-cost-function ?position :X > 0.99)
   ;                       ?costmap)
    ;; (debug-costmap ?costmap 0.5)
    ;; (debug-costmap ?costmap 0.4)
    ;; (costmap-add-orientation-generator (make-orientation-function 
    ;;                                     ?location)
    ;;                                    ?costmap)
    ;; (costmap-add-height-generator (make-constant-height-function
    ;;                                3.0) 
    ;;                               costmap)
    ;; (costmap-add-function sherpa-angle-generator
    ;;                       (make-costmap-with-angle-function ?location)
    ;;                       ?costmap)))
    ;; (costmap-add-height-generator (make-constant-height-function
    ;;                              3.0) 
    ;;                             ?costmap) 
    ;; (costmap-add-orientation-generator (add-costmap-function-object ?costmap
    ;;          )
    ;;                                    ?costmap)
   ; (instance-of range-generator ?range-generator-id-1)
   ; (costmap-add-function ?range-generator-id-1
    ;                      (make-range-cost-function ?position 7.5)
     ;                     ?costmap)
 ;   (format "costmap~a~%" ?costmap)

;    (costmap-add-height-generator (make-constant-height-function
 ;                                  3.0)
  ;                                ?costmap))
  
  ;; (setf  btr::*costmap-z* 0.6)
  ;; (setf  btr::*costmap-tilt* 
  ;;        (cl-transforms:axis-angle->quaternion
  ;;         (cl-transforms:make-3d-vector 0 1 0)
  ;;         -0.25))

      (desig-prop ?designator (far-from ?object))
      (bullet-world ?world)
      (once
       (or (cram-environment-representation:object-designator-name
	    ?object ?object-instance-name)
	   (== ?object ?object-instance-name)))
      (%object ?world ?object-instance-name ?object-instance)
      (costmap ?costmap)
      (costmap-add-function
       on-the-bounding-box (make-object-bounding-box-costmap-gen
			    ?object-instance) ?costmap)))
  ;; (<- (solutions-not-in-collision ?desig ?obj-to-check ?pose)
  ;;   (format "gointo solution~%")
  ;;   (bullet-world ?world)
  ;;   (with-copied-world ?world
  ;;     (format "hiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii: object-name ~a~%" ?obj-name)
  ;;     (extract-object-instance-name ?obj-to-check ?obj-name)
  ;;     (format "hii: object-name ~a~%" ?obj-name)
  ;;     (assert (object-pose ?world ?obj-name ?pose))
  ;;     (format "obj-pose: ~a ~%" (object-pose ?w ?obj-name ?pose)))))
  ;;     ;; (forall
  ;;     ;;  (contact ?world ?obj-name ?new-obj-name)
  ;;     ;;  (and
  ;;     ;;   (object-type ?world ?new-obj-name ?type)
  ;;     ;;   ?new-obj-name ?type ?link)
  ;;     ;;  (or
  ;;     ;;   ;;             (object-type ?world ?new-obj-name btr::semantic-map-object)
  ;;     ;;   (and (robot ?new-obj-name)
  ;;     ;;        (attached ?world ?new-obj-name ?_ ?obj-name)))))))

;; (def-fact-group desig-location-utils ()
;;  (<- (checking-desig-with-type ?desig ?name ?type)
;;    ()

;;  (<- (extract-object-instance-name ?name)
;;     (lisp-type ?name symbol))

;;   (<- (environment-object-type ?world ?obj-name ?obj-type)
;;     (bullet-world ?world)
;;     (format "checking if working ~a~%" ?world)
;;     (object ?world ?obj-name)
;;   (format "checking if working2 ~a~%" ?obj-name)
;;     (%object ?world ?obj-name ?obj-type)
;;   (format "checking if working3 ~a~%" ?obj-type)))
  ;  (lisp-type ?obj-type environment-object)
; (format "checking if working4 ~a~%" environment-object)
 ;   (get-slot-value ?obj-type types ?types)
 ;(format "checking if working5 ~a~a~%" types ?types)
;    (member ?obj-type ?types)))



    ;; (instance-of gaussian-generator ?gaussian-generator-id)
    ;; (costmap-add-function ?gaussian-generator-id
    ;;                       (make-location-cost-function ?location 1.5)
    ;;                       ?costmap)
    ;; (costmap-add-function sherpa-angle-generator
;;                       (make-costmap-with-angle ?location :Y <)
;;                        ?costmap)
;;     (format "hallo15~%")
;; ))
;; (bullet-world ?world)
;; (once
;;  (cram-environment-representation:object-designator-name
;;   ?object ?object-name)
;;  (and (lisp-type ?object symbol)
;;       (== ?object ?object-name)))
;; (costmap ?costmap)
;; (costmap-add-function
;;  sherpa-spatial-generator
;;  (make-object-costmap-generator ?object-instance)
;;  ?costmap))
;; (<- (desig-costmap ?designator ?costmap)
;;   (desig-prop ?designator (right-of ?location))
;;   (costmap ?costmap)
;;   (costmap-add-function 
;;    sherpa-spatial-generator
;;    (make-spatial-relation-cost-function ?location :Y <)
;;    ?costmap)
;;   (costmap-add-height-generator
;;    (make-constant-height-function 6.0)
;;    ?costmap)) 
;; (desig-prop ?designator (pointed-at ?direction))
;; (object-instance-name ?object ?obj-name)
;; (btr:bullet-world ?world)
;; (btr:object ?world ?obj-name))
;; (costmap ?costmap)
;; (costmap-add-function 
;;  sherpa-spatial-generator
;;  (make-spatial-relation-cost-function ?location :Y <)
;;  ?costmap)
;; (costmap-add-height-generator
;;  (make-constant-height-function 6.0)
;;  ?costmap)))

 