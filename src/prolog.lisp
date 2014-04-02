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

(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)

(defclass gaussian-generator () ())
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)

(def-fact-group spatial-relations-costmap (desig-costmap)
  
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (go-to ?location))
    (costmap ?costmap)
    (format "hallo function~%")
    (costmap-add-function sherpa-spatial-generator
                          (make-spatial-relation-cost-function ?location :X > 0.99)
                          ?costmap)
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
    (instance-of range-generator ?range-generator-id-1)
    (costmap-add-function ?range-generator-id-1
                          (make-range-cost-function ?location 7.5)
                          ?costmap)
    (costmap-add-height-generator (make-constant-height-function
                                   3.0)
                                  ?costmap)))
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

 