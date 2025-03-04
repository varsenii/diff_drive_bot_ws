(define (domain monitoring)
    (:requirements :strips :typing :adl :fluents :durative-actions)

    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:types
        robot
        waypoint
    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

    ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:predicates
        (robot_at ?r - robot ?wp - waypoint)
        (connected ?wp1 ?wp2 - waypoint)
        (scanned ?wp - waypoint)
    );; end Predicates ;;;;;;;;;;;;;;;;;;;;

    ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
    (:functions
        ;; Define any necessary functions here, if needed
    );; end Functions ;;;;;;;;;;;;;;;;;;;;

    ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (:durative-action move
        :parameters (?r - robot ?wp1 ?wp2 - waypoint)
        :duration ( = ?duration 5)
        :condition (and 
            (at start (robot_at ?r ?wp1))
            (at start (connected ?wp1 ?wp2))
        ) 
        :effect (and 
            (at start (not (robot_at ?r ?wp1)))
            (at end (robot_at ?r ?wp2))
        )
    )

    (:durative-action scan
        :parameters (?r - robot ?wp - waypoint)
        :duration ( = ?duration 3)
        :condition (and 
            (at start (robot_at ?r ?wp))
        )
        :effect (and 
            (at end (scanned ?wp))
        )
    )
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;;

(define (problem robot_navigation)
    (:domain monitoring)  ;; Refers to the domain definition

    ;; Objects (Instances of Types)
    (:objects
        diff_drive_bot - robot
        wp1 wp2 - waypoint
    )

    ;; Initial state (Predicates)
    (:init
        (connected wp1 wp2)
        (robot_at diff_drive_bot wp1)  ;; Initial position of the robot
    )

    ;; Goal state
    (:goal
        (and 
            (robot_at diff_drive_bot wp2)   ;; Goal position of the robot
            (scanned diff_drive_bot wp1)    ;; Ensure scanning at wp1
            (scanned diff_drive_bot wp2)    ;; Ensure scanning at wp2
        )
    )
)
