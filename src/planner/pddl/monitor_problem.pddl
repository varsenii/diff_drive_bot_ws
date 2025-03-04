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
        (and (robot_at diff_drive_bot wp2))  ;; Goal position of the robot
    )
)
