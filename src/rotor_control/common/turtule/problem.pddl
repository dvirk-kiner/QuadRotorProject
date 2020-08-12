(define (problem task)
(:domain turtlebot_demo)
(:objects
    wp0 wp1 wp2 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)

    (connected wp0 wp1)
    (connected wp0 wp2)
    (connected wp1 wp0)
    (connected wp1 wp2)
    (connected wp2 wp0)
    (connected wp2 wp1)


    (= (distance wp0 wp1) 2)
    (= (distance wp0 wp2) 3.93856)
    (= (distance wp1 wp0) 2)
    (= (distance wp1 wp2) 2)
    (= (distance wp2 wp0) 3.93856)
    (= (distance wp2 wp1) 2)

)
(:goal (and
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (visited wp5)
    (visited wp6)
))
)
