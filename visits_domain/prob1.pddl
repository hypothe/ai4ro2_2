(define (problem prob1)
(:domain localization)
(:objects
     r12 r23 r34 r41 r0 r4 r3 r2 r1 - region
     
     R2D2 - robot
)
(:init
     (robot_in R2D2 r0)

     (= (act-cost) 0)
     (= (dummy) 0)

     ;; Uncomment these lines for 
     ;; setting up a not fully connected graph
     (connected r0 r1) (connected r1 r0) 
     (connected r0 r2) (connected r2 r0) 
     (connected r0 r3) (connected r3 r0) 
     (connected r0 r4) (connected r4 r0) 
     
     (connected r1 r12) (connected r12 r1)
     (connected r1 r41) (connected r41 r1)
     (connected r2 r12) (connected r12 r2)
     (connected r2 r23) (connected r23 r2)
     (connected r3 r23) (connected r23 r3)
     (connected r3 r34) (connected r34 r3)
     (connected r4 r34) (connected r34 r4)
     (connected r4 r41) (connected r41 r4)
     
     (connected r1 r2) (connected r2 r1)
     (connected r2 r3) (connected r3 r2)
     (connected r3 r4) (connected r4 r3)
     (connected r4 r1) (connected r1 r4)

     (connected r0 r12) (connected r12 r0)
     (connected r0 r41) (connected r41 r0)
     (connected r0 r23) (connected r23 r0)
     (connected r0 r34) (connected r34 r0)
)
(:goal 
     (and (visited r4)  (visited r3)
          (visited r2) (visited r1) 
          (>= (act-cost) 0)
     )
)
(:metric minimize (act-cost))
)