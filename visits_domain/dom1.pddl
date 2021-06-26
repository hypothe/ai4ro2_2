(define (domain localization)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types 	robot region 
)

(:predicates
		(robot_in ?v - robot ?r - region) (visited ?r - region )
		(connected ?r1 ?r2 - region)
)

(:functions 
		(act-cost) (triggered ?from ?to - region) (dummy)
)

(:action visit_region
		:parameters (?v - robot ?r - region)
		:precondition (robot_in ?v ?r)
	    :effect (visited ?r)
)

(:durative-action localize
		:parameters (?v - robot ?from ?to - region)
		:duration (= ?duration 100)
		:condition (and (at start (robot_in ?v ?from))
						(over all (connected ?from ?to)) ;; comment for a fully connected graph
					) 
		:effect (and
					(at start (not (robot_in ?v ?from))) (at start (assign (dummy) 0))
					(at start (increase (triggered ?from ?to) 1))
					(at end (robot_in ?v ?to)) (at end (assign (triggered ?from ?to) 0))	
					(at end (increase (act-cost) (dummy)))
				)
)

)

