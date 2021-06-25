- overall description of what the systems does
- changes in the domain and problem files
- changes in the external solver/semantic attachment/C++
- how to use
- maybe documemtn code(?)

# AI4RO2 - Second Assignment

## Task Motion Planning

The content of this repository is an implementation of the system required by the Second Assignment of the **Artificial Intelligence for Robotics II** at [University of Genoa, MSc in Robotics Engineering](https://courses.unige.it/10635). Please note that everything here presented is a variation of a base implementation produced by Dr. [Anthony Thomas](https://www.dibris.unige.it/thomas-antony), to which full credits are given.
The goal of the system is to implement a PPDL 2.1 domain dealing with the movement of a mobile robot in a constrained environment. The robot is able to move between predefined waypoints, grouped in regions, and, in this scenario, must visit a subset of them in pseudo-optimal way.
Each path between two regions is associated with a cost, a function of the Euclidean distance between their two closest waypoints and the trace of the Covariance matrix associated to the robot State ($x, y, \theta$); the robot state is, in fact, assumed affected by uncertainties due to odometry errors, which add up during the whole path between two waypoints. Beacons are present in the environment and provide an extrinsic reference, being measurable by the robot when passing close enough to one of them, obtaining both the squared distance and the relative heading with respect to the robot frame (assumed coinciding with the sensor providing such data). This data is used in an Extended Kalman Filter model, in order to update the robot state and the Covariance matrix associated to it.
The path planning is carried out by the PDDL 2.1 planner, whilst an external solver (*a semantic attachment*) computed the weight associated to each edge between regions.
More info on each step in the respective section later on.

---

## Task Planning

The planner for which the system is defined is [popf-tif](https://github.com/popftif/popf-tif) thanks to its ability to deal with numerical fluents, temporal planning and *semantic attachments*.
The original domain and problem files have been only lightly modified, with the introduction of a new action, `localize` moving the robot between two regions, which is responsible for calling the *semantic attachment*.
Such call is carried out thanks to a syntactic trick: every time the PDDL 2.1 keyword *increase* is encountered a call to the external solver is made, passing  to it the map of all the parameters interested by such increase, and receiving a map of all the values modified in return.
At the same time, the previously defined action `goto_region` has been changed into `visit_region`, since it now only sets a region as *visited* if the robot finds itself in it, without dealing directly with the localization and planning aspect. While its usefulness might be debatable in this toy example (since the `localize` action could set the region as visited by itself), it allows to expand the notion of "visiting a region" in future installments: as an example, think of a waiter robot case, where the robot might move between tables with different goal, either serving dishes, taking orders, or cleaning the tables. No changes to the `localize` action here presented would be necessary in order to achieve that.

Furthermore, the connectivity of the waypoint graph has been reduced, from full to implementation specific, thanks to the addition of the `(connected region1 region2)` predicate. This choice has been made both to reduce, albeit slightly, the search space dimension at each step, and to better demonstrate the effect of the external planner in the definition of the cost.

## Empirical values

- trace-weight: between 20-50

- odom_noise_mod: 0.01, very high, but allows to see the effect of the increase of P

## Running

```bash
    popf3-clp -x -n -t100 dom1.pddl prob1.pddl ../visits_module/build/libVisits.so region_poses
```

## Authors

- Marco Gabriele Fedozzi, 5083365@studenti.unige.it
- Dr. [Anthony Thomas](https://www.dibris.unige.it/thomas-antony) (original author)