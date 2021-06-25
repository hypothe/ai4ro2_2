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


# ai4ro2_2
Repository storing the files for AI4RO2 Second Assignment

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