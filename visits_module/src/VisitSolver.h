/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2015  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TESTSOLVER_H
#define TESTSOLVER_H

#include "ExternalSolver.h"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include "armadillo"

using namespace std;

class VisitSolver : public ExternalSolver
{
public:
    VisitSolver();
    ~VisitSolver();
    virtual void loadSolver(string *parameters, int n);
    virtual map<string, double> callExternalSolver(map<string, double> initialState, bool isHeuristic);
    virtual list<string> getParameters();
    virtual list<string> getDependencies();
    map<string, vector<double>> waypoint;
    map<string, vector<double>> landmark;

    void parseWaypoint(string waypoint_file);
    void parseLandmark(string landmark_file);

    map<string, vector<string>> region_mapping;
    vector<string> source, target;
    string starting_position;

    void parseParameters(string parameters);

private:
    list<string> affected;
    list<string> dependencies;
    double dist;
    double trace;
    const double trace_weight = 50.0; // 50
    const double robot_vel = 0.1; // m/s
    const double odom_rate = 20; // Hz
    const double odom_noise_mod = 0.025; // 0.05 noise injected in the odometry
    /*  Why 1cm?  It's very high since the robot moves around 5mm each odom step (0.1m/s/20Hz) 
        but generates a semi-realistic behavior.
    */
    const double detection_noise_mod = 0.05; // 0.05 noise injected in the beacon detection
    /*  As above, a very rough guesstimate  */

    const double init_noise = 0.14; // initial covariance value, sigma^2  = 0.02
    const double beacon_dist_th = 0.5; 

    std::string p_from_, p_to_;

    double calculateExtern(double external, double total_cost);
    vector<string> findParameters(string line, int &n);
    void localize( string from, string to);
    void beaconDetectedEKF(arma::vec& X_k, arma::mat& P_k, arma::vec y);
};

#endif // TESTSOLVER_H
