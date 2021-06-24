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

// #define ARMA_DONT_USE_WRAPPER
#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include "armadillo"
#include <initializer_list>

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace arma;

//map <string, vector<double> > region_mapping;

extern "C" ExternalSolver *create_object()
{
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver)
{
  delete externalSolver;
}

VisitSolver::VisitSolver()
{
}

VisitSolver::~VisitSolver()
{
}

void VisitSolver::loadSolver(string *parameters, int n)
{
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[] = {"dummy"};
  char const *y[] = {"act-cost", "triggered"};
  parseParameters(Paramers);
  affected = list<string>(x, x + 1);
  dependencies = list<string>(y, y + 2);

  string waypoint_file = "/root/ai4ro2/visits_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/root/ai4ro2/visits_domain/landmark.txt";
  parseLandmark(landmark_file);

  arma::arma_rng::set_seed_random();
}

map<string, double> VisitSolver::callExternalSolver(map<string, double> initialState, bool isHeuristic)
{

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;

  map<string, double> trigger;

  for (; iSIt != isEnd; ++iSIt)
  {
    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0, 1);
    function.erase(function.length() - 1, function.length());
    int n = function.find(" ");

    if (n != -1)
    {
      string arg = function;
      string tmp = function.substr(n + 1);

      function.erase(n, function.length() - 1);
      arg.erase(0, n + 1);
      if (function == "triggered")
      {
        trigger[arg] = value > 0 ? 1 : 0;
        if (value > 0)
        {
          int curr1 = tmp.find(" ");
          string from = tmp.substr(0, curr1); // from and to are regions, need to extract wps (poses)
          int curr2 = tmp.find(" ", curr1+1);
          string to = tmp.substr(curr1+1, curr2);

          p_from_ = from;
          p_to_ = to;

          /*  LOCALIZE  */
          localize(from, to);
        }
      }
    }
    else
    {
      if (function == "dummy")
      {
        dummy = value;
      }
      else if (function == "act-cost")
      {
        act_cost = value;
      }
    }
  }

  double results = calculateExtern(dummy, act_cost);
  if (ExternalSolver::verbose)
  {
    std::cout << "(dummy) " << results << endl;
  }

  toReturn["(dummy)"] = results;

  return toReturn;
}

list<string> VisitSolver::getParameters()
{
  return affected;
}

list<string> VisitSolver::getDependencies()
{
  return dependencies;
}

void VisitSolver::parseParameters(string parameters)
{
  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open())
  {
    while (getline(parametersFile, line))
    {
      boost::trim(line);
      curr = line.find(" ");
      string region_name = line.substr(0, curr).c_str();
      curr = curr + 1;
      while (true)
      {
        next = line.find(" ", curr);
        region_mapping[region_name].push_back(line.substr(curr, next - curr).c_str());
        if (next == -1)
          break;
        curr = next + 1;
      }
    }
  }
}

double VisitSolver::calculateExtern(double dummy, double total_cost)
{
  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  //double cost = 2; //random1;
  double cost = dist + trace_weight *trace; /*  The trace is here given an extra weight  */
  if (ExternalSolver::verbose)
  {
    cout << endl << "(" << p_from_ << ")->(" << p_to_ << "): " << "dist_" << dist << " trace_" << trace << " cost_" << cost << endl;
  }
  return cost;
}

void VisitSolver::parseWaypoint(string waypoint_file)
{

  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(waypoint_file);
  if (parametersFile.is_open())
  {
    while (getline(parametersFile, line))
    {
      boost::trim(line);
      curr = line.find("[");
      string waypoint_name = line.substr(0, curr).c_str();

      curr = curr + 1;
      next = line.find(",", curr);

      pose1 = (double)atof(line.substr(curr, next - curr).c_str());
      curr = next + 1;
      next = line.find(",", curr);

      pose2 = (double)atof(line.substr(curr, next - curr).c_str());
      curr = next + 1;
      next = line.find("]", curr);

      pose3 = (double)atof(line.substr(curr, next - curr).c_str());

      waypoint[waypoint_name] = vector<double>{pose1, pose2, pose3};
    }
  }
}

void VisitSolver::parseLandmark(string landmark_file)
{
  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(landmark_file);
  if (parametersFile.is_open())
  {
    while (getline(parametersFile, line))
    {
      curr = line.find("[");
      string landmark_name = line.substr(0, curr).c_str();

      curr = curr + 1;
      next = line.find(",", curr);

      pose1 = (double)atof(line.substr(curr, next - curr).c_str());
      curr = next + 1;
      next = line.find(",", curr);

      pose2 = (double)atof(line.substr(curr, next - curr).c_str());
      curr = next + 1;
      next = line.find("]", curr);

      pose3 = (double)atof(line.substr(curr, next - curr).c_str());

      landmark[landmark_name] = vector<double>{pose1, pose2, pose3};
    }
  }
}


double dist2(arma::vec start, arma::vec goal)
{
  return sqrt(pow(goal(0) - start(0), 2) + pow(goal(1) - start(1), 2));
}

double normAngle(double angle)
{
  angle = fmod(angle, 2 * arma::datum::pi);
  if (angle < 0) {
    angle += arma::datum::pi;
  }
  return angle;
}

void VisitSolver::localize( string from, string to){

  /*
  TODO - EKF:
  x retrieve position of the two waypoints associated to (from, to)
  x separate the path (from->to) into 'N' small steps of length \deltaD
  - foreach step:
  --  update position + added odometry synthetic noise
  --  forall beacons closer than threshold to current position:
  ---   update the estimated EKF with the position and orientation of the beacon (with added synth noise)
  - return the estimated cost as the dist + trace(covM)

  */
  string ws_, wg_;
  double tmp_dist = datum::inf;

  if (!region_mapping.count(from))  {throw invalid_argument(string("'from' region "+ from +" not found")); }
  if (!region_mapping.count(to))  {throw invalid_argument(string("'to' region "+ to +" not found")); }   
  for (string ws : region_mapping[from])
  {
    for (string wg : region_mapping[to])
    {
      if (!waypoint.count(ws))  {throw invalid_argument(string("'from' waypoint "+ ws +" not found")); }
      if (!waypoint.count(wg))  {throw invalid_argument(string("'to' waypoint "+ wg +" not found")); }   

      double tt = dist2(arma::conv_to<vec>::from(waypoint[ws]), arma::conv_to<vec>::from(waypoint[wg]));

      if (tmp_dist > tt)
      {
        tmp_dist = tt;
        ws_ = ws;
        wg_ = wg;
      }
    }
  }
  if (ExternalSolver::verbose)
  {
    std::cout << endl << "(ws) " << ws_ << "; (wg) " << wg_ << " : " << tmp_dist << endl;
  }

  // dist = tmp_dist;

  // First version: split both position and orientation in equal number of steps
  // A very simple controller working only for holonomic robots
  // in the actual case, eg. for a (2,0) robot, it would first need to orient
  // toward the goal position, reach it, then orient like the goal orientation

  /*  To find the number of steps we divide the total length of the path,
      position only, by the robot translational speed (to get an estimate of
      the time taken) and then multiply that by the sample frequency for the
      synthetic odometry encoders.
    */
  const uint N_STEPS = ceil(tmp_dist/robot_vel * odom_rate);
  
  arma::vec step_dim(3), X_k(3), syn_noise(3, arma::fill::zeros);
  arma::mat P_k(3,3, arma::fill::eye), Q_a(3,3, arma::fill::eye); // Q_a: variance 1 of the "random" noise
  /*  Initiate the current position (with covariance matrix) */
  X_k = arma::conv_to<arma::vec>::from(waypoint[ws_]);
  P_k = P_k * pow(init_noise, 2);
  /*  Covariance matrix of the noise introduced by the odometry  */
  Q_a = pow(odom_noise_mod,2) * Q_a;
  

  mat A(3, 3, arma::fill::eye);
  waypoint[ws_][2] = normAngle(waypoint[ws_][2]);
  waypoint[wg_][2] = normAngle(waypoint[wg_][2]);

      for (uint i = 0; i < waypoint[ws_].size(); i++)
  {
    step_dim(i) = (waypoint[wg_][i] - waypoint[ws_][i])/N_STEPS;
  }
  /* */
  arma::mat Beac(3, landmark.size());
  int i = 0;
  for (auto beacon : landmark)
  {
    Beac.col(i++) = arma::conv_to<arma::vec>::from(beacon.second);
  }

  bool flag = false;
  for (uint i = 0; i < N_STEPS; i++)
  {
    syn_noise = arma::randn<vec>(3) * odom_noise_mod;
    X_k = X_k + step_dim + syn_noise;

    P_k = A * P_k * A.t() + Q_a;

    Beac.each_col( [&X_k, &P_k, this, &flag](vec& y)
      {
        if (dist2(X_k, y) < beacon_dist_th)
        {

          if (!flag && ExternalSolver::verbose){cout << endl << "(" << p_from_ << ")->(" << p_to_ << "): beacon detected" << endl;
                     flag = true; }
          beaconDetectedEKF(X_k, P_k, y);
        }
      }
    );
  }

  dist = tmp_dist;
  trace = arma::trace(P_k);
}

void VisitSolver::beaconDetectedEKF(arma::vec& X_k, arma::mat& P_k, arma::vec y)
{
  // pick y, make a 2x1 column vector storing dist and angle
  // make a noisy version of it
  // compute C matrix (dg/dX)
  // compute K
  // update X
  // update P

  arma::vec g(2);
  double dist_g_m = dist2(X_k, y);
  g(0) = dist_g_m; //< distance of the beacon from the robot
  g(1) = std::atan2(y(1)-X_k(1), y(0)-X_k(0)) - X_k(2); //< orientation of the beacon wrt the robot


  /*
    C = dg/dX = 
    | 2(x_g - x_m)  2(y_G - y_m)    0  |
    | (y_g - y_m)/D (x_g - x_m)/D   -1 |

    D  = dist(Beacon, Robot)
  */
  arma::mat C_k(2, 3);
  C_k(0, 0) = 2*(y(0) - X_k(0));
  C_k(0, 1) = 2*(y(1) - X_k(1));
  C_k(0, 2) = 0;
  C_k(1, 0) = (y(1) - X_k(1))/dist_g_m;
  C_k(1, 1) = (y(0) - X_k(0))/dist_g_m;
  C_k(1, 2) = -1;

  arma::vec Y_meas(g);
  Y_meas = Y_meas + arma::randn<vec>(2) * detection_noise_mod;
  arma::mat Q_gamma(2,2, arma::fill::eye);
  Q_gamma = std::pow(detection_noise_mod,2) * Q_gamma;

  // K: Innovation "trust" coefficient
  arma::mat K_k(3, 2);
  K_k = P_k * C_k.t() * arma::inv(C_k * P_k * C_k.t() + Q_gamma);
  // Update X_k -> X_{k+1}
  //  *** X_k = X_k + K_k*(Y_meas - g); ***
  // Update P_{k+1/k} -> P_{k+1/k+1}
  P_k = (arma::eye(3,3) - K_k*C_k)*P_k;

}