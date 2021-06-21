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

  
  //arma_rng::set_seed_random();
  //startEKF();
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
      string tmp = function.substr(n + 1, 5);

      function.erase(n, function.length() - 1);
      arg.erase(0, n + 1);
      if (function == "triggered")
      {
        trigger[arg] = value > 0 ? 1 : 0;
        if (value > 0)
        {

          string from = tmp.substr(0, 2); // from and to are regions, need to extract wps (poses)
          string to = tmp.substr(3, 2);

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
      //else if(function=="dummy1"){
      //duy = value;
      ////cout << parameter << " " << value << endl;
      //}
    }
  }

  double results = calculateExtern(dummy, act_cost);
  if (ExternalSolver::verbose)
  {
    cout << "(dummy) " << results << endl;
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
  double cost = dist;
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

void VisitSolver::localize( string from, string to){

/*
  TODO - EKF:
  x retrieve position of the two waypoints associated to (from, to)
  - separate the path (from->to) into 'N' small steps of length \deltaD
  - foreach step:
  --  update position + added odometry synthetic noise
  --  forall beacons closer than threshold to current position:
  ---   update the estimated EKF with the position and orientation of the beacon (with added synth noise)
  - return the estimated cost as the dist + trace(covM)


  for (double ii : start){cout << ii;}

*/
  string ws_="empty", wg_="empty";
  double tmp_dist = datum::inf;

  if (!region_mapping.count(from))  {throw invalid_argument(string("'from' region "+ from +" not found")); }
  if (!region_mapping.count(to))  {throw invalid_argument(string("'to' region "+ to +" not found")); }   
  for (string ws : region_mapping[from])
  {
    for (string wg : region_mapping[to])
    {
      if (!waypoint.count(ws))  {throw invalid_argument(string("'from' waypoint "+ ws +" not found")); }
      if (!waypoint.count(wg))  {throw invalid_argument(string("'to' waypoint "+ wg +" not found")); }   

      double tt = sqrt(pow(waypoint[wg][0] - waypoint[ws][0], 2) + pow(waypoint[wg][1] - waypoint[ws][1], 2));

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
    cout << endl << "(ws) " << ws_ << "; (wg) " << wg_ << " : " << tmp_dist << endl;
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
  // vector<double> step_dim(3, 0), current_pose(waypoint[ws_]);
  
  arma::vec step_dim(3), current_pose(3), syn_noise(3, fill::zeros);
  arma::mat P_k(3,3, fill::eye);
  P_k = P_k * init_noise;

  mat A(3, 3, fill::eye);
  
  for (uint i = 0; i <  waypoint[ws_].size(); i++)
  {
    step_dim(i) = (waypoint[wg_][i] - waypoint[ws_][i])/N_STEPS;
  }

  /* */

  for (uint i = 0; i < N_STEPS; i++)
  {
      //syn_noise.randn();
      syn_noise = syn_noise * odom_noise_mod;
      current_pose = current_pose + step_dim + syn_noise;

      P_k = A * P_k * A.t();
  }

  /**/
  dist = tmp_dist;
  trace = arma::trace(P_k);
}
