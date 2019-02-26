#include"DubinsSolver.hpp"
#include"Utility.hpp"
#include<cmath>
#include<limits>
#include<iostream>

using namespace std;

const DubinsSolver::curve_type_solution DubinsSolver::methods[6] = {&LSL, &RSR, &LSR, &RSL, &RLR, &LRL};
const int DubinsSolver::curvatures[6][3] = {{1,0,1}, {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}, {-1, 1, -1}, {1, -1, 1}};

// function that will compute the shortest path
int DubinsSolver::shortestPath(double x0, double y0, double th0, double xf, double yf, double thf, double k_max, DubinsCurve *results[6]){
  double dx = xf - x0, dy = yf - y0;

  /*SCALE PARAMETERS*/
  DubinsProblemScaledParameters sc_params;
  sc_params.k_max = std::atan2(dy, dx);
  sc_params.lambda = sqrt(dx*dx + dy*dy)/2;
  sc_params.th0 = Utility::mod2pi(th0 - sc_params.k_max);
  sc_params.thf = Utility::mod2pi(thf - sc_params.k_max);
  sc_params.k_max = k_max * sc_params.lambda;

  double length = std::numeric_limits<double>::max(), l_tmp;
  double s1, s2, s3;
  int best_solution = -1;
  for(unsigned int i = 0; i < 6; i++){
    if((methods[i])(sc_params, s1, s2, s3)){
      scaleFromStandard(sc_params.lambda, s1, s2, s3);
      results[i] = new DubinsCurve(x0, y0, th0, s1, s2, s3, k_max * curvatures[i][0], k_max * curvatures[i][1], k_max * curvatures[i][2]);
      l_tmp =  s1 + s2 + s3;
      if(l_tmp < length){
        best_solution = i;
        length = l_tmp;
      }
    } else {
      results[i] = NULL;
    }
  }
  return best_solution;
};

// function to free the six curves
void DubinsSolver::freeCurves(DubinsCurve *results[6]){
  for(unsigned int i = 0; i < 6; i++){
    if(results[i]){
      delete results[i];
      results[i] = NULL;
    }
  }
}

// function to scale the paramenters 
void DubinsSolver::scaleFromStandard(double lambda, double &s1, double &s2, double &s3){
  s1 = s1 * lambda;
  s2 = s2 * lambda;
  s3 = s3 * lambda;
}

// LeftStraightLeft DubinsArc
bool DubinsSolver::LSL(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(std::cos(sc_params.thf) - std::cos(sc_params.th0), 2 * sc_params.k_max + sin(sc_params.th0) - sin(sc_params.thf));
  sc_s1 = invK * Utility::mod2pi(temp1 - sc_params.th0);
  double temp2 = 2 + 4 * sc_params.k_max * sc_params.k_max - 2 * std::cos(sc_params.th0 - sc_params.thf) + 4 * sc_params.k_max * (sin(sc_params.th0) - sin(sc_params.thf));

  if(temp2 < 0)
    return false;

  sc_s2 = invK * std::sqrt(temp2);
  sc_s3 = invK * Utility::mod2pi(sc_params.thf - temp1);
  return true;
}

// RightStraightRight DubinsArc
bool DubinsSolver::RSR(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(std::cos(sc_params.th0) - std::cos(sc_params.thf), 2 * sc_params.k_max - sin(sc_params.th0) + sin(sc_params.thf));
  sc_s1 = invK * Utility::mod2pi(sc_params.th0 - temp1);
  double temp2 = 2 + 4 * sc_params.k_max * sc_params.k_max - 2 * std::cos(sc_params.th0 - sc_params.thf) - 4 * sc_params.k_max * (sin(sc_params.th0) - sin(sc_params.thf));
  if(temp2 < 0)
    return false;

  sc_s2 = invK * sqrt(temp2);
  sc_s3 = invK * Utility::mod2pi(temp1 - sc_params.thf);
  return true;
}

// LeftStraightRight DubinsArc
bool DubinsSolver::LSR(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(-std::cos(sc_params.th0) - std::cos(sc_params.thf), 2 * sc_params.k_max + sin(sc_params.th0) + sin(sc_params.thf));
  double temp3 = 4 * sc_params.k_max * sc_params.k_max - 2 + 2 * std::cos(sc_params.th0 - sc_params.thf) + 4 * sc_params.k_max * (sin(sc_params.th0) + sin(sc_params.thf));
  if(temp3 < 0)
    return false;

  sc_s2 = invK * sqrt(temp3);
  temp3 = -std::atan2(-2, sc_s2 * sc_params.k_max);
  sc_s1 = invK * Utility::mod2pi(temp1 + temp3 - sc_params.th0);
  sc_s3 = invK * Utility::mod2pi(temp1 + temp3 - sc_params.thf);
  return true;
}

// RightStraightLeft DubinsArc
bool DubinsSolver::RSL(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(std::cos(sc_params.th0) + std::cos(sc_params.thf), 2 * sc_params.k_max - sin(sc_params.th0) - sin(sc_params.thf));
  double temp2 = 4 * sc_params.k_max * sc_params.k_max - 2 + 2 * std::cos(sc_params.th0 - sc_params.thf) - 4 * sc_params.k_max * (sin(sc_params.th0) + sin(sc_params.thf));
  if(temp2 < 0)
    return false;

  sc_s2 = invK * sqrt(temp2);
  temp2 = std::atan2(2, sc_s2 * sc_params.k_max);
  sc_s1 = invK * Utility::mod2pi(sc_params.th0 - temp1 + temp2);
  sc_s3 = invK * Utility::mod2pi(sc_params.thf - temp1 + temp2);
  return true;
}

// RightStraightRight DubinsArc
bool DubinsSolver::RLR(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(std::cos(sc_params.th0) - std::cos(sc_params.thf), 2 * sc_params.k_max - sin(sc_params.th0) + sin(sc_params.thf));
  double temp2 = 0.125 * (6 - 4 * sc_params.k_max * sc_params.k_max + 2 * std::cos(sc_params.th0 - sc_params.thf) + 4 * sc_params.k_max * (sin(sc_params.th0) - sin(sc_params.thf)));

  if(abs(temp2) > 1)
    return false;

  sc_s2 = invK * Utility::mod2pi(2 * M_PI - std::acos(temp2));
  sc_s1 = invK * Utility::mod2pi(sc_params.th0 - temp1 + 0.5 * sc_s2 * sc_params.k_max);
  sc_s3 = invK * Utility::mod2pi(sc_params.th0 - sc_params.thf + sc_params.k_max * (sc_s2 - sc_s1));
  return true;
}

// LeftRightLeft DubinsArc
bool DubinsSolver::LRL(const DubinsProblemScaledParameters &sc_params, double &sc_s1, double &sc_s2, double &sc_s3){
  double invK = 1 / sc_params.k_max;
  double temp1 = std::atan2(std::cos(sc_params.thf) - std::cos(sc_params.th0), 2 * sc_params.k_max + sin(sc_params.th0) - sin(sc_params.thf));
  double temp2 = 0.125 * (6 - 4 * sc_params.k_max*sc_params.k_max + 2 * std::cos(sc_params.th0 - sc_params.thf) - 4 * sc_params.k_max * (sin(sc_params.th0) - sin(sc_params.thf)));

  if(abs(temp2) > 1)
    return false;

  sc_s2 = invK * Utility::mod2pi(2 * M_PI - std::acos(temp2));
  sc_s1 = invK * Utility::mod2pi(temp1 - sc_params.th0 + 0.5 * sc_s2 * sc_params.k_max);
  sc_s3 = invK * Utility::mod2pi(sc_params.thf - sc_params.th0 + sc_params.k_max * (sc_s2 - sc_s1));
  return true;
}
