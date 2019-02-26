#include "robot_project.h"
#include "Recognitor.hpp"
#include "Plotter.hpp"
#include "PathPlanner.hpp"
#include "Decomposer.hpp"
#include "Obstacle.hpp"
#include "DubinsCurve.hpp"
#include "DubinsSolver.hpp"
#include "json.hpp"

#include <chrono>
#include <cstdlib>

RobotProject::RobotProject(int argc, char * argv[]) : detector(){
  srand(time( NULL ));
  json11::Json json;
  JSONCompatible::read_json(json, "input/planning_parameters.json");
  fromJSON(json);
}

void RobotProject::fromJSON(const json11::Json &json){
  json11::Json sub_json = JSONCompatible::element_of(json, "PointsPerCell");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading points per cell", "not receiving a number");
  points_per_cell = sub_json.int_value();

  if(points_per_cell < 1)
    throw JSONCompatible::InvalidJSON("reading points per cell", "receiving a number smaller than 1");

  sub_json = JSONCompatible::element_of(json, "ClippingQuantity");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading clipping quantity", "not receiving a number");
  clippingQuantity = sub_json.number_value();

  sub_json = JSONCompatible::element_of(json, "VictimBonus");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading victim bonus", "not receiving a number");
  victim_bonus = sub_json.number_value();

  sub_json = JSONCompatible::element_of(json, "RobotCurvature");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading robot curvature", "not receiving a number");
  robot_k = sub_json.number_value();

  sub_json = JSONCompatible::element_of(json, "RobotVelocity");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading robot velocity", "not receiving a number");
  robot_velocity = sub_json.number_value();

  sub_json = JSONCompatible::element_of(json, "MissionType");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading mission type", "not receiving a number");
  mission_type = sub_json.int_value();

  sub_json = JSONCompatible::element_of(json, "NumberTrials");
  if(sub_json.type() != json11::Json::NUMBER)
    throw JSONCompatible::InvalidJSON("reading number of trials", "not receiving a number");
  n_of_trials = sub_json.number_value();

}

json11::Json RobotProject::toJSON() const{
  return json11::Json();
}


bool RobotProject::preprocessMap(cv::Mat const & img){
  cv::Mat img_in = img.clone();

  detector.detect_board_and_warp(img_in);
  detector.detect_robot_plane(img);
  std::vector<double> state;
  if(detector.localize(img, state)){
	state[0] -= clippingQuantity;
	state[1] -= clippingQuantity;
	//state[0] -= 35 * std::cos(state[2]);
	//state[1] -= 35 * std::sin(state[2]);
	robot_x = state[0];
	robot_y = state[1];
	robot_theta = state[2];
  } else {
	throw std::logic_error("Unable to localise");
  }
  json11::Json json;
  json11::Json &j = JSONCompatible::read_json(json, "input/recognition_parameters.json");
  LaR::Recognitor recognitor(j);
  recognitor.recognize(img_in, map);

  #define SHOW_INTERMEDIATE_PLANNER

  #ifdef SHOW_INTERMEDIATE_PLANNER
    map.print();
    map.printOnFileSystem("map_txt_version.txt");
    std::cout <<"PLOTTING VIRTUAL MAP" << std::endl;

    Plotter p = Plotter();
    p.drawMap(map);
    std::cout << "COORDINATES: " << robot_x << " " << robot_y << " " << robot_theta << std::endl;
    p.drawCircle(Circle(cv::Point2f(robot_x, robot_y), 4));
    p.show();
    p.show();
  #endif


  return true;
}

bool RobotProject::planPath(cv::Mat const & img, Path & path){
    float start_point_x = robot_x;
    float start_point_y = robot_y;
    float start_angle = robot_theta;

    Plotter plot = Plotter();
    plot.drawMap(map);

    vector<Obstacle> nonoverlappingPolygons = {};
    Decomposer::prepareMap(clippingQuantity,map,nonoverlappingPolygons);
    cout << "MAP preprocessed: " << endl; map.print(); plot.drawMap(map);

    PointsGraph pg = PointsGraph();
    Decomposer::decompose(pg,map,nonoverlappingPolygons,points_per_cell);

    cout << "Computed graph of points" << endl;
    //pg.print();
    plot.drawPointsGraph(pg);  plot.drawMap(map);

    PathPlanner pathplanner = PathPlanner(map,pg, robot_k, robot_velocity, victim_bonus);
    vector<vector<DubinsCurve>> paths = {};

    float min_score = 100000000;
    int min_score_index = 0;
    float total_time = 0;

    for(int i = 0; i < n_of_trials; i++) {
      cout << "path " << i << " being computed" << endl;
      vector<DubinsCurve> vec = {};
      paths.push_back(vec);

      bool feasable_path = false;
      float current_score = 0;
      auto start = std::chrono::high_resolution_clock::now();
      if(mission_type == 1) {
        feasable_path = pathplanner.saveVictimsPath(Point2f(start_point_x,start_point_y),start_angle,paths[i]);
        current_score = PathPlanner::calculatePathLenght(paths[i]);
      } else if(mission_type == 2) {
        feasable_path = pathplanner.missionPlan(Point2f(start_point_x,start_point_y),start_angle, paths[i],current_score);
      }
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> vector_setup = finish - start;
      cout << "time: " << vector_setup.count() << endl;
      total_time = total_time + vector_setup.count();

      if(feasable_path) {
        cout << "Trial " << i << " score: " << current_score << endl;
        if(current_score < min_score) {
          min_score = current_score;
          min_score_index = i;
        }
        plot.drawMap(map);
        plot.drawDubinsCurves(paths[i]);
        plot.show();
      }


    }

    cout << "Best path, with score: " << min_score << endl;
    cout << "Total computation time: " << total_time << " Average time per trial: " << total_time/n_of_trials << endl;
    plot.drawMap(map);
    plot.drawDubinsCurves(paths[min_score_index]);
    plot.show();

    pathplanner.dubinsCurvesToPoints(paths[min_score_index],PathPlanner::calculatePathLenght(paths[min_score_index]),path);
	std::cout << "Path computed" << std::endl;
    return true;
}

bool RobotProject::localize(cv::Mat const & img, std::vector<double> & state){
  if(detector.localize(img, state)){
	state[0] -= clippingQuantity;
	state[1] -= clippingQuantity;

	state[0] /= 1000;
	state[1] = (map.getHeight() - state[1])/1000;
	state[2] = -state[2];

	robot_x = state[0];
	robot_y = state[1];
	robot_theta = state[2];
	return true;
  }
  return false;
}
