#include "PathPlanner.hpp"
//#define SHOW_PATHPLANNER_STEPS

// constructor
PathPlanner::PathPlanner() {
    cd = CollisionDetector();
    pg = PointsGraph();
    m = Map();
    victim_bonus = 0;
    k = 1;
    robot_velocity = 1;
}

// full constructor
PathPlanner::PathPlanner(const Map &_m, const PointsGraph &_pg, float robot_k,float robot_velocity,float victim_bonus) {
    this->m = _m;
    this->cd = CollisionDetector(m);
    this->pg = _pg;
    this->victim_bonus = victim_bonus;
    this->robot_velocity = robot_velocity;
    this->k=robot_k;
}

// from dubins curves to a path object
void PathPlanner::dubinsCurvesToPoints(const vector<DubinsCurve> &curves, double path_length, Path &path){
  double sampling_length = 10;
  path.points.clear();
  path.points.reserve(path_length / sampling_length);
  std::cout << "CURVEs: " << curves.size() << std::endl;
  double overlength = 0;
  unsigned int i = 0;
  std::cout << "Starting conversion" << std::endl;
  if(curves.size() > 0){
    for(; i < curves.size(); i++){
      overlength = curves[i].addPoints(overlength, path);
    }
    i--;
    path.points.push_back(Pose(path.points.back().s + overlength, curves[i].getXf(), curves[i].getYf(), curves[i].getThf(), curves[i].getArc(2).getK()));

    for(i = 0; i < path.points.size(); i++){
    	Pose &p = path.points[i];
    	p.s /= 1000;
    	p.kappa *= -1000;
    	//p.x += 35 * std::cos(p.theta);
    	//p.y += 35 * std::sin(p.theta);
    	p.x /= 1000;
    	p.y = (m.getHeight() - p.y)/1000;
    	p.theta = -p.theta;
    }
  }
  std::cout << "Finished conversion" << std::endl;
}

// Point-to-point general path
bool PathPlanner::getPath(Point2f start_point, float start_angle, Point2f end_point, float *end_angle, vector<DubinsCurve> *pathreturned) {
    // find the closest point in the graph to the start point and to the end point
    vector<DubinsCurve> pathplanned = {};
    const vector<PointNode> &nodes = pg.getNodes();
    int closest_to_start_index = 0;
    int closest_to_end_index = 0;
    float closest_start_distance = 1000000000;
    float closest_end_distance = 1000000000;

    // calculate the closest point of the graph to the start and end point
    for(unsigned int i = 0; i < nodes.size();i++) {

        float px = nodes[i].getX();
        float py = nodes[i].getY();

        float start_distance = (start_point.x - px)*(start_point.x - px) + (start_point.y - py)*(start_point.y - py);
        start_distance = sqrt(start_distance);
        if(start_distance < closest_start_distance) {
            closest_start_distance = start_distance;
            closest_to_start_index = i;
        }

        float end_distance = (end_point.x - px)*(end_point.x - px) + (end_point.y - py)*(end_point.y - py);
        end_distance = sqrt(end_distance);
        if(end_distance < closest_end_distance) {
            closest_end_distance = end_distance;
            closest_to_end_index = i;
        }

    }

    // print the information
    Point2f startGraphPoint = Point2f(  nodes[closest_to_start_index].getX(), nodes[closest_to_start_index].getY()   );
    Point2f endGraphPoint = Point2f(  nodes[closest_to_end_index].getX(), nodes[closest_to_end_index].getY()   );
    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "closest start graph point: " << startGraphPoint << endl;
        cout << "closest end graph point: " << endGraphPoint << endl;
    #endif

    // calculate the best solution of the graph
    vector<Point2f> best_solution;
    pg.shortestPath(startGraphPoint,endGraphPoint,best_solution);

    /*
    Plotter p = Plotter();
    p.drawMap(m);
    for(int i = 1; i < best_solution.size(); i++ ) {
        p.drawSegment(Segment(best_solution[i-1],best_solution[i]));
    }
    p.show();*/

    // add to the solution at the start the initial point and at the end the final point
    //cout << "shortest path computed" << endl;
    best_solution.insert(best_solution.begin(),start_point);
    best_solution.push_back(end_point);


    // try a solution calling the recursive funztion
    bool existingsolution = false;
    existingsolution = recursive(0,start_angle,best_solution.size()-1,end_angle,best_solution, pathreturned);
    if(existingsolution == true) {  // success, put the new solution in the returning solution
        return true;
    } else {    // failure
        #ifdef SHOW_PATHPLANNER_STEPS
            cout << "best solution not found from " << start_point << " with angle " << start_angle << " to " << end_point  << ""  << endl;
        #endif
        return false;

    }

}

// Point-to-point specific path with recursiveiterations
bool PathPlanner::recursive(int start_index, float start_angle, int end_index, float* end_angle, const vector<Point2f> &graph, vector<DubinsCurve> *solution) {

    bool finished = false;
    Point2f start_point = graph[start_index];
    Point2f end_point = graph[end_index];

    float first_arrival_angle;
    if(end_index < ((int)graph.size() - 1) && end_angle == NULL) {
        float random_part =  (-20.0f + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(40.0f))) )/100.0f;
        first_arrival_angle = atan2(graph[end_index+1].y - end_point.y, graph[end_index+1].x - end_point.x) + random_part; // can be usefoul?

    } else if (end_angle != NULL) {
        first_arrival_angle = *end_angle;
        #ifdef SHOW_PATHPLANNER_STEPS
            cout << "end angle: " << *end_angle << endl;
        #endif
    } else {
        first_arrival_angle = ( (float)(rand() % 628) / 100 ) ; // from 0 to 2 Pi with resolution of 0.01
    }
    DubinsCurve dc_first;
    bool feasiblepath_first = cd.pathWithoutCollisions(start_point.x, start_point.y, start_angle,end_point.x,end_point.y,first_arrival_angle,k,dc_first);
    if(feasiblepath_first) {
        finished = true;
        solution->push_back(dc_first);
        return true;
    }

    // direct random search
    for(unsigned int j = 0; j < 30 && !finished;j++) {
        float randomArrivalAngle;
        if(end_angle == NULL) {
            randomArrivalAngle = ( (float)(rand() % 628) / 100 ) ; // from 0 to 2 Pi with resolution of 0.01
        } else {
            randomArrivalAngle = ( (float)(rand() % 60) / 100 ) ; // from 0 to 0.6 radiants with resolution of 0.01
            randomArrivalAngle = randomArrivalAngle - 0.3 + *end_angle; // end angle +- 0.3 radiants is the tolerance
        }

        DubinsCurve dc;
        bool feasiblepath = cd.pathWithoutCollisions(start_point.x, start_point.y, start_angle,end_point.x,end_point.y,randomArrivalAngle,k,dc);
        if(feasiblepath) {
            finished = true;
            solution->push_back(dc);
            return true;
        }

    }

    // no feasable path found, if the two index are subsequent points, there is no feasable path
    if(end_index-start_index <= 1) {
        return false;
    } else { // else split the research in two steps

        int middlepointindex = ( (end_index + start_index) / 2 ) ;
        vector<DubinsCurve> s_copy = {};

        bool first_half_solution = this->recursive(start_index,start_angle,middlepointindex,NULL,graph,&s_copy);
        if( first_half_solution) { // first part success, check the second part
            float last_angle = s_copy.back().getThf();
            bool second_half_solution = this->recursive(middlepointindex,last_angle,end_index,end_angle,graph,&s_copy);

            if(second_half_solution) { // second solution success, return success

                for(unsigned int i = 0; i < s_copy.size();i++) {
                    solution->push_back(s_copy[i]);
                }

                return true;
            } else { // second solution failure, try to recompute the first one
                s_copy.clear();
                first_half_solution = this->recursive(start_index,start_angle,middlepointindex,NULL,graph,solution);
                if(first_half_solution) {
                    last_angle = solution->back().getThf();
                    second_half_solution = this->recursive(middlepointindex,last_angle,end_index,end_angle,graph,solution);

                    if(second_half_solution) {
                        #ifdef SHOW_PATHPLANNER_STEPS
                            cout << "retrial of the p-t-p path successful" << endl;
                        #endif
                        return true;
                    } else {
                        return false;
                    }

                } else {
                    return false;
                }

               return false;
            }
        } else { // first part failure, return failure
            return false;
        }


    }

}

// Path to save all the victims in order
bool PathPlanner::saveVictimsPath(Point2f start_point, float start_angle,vector<DubinsCurve> &returnedPath) {
    vector<Point2f> points_to_visit = {};
    points_to_visit.push_back(start_point);
    float goal_end_angle = calculateGoalArrivalAngle();
    vector<Victim> &victims = m.getVictims(); // victims are in order in map
    std::sort(victims.begin(),victims.end());
    BoundingBox mapLimits = BoundingBox(0,0,m.getWidth(),m.getHeight());
    vector<Obstacle> &obstacles = m.getObstacles();

    for(unsigned int i = 0; i < victims.size();i++) {
        Point2f new_point = getPointInsideCircle(victims[i],obstacles,mapLimits);
        points_to_visit.push_back(new_point);
    }

    points_to_visit.push_back(m.getGoal().getCenter());

    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "Points to visit: " << endl;
        for(unsigned int i = 0; i < points_to_visit.size();i++) {
            cout << " " << points_to_visit[i] ;
        } cout << endl;
    #endif

    return generalPathPlanner(start_angle, goal_end_angle, points_to_visit, &returnedPath);
}

// calculate the arrival angle based on the goal position
float PathPlanner::calculateGoalArrivalAngle() {
    float arrivalAngle;

    Obstacle goal = m.getGoal();
    Point2f goalCenter = goal.getCenter();
    float xc = goalCenter.x;
    float yc = goalCenter.y;
    float x_to_confront = xc;
    float y_to_confront = yc;
    bool right;
    bool top;

    if( xc < (m.getWidth()-xc)) { // the goal is closer to the left
        x_to_confront = xc;
        right = false;
    } else {
        x_to_confront = m.getWidth()-xc;
        right = true;
    }
    if( yc < (m.getHeight()-yc)) {
        top = false;
        y_to_confront = yc;
    } else {
        top = true;
        y_to_confront = m.getHeight()-yc;
    }

    if(x_to_confront <= y_to_confront) { // goal is on the right or on the left
        if(right) {
            arrivalAngle = 0;
        } else {
            arrivalAngle = M_PI;
        }
    } else {
        if(top) {
            arrivalAngle = M_PI/2;
        } else {
            arrivalAngle = ( M_PI * 3) /2;
        }
    }


    return arrivalAngle;
}

// get the two closest point in the graph to the two closest point in the input
void PathPlanner::getClosestPointsToGraph(Point2f &start_point, Point2f &end_point) {
    vector<PointNode> &nodes = pg.getNodes();
    int closest_to_start_index = 0;
    int closest_to_end_index = 0;
    float closest_start_distance = 1000000000;
    float closest_end_distance = 1000000000;

    // calculate the closest point of the graph to the start and end point
    for(unsigned int i = 0; i < nodes.size();i++) {

        float px = nodes[i].getX();
        float py = nodes[i].getY();

        float start_distance = (start_point.x - px)*(start_point.x - px) + (start_point.y - py)*(start_point.y - py);
        start_distance = sqrt(start_distance);
        if(start_distance < closest_start_distance) {
            closest_start_distance = start_distance;
            closest_to_start_index = i;
        }

        float end_distance = (end_point.x - px)*(end_point.x - px) + (end_point.y - py)*(end_point.y - py);
        end_distance = sqrt(end_distance);
        if(end_distance < closest_end_distance) {
            closest_end_distance = end_distance;
            closest_to_end_index = i;
        }

    }

    // print the information
    start_point = Point2f(  nodes[closest_to_start_index].getX(), nodes[closest_to_start_index].getY()   );
    end_point = Point2f(  nodes[closest_to_end_index].getX(), nodes[closest_to_end_index].getY()   );
}

// function that prints a float matrix
void printMatrix( vector<vector<float>> matrix) {

    for(unsigned int i = 0; i < matrix.size();i++) {

        for(unsigned int j = 0; j < matrix[i].size();j++) {
            cout << " " << matrix[i][j];
        }
        cout << endl;
    }
    cout << endl;

}

// Mission planner algorithm
bool PathPlanner::missionPlan(Point2f start_point, float start_angle,vector<DubinsCurve> &returnedPath,float &path_score) {
    // Insert the points to visit, they are the start, all the vistims and the goal
    vector<Point2f> points_to_visit = {};
    points_to_visit.push_back(start_point);
    vector<Victim> &victims = m.getVictims();
    BoundingBox mapLimits = BoundingBox(0,0,m.getWidth(),m.getHeight());
    vector<Obstacle> &obstacles = m.getObstacles();

    for(unsigned int i = 0; i < victims.size();i++) {
        Point2f new_point = getPointInsideCircle(victims[i],obstacles,mapLimits);
        points_to_visit.push_back(new_point);
    }
    points_to_visit.push_back(m.getGoal().getCenter());

    // The matrix of all the distances of all the points to visit to each other
    vector<vector<float>> matrix;

    // initialize the distance matix
    for(unsigned int i = 0; i < points_to_visit.size();i++) {
        vector<float> temp = {};
        matrix.push_back(temp);
        for(unsigned int j = 0; j < points_to_visit.size();j++) {
            matrix[i].push_back(0.0f);
        }

    }

    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "Points to visit: " << endl;
        for(unsigned int i = 0; i < points_to_visit.size();i++) {
            cout << " " << points_to_visit[i] << " ";
        } cout << endl << endl;
    #endif

    // find the distances between the points
    for(unsigned int i = 0; i < points_to_visit.size();i++) {

        //Plotter plotter = Plotter();
        //plotter.drawMap(m);

        for(unsigned int j = 0; j < points_to_visit.size();j++) {

            if(i == j) { // the distance from a point to itself is 0
                matrix[i][j] = 0.0f;
            } else { // find the points to visit, search for the two closests point in the graph, find a path from that two points and calculate the total path lenght
                Point2f start = points_to_visit[i];
                Point2f end = points_to_visit[j];

                this->getClosestPointsToGraph(start,end);

                vector<Point2f> best_solution;
                pg.shortestPath(start,end,best_solution);
                best_solution.insert(best_solution.begin(),points_to_visit[i]);
                best_solution.push_back(points_to_visit[j]);

                float lenght = getGraphPathLenght(best_solution);
                matrix[i][j] = lenght;

                for(unsigned int k = 0; k < best_solution.size();k++) {
                    if(k != 0) {
                        //plotter.drawSegment(Segment(best_solution[k-1],best_solution[k]),Scalar(0,0,255));
                    }

                }

            }

        }
        //plotter.show();

    }

    #ifdef SHOW_PATHPLANNER_STEPS
        printMatrix(matrix);
    #endif

    // rescale the distance based on the relative distance between a victim to the others
    vector<float> ratios = {};

    for(unsigned int i = 1; i < points_to_visit.size()-1;i++) {
        float sum = 0 ;

        for(unsigned int j = 1; j < points_to_visit.size()-1;j++) {
            sum = sum + matrix[i][j];
        }

        ratios.push_back( ( sum/victims.size() ) / 2);
    }

    // rescale the matrix
    for(unsigned int i = 1; i < points_to_visit.size()-1;i++) {

        #ifdef SHOW_PATHPLANNER_STEPS
            cout << "ratio victim " << i << " : " << ratios[i-1]  << endl;
        #endif
        for(unsigned int j = 0; j < points_to_visit.size()-1;j++) {
            if(j != i) {
                float tmp = ratios[i-1] + matrix[j][i];
                matrix[j][i] = tmp;
            }

        }
    }
    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "updated matrix" << endl;
        printMatrix(matrix);
        cout << endl;
    #endif

    float end_angle = this->calculateGoalArrivalAngle();

    // get the various orders and evaluate them

    vector<vector<Point2f>> path_order_trials = {};
    vector<vector<DubinsCurve>> path_trial = {};

    getVictimsOrder(points_to_visit,matrix,path_order_trials);

    #ifdef SHOW_PATHPLANNER_STEPS
        for(unsigned int i = 0; i < path_order_trials.size();i++) {
            cout<< "Order solution in the path planner" << i << " : ";
            for(unsigned int j = 0; j < path_order_trials[i].size();j++) {
                cout << " " << path_order_trials[i][j];
            }
            cout << endl;

        }
    #endif

    float min_score = 1000000;
    int min_score_index = -1;

    for(unsigned int i = 0; i < path_order_trials.size();i++) {
        path_trial.push_back({});

        if(generalPathPlanner(start_angle,end_angle, path_order_trials[i],&path_trial[i])) {
            #ifdef SHOW_PATHPLANNER_STEPS
                cout << "path " << i << " feasable" << endl;
            #endif
            int n_victim_rescued = path_order_trials[i].size() - 2;

            if(calculatePathScore(path_trial[i],n_victim_rescued ) < min_score) {
                min_score = calculatePathScore(path_trial[i],n_victim_rescued );
                min_score_index = i;
            }

            #ifdef SHOW_PATHPLANNER_STEPS
                Plotter p;
                p.drawMap(m);
                p.drawDubinsCurves(path_trial[i]);
                p.show();
            #endif

        } else {
            #ifdef SHOW_PATHPLANNER_STEPS
                cout << "path " << i << " NOT feasable" << endl;
            #endif
        }

    }
    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "Min score: " << min_score << " min score index " << min_score_index << endl;
    #endif

    if(min_score_index== -1) {
        return false;
    }

    for(unsigned int i = 0; i < path_trial[min_score_index].size();i++) {
        returnedPath.push_back(path_trial[min_score_index][i]);
    }

    path_score = min_score;
    return true;
}

// get the lenght of a path of points
float PathPlanner::getGraphPathLenght(const vector<Point2f> &path) {
    float tot = 0;

    for(unsigned int i = 0; i < path.size();i++) {

        if(i != path.size() -1) {
            tot += sqrt((path[i].x - path[i+1].x)*(path[i].x - path[i+1].x) + (path[i].y - path[i+1].y)*(path[i].y - path[i+1].y));
        }

    }

    return tot;
}

// general multiple point path planner
bool PathPlanner::generalPathPlanner(float start_angle, float end_angle, const vector<Point2f> &points_to_visit, vector<DubinsCurve>* solution) {

    Point2f start_point = points_to_visit[0];

    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "Points to visit: " << endl;
        for(unsigned int i = 0; i < points_to_visit.size();i++) {
            cout << " " << points_to_visit[i];
        } cout << endl;
    #endif

    vector<vector<DubinsCurve>> solutions = {};

    // for every victim find a path from the start to the victim
    for (unsigned int i = 1; i < points_to_visit.size();i++ ){
        Point2f end_point = points_to_visit[i];
        vector<DubinsCurve> a = {};
        solutions.push_back(a);

        vector<DubinsCurve>tmp = {};

        bool feasable_path = false;

        if(i == points_to_visit.size()-1) {
            feasable_path = this->getPath(start_point,start_angle,end_point,&end_angle,&solutions[i-1]);
        } else {
            feasable_path = this->getPath(start_point,start_angle,end_point,NULL,&solutions[i-1]);
        }

        // retry the previous point-to-point connection
        if( (!(feasable_path)) && i != 1) {
            #ifdef SHOW_PATHPLANNER_STEPS
                cout << " we are retrying" << endl;
            #endif
            solutions[i-1].clear();

            // recompute previous path
            Point2f old_start_point = Point2f(solutions[i-2][0].getX0(),solutions[i-2][0].getY0());
            float old_start_angle = solutions[i-2][0].getTh0();
            Point2f old_finish_point = start_point;
            solutions[i-2].clear();

            feasable_path = this->getPath(old_start_point,old_start_angle,old_finish_point,NULL,&solutions[i-2]);

            if(feasable_path == false) {
                #ifdef SHOW_PATHPLANNER_STEPS
                    cout << "retrying failed first" << endl;
                #endif
                return false;
            } else {
                #ifdef SHOW_PATHPLANNER_STEPS
                    cout << "retrying first successful" << endl;
                #endif
                // recompute current path
                start_angle = solutions[i-2].back().getThf();
                if(i == points_to_visit.size()-1) {
                    feasable_path = this->getPath(start_point,start_angle,end_point,&end_angle,&solutions[i-1]);
                } else {
                    feasable_path = this->getPath(start_point,start_angle,end_point,NULL,&solutions[i-1]);
                }

                if(!feasable_path) {
                    #ifdef SHOW_PATHPLANNER_STEPS
                        cout << "retrying second failed" << endl;
                     #endif
                    return false;
                }
                // otherwise success
                #ifdef SHOW_PATHPLANNER_STEPS
                    cout << "retrying of the path from " << start_point << " to " << end_point << " successful" << endl;
                #endif
            }

        } else if( feasable_path == false) {
            #ifdef SHOW_PATHPLANNER_STEPS
                cout << "Complete path not found" << endl;
            #endif

            return false;
        }

        start_point = end_point;
        start_angle = solutions[i-1].back().getThf();

    }

    for(unsigned int i = 0; i < solutions.size();i++) {

        for(unsigned int j = 0; j < solutions[i].size();j++) {
            solution->push_back(solutions[i][j]);
        }
    }



    return true;
}

// get a random point inside a victim that does not collide with obstacles and that does not exit from the map
Point2f PathPlanner::getPointInsideCircle(const Victim &v, const vector<Obstacle>&obstacles, const BoundingBox &mapLimits) {

    Point2f center = v.getCenter();
    float range = v.getRadius();
    bool acceptablePoint = false;
    Point2f new_point = Point2f(0,0);

    while(!(acceptablePoint)) {

        float noise_x = -range/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(range)));
        float noise_y = -range/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(range)));
        new_point = Point2f(center.x+noise_x,center.y+noise_y);
        bool isInsideObstacle = false;

        if(!(mapLimits.isPointInside(new_point))) {
            continue;
        }

        for(unsigned int j = 0; j < obstacles.size();j++) {
            if(obstacles[j].isPointInside(new_point)) {
                isInsideObstacle = true;
            }
        }

        if(isInsideObstacle) {
            continue;
        }

        acceptablePoint = true;

    }

    return new_point;
}

// get the lenght of a path of DubinsCurve
float PathPlanner::calculatePathLenght(const vector<DubinsCurve> &path) {

    float tot_lenght = 0;

    for(unsigned int i = 0; i < path.size();i++) {
        tot_lenght += path[i].getTotalLenght();
    }

    return tot_lenght;

}

// Calculate the score (in seconds) of a path
float PathPlanner::calculatePathScore(const vector<DubinsCurve> &path, int n_victims_rescued) {

    float score = ( PathPlanner::calculatePathLenght(path) / robot_velocity ) - n_victims_rescued*this->victim_bonus;
    #ifdef SHOW_PATHPLANNER_STEPS
        cout << " pathLenght " <<  PathPlanner::calculatePathLenght(path) << " velocity " << robot_velocity << " nvictims rescued " << n_victims_rescued
        << " bonus " << victim_bonus << " score: " << score << endl;
    #endif

    return score;

}

// Get int the returned points a vector of vector containing the best order or visiting each victim for any number of victims
void PathPlanner::getVictimsOrder(vector<Point2f> points_to_visit, const vector<vector<float>> &distances, vector<vector<Point2f>> &return_points) {

    vector<vector<float>> updated_distances = distances;
    vector<Point2f> new_points_to_visit = points_to_visit;

    for(unsigned int i = 0; i < points_to_visit.size()-1; i++) {

        if(i != 0) {
            int victim_to_eliminate = 0;
            updated_distances = eliminateFurthestVictimFromMatrix(updated_distances,victim_to_eliminate);
            new_points_to_visit.erase(new_points_to_visit.begin() + victim_to_eliminate);
        }

        vector<vector<float>> distances_copy = updated_distances;

        #ifdef SHOW_PATHPLANNER_STEPS
            cout << " Trial n " << i << " matrix: " << endl;
            printMatrix(distances_copy);
        #endif

        int current_pos = 0;
        vector<Point2f> order = {};
        order.push_back(new_points_to_visit[0]);

        for(unsigned int j = 0; j < distances_copy.size()-2;j++) {

            float minimum = 1000000;
            int minimum_index = 0;

            for(unsigned int k = 0; k < distances_copy.size()-1;k++) {
                if(distances_copy[current_pos][k] != 0 && distances_copy[current_pos][k] < minimum) {
                    minimum = distances_copy[current_pos][k];
                    minimum_index = k;
                }
            }
            order.push_back(new_points_to_visit[minimum_index]);

            for(unsigned int k = 0; k < distances_copy.size();k++) {
                distances_copy[k][minimum_index] = 0;
                if(current_pos == 0) {
                    distances_copy[k][0] = 0;
                }
            }

            current_pos = minimum_index;
        }
        order.push_back(m.getGoal().getCenter());
        return_points.push_back(order);
    }

}

// Method to eliminate the furtherst victim(from the start point and the last point) from the matrix of the distances
vector<vector<float>> PathPlanner::eliminateFurthestVictimFromMatrix(vector<vector<float>> matrix, int &victim_to_eliminate) {

    float max_dis = 0;
    int max_dis_index = 0;


    for(unsigned int i = 1; i < matrix.size();i++) {

        float dis = matrix[0][i] + matrix[matrix.size()-1][i];

        if(dis > max_dis) {
            max_dis = dis;
            max_dis_index = i;
        }

    }

    #ifdef SHOW_PATHPLANNER_STEPS
        cout << "victim further away: " << max_dis_index <<  " with distance " << max_dis << endl;
    #endif
    victim_to_eliminate = max_dis_index;

    vector<vector<float>> new_matrix = {};

    for(unsigned int i = 0; i < matrix.size();i++) {

        if( (int)i != max_dis_index) {
            vector<float> new_row = {};
            for(unsigned int j = 0; j < matrix[i].size();j++) {

                if((int)j != max_dis_index) {
                    new_row.push_back(matrix[i][j]);
                }

            }
            new_matrix.push_back(new_row);
        }

    }

    return new_matrix;

}
