#include "MapGenerator.hpp"

// generate random map
Map MapGenerator::getRandomMap(int width, int height,int n_victims, float victims_radius, int n_obstacles, float obstacles_radius) {
    cout << "map generation started"<< endl;
    vector<Victim> victims = {};
    vector<Obstacle> obstacles = {};
    Obstacle goal = MapGenerator::getRandomGoal(width,height,obstacles_radius);

    // random victims generation
    for(int i = 0; i < n_victims;i++) {

        bool goodCoordinates = false;
        //cout << "victim " << i << " being generated" << endl;
        while(!(goodCoordinates)) {

            float xc = victims_radius +  static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(width-victims_radius)));
            float yc = victims_radius +  static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(height-victims_radius)));
            Point2f centerTrial = Point(xc,yc);
            goodCoordinates = true;

            // No close to the boundaries
            if(xc - victims_radius < 0 || yc - victims_radius < 0 || xc + victims_radius > width || yc + victims_radius > height) {
                goodCoordinates = false;
            }

            // other victims collisions
            for(unsigned int j = 0; j < victims.size();j++) {
                Circle c = Circle(victims[j].getCenter(),victims[j].getRadius()*2.1);
                if(c.isPointInside(centerTrial)) {
                    goodCoordinates = false;
                }
            }

            if(goodCoordinates) {
                victims.push_back(Victim(i+1,centerTrial,victims_radius));
            }

        }


    }

    // random obstacle generation
    for(int i = 0; i < n_obstacles; i++) {
        bool goodCoordinates = false;
        //cout << "obstacle " << i << " being generated" << endl;
        while(!(goodCoordinates)) {

            float xc = obstacles_radius +  static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(width-obstacles_radius)));
            float yc = obstacles_radius +  static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(height-obstacles_radius)));
            Point2f centerTrial = Point(xc,yc);
            goodCoordinates = true;
            vector<Point2f> vertices = MapGenerator::getRandomPolygon(centerTrial,obstacles_radius);

            // No close to the boundaries
            if(xc - obstacles_radius < 0 || yc - obstacles_radius < 0 || xc + obstacles_radius > width || yc + obstacles_radius > height) {
                goodCoordinates = false;
            }

            // victims collisions
            for(unsigned int j = 0; j < victims.size();j++) {
                for(unsigned int k = 0; k < vertices.size();k++) {
                    if(victims[j].isPointInside(vertices[k])) {
                        goodCoordinates = false;
                    }
                }
                if(victims[j].isPointInside(centerTrial)) {
                    goodCoordinates = false;
                }

            }

            // other obstacles collisions
            for(unsigned int j = 0; j < obstacles.size();j++) {
                BoundingBox bb = obstacles[j].getBoundingBox();
                for(unsigned int k = 0; k < vertices.size();k++) {
                    if(bb.isPointInside(vertices[k])) {
                        goodCoordinates = false;
                    }
                }
                if(bb.isPointInside(centerTrial)) {
                    goodCoordinates = false;
                }
            }

            if(goodCoordinates) {
                obstacles.push_back(Obstacle(centerTrial,vertices));
            }

        }


    }

    // create new map
    Map m = Map(width,height,goal,obstacles,victims);

    return m;
}

// goal generator
Obstacle MapGenerator::getRandomGoal(int width, int height,float obstacles_radius) {

    int location = 1 + (rand() % static_cast<int>(4 - 1 + 1));
    Point2f goal_center = Point2f(0,0);

    if(location == 1){// bottom
        float xc = width/2;
        float yc = obstacles_radius/ sqrt(2);
        goal_center = Point2f(xc,yc);
    } else if( location == 2) { // top
        float xc = width/2;
        float yc = height - obstacles_radius / sqrt(2);
        goal_center = Point2f(xc,yc);
    } else if(location == 3) { // left
        float xc = obstacles_radius / sqrt(2);
        float yc = height/2;
        goal_center = Point2f(xc,yc);
    } else if(location == 4) { // right
        float xc = width - obstacles_radius / sqrt(2);
        float yc = height/2;
        goal_center = Point2f(xc,yc);
    } else {
        cout << "goal generation number not recognized" << endl;
    }

    float half_side = obstacles_radius/ sqrt(2);
    Point2f p1 = Point2f(goal_center.x + half_side,goal_center.y + half_side);
    Point2f p2 = Point2f(goal_center.x - half_side,goal_center.y + half_side);
    Point2f p3 = Point2f(goal_center.x - half_side,goal_center.y - half_side);
    Point2f p4 = Point2f(goal_center.x + half_side,goal_center.y - half_side);
    vector<Point2f> vertices = {};
    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
    vertices.push_back(p4);

    Obstacle goal = Obstacle(goal_center,vertices);

    return goal;
}


// generate polygon vertices
vector<Point2f> MapGenerator::getRandomPolygon(Point2f center, float obstacles_radius) {
    vector<Point2f> vertices = {};
 
    vertices.push_back(Point2f(center.x - 3*obstacles_radius/4 , center.y));
    vertices.push_back(Point2f(center.x - obstacles_radius/4 , center.y - 3*obstacles_radius/4 ));
    vertices.push_back(Point2f(center.x + 3*obstacles_radius/4 , center.y));
    vertices.push_back(Point2f(center.x + obstacles_radius/4 , center.y + 3*obstacles_radius/4 ));
   
    return vertices;
}
