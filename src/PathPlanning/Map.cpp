#include "Map.hpp"

/**
 * Empty costructor, it sets null or 0 all the attributes
 */
Map::Map() {
    width = 0;
    height = 0;
    vector<Point2f> goal_vertices;
    goal_vertices.push_back(Point2f(0,0));
    goal_vertices.push_back(Point2f(0,0));
    goal_vertices.push_back(Point2f(0,0));
    goal_vertices.push_back(Point2f(0,0));
    goal = Obstacle(Point(0,0),goal_vertices);
    obstacles = {};
    victims = {};
}

/**
 * Full constructor
 */
Map::Map(int width, int height, Obstacle goal, vector<Obstacle> obstacles, vector<Victim> victims) {
    this->width = width;
    this->height = height;
    this->goal = goal;
    this->obstacles = obstacles;
    this->victims = victims;
    sort(victims.begin(),victims.end());
}

// get the width
int Map::getWidth() const {
    return width;
}

// get the height
int Map::getHeight() const {
    return height;
}

// get the goal 
Obstacle Map::getGoal() const {
    return this->goal;
}

// get the obstacles
vector<Obstacle>& Map::getObstacles() {
    return this->obstacles;
}

// copy the new obstacles
void Map::setObstacles(const vector<Obstacle> &new_obstacles) {
    this->obstacles.clear();
    for(unsigned int i = 0; i < new_obstacles.size();i++) {
        this->obstacles.push_back( new_obstacles[i] );
    }
}

// get the victims
vector<Victim>& Map::getVictims() {
    return this->victims;
}

/*
* Function that print every information in the map
*/
void Map::print() {
    cout << "MAP width: " << width << " height: " << height << endl;

    cout << "Goal: " ;
    this->goal.print();

    cout << "N° victims: " << victims.size() << endl;
    for(unsigned int i = 0; i < victims.size(); i++) {
        cout << "Circle " << i << "  ";
        victims[i].print();
    }

    cout << "N° obstacles: " << obstacles.size() << endl;
    for(unsigned int i = 0; i < obstacles.size(); i++) {
        cout << "Obstacle " << i << "  ";
         obstacles[i].print();

    }

}

// function that clip the edges of all the polygons
void Map::clipObstaclesEdges(float offset) {

    for(unsigned int i = 0; i < obstacles.size(); i++) {
        obstacles[i].clipperEdges(offset);
    }
}

/**
 * Function that prints on the file system the object as a txt file
 *
 * @argument filename: the filename of the file where te output will be written
 *
 * The format is the following:
 *
 * Width Height
 * Number_of_obstacles+number_of_victims
 * X_center_goal Y_center_goal Goal_vertice_1 Goal_vertice_2 Goal_vertice_3 Goal_vertice_4
 * X_center_shape Y_center_shape Num_vertices Vertice1_shape ...
 *
 * the last row was a generis shape description, with first the coordinates of the center,
 * then how many vertices has the shape and then all the coordinates of the shape.
 * Is the shape is a circle it has 0 vertices, and after the 0 there is the radius and the number
 *
 */
void Map::printOnFileSystem(string filename) {
    // Open the file
    ofstream myfile;
    // write width and height of the map
    myfile.open (filename);
    myfile << width << " " << height << "\n";

    // write the total number of shapes in the map (excluding the goal)
    int tot_shapes = obstacles.size() + victims.size();
    myfile << tot_shapes << "\n";

    // write the goal center and the 4 coordinates of its vertices
    Point2f goal_center = goal.getCenter();
    myfile << goal_center.x << " " << goal_center.y << " ";
    vector<Point2f> goal_vertices = goal.getVertices();
    if(goal.getVertices().size() == 4){
      for(unsigned int i = 0; i < 4; i++) {
          Point2f p = goal_vertices[i];
          myfile << p.x << " " << p.y << " ";
      }
      myfile << "\n";
    } else {
      for(unsigned int i = 0; i < 4; i++) {
          Point2f p(0,0);
          myfile << p.x << " " << p.y << " ";
      }
      myfile << "\n";
    }

    // iterate every obstacle and write its center, its number of vertices and the coordinates of each vertice
    for(unsigned int i = 0; i < obstacles.size(); i++) {
        Obstacle poly = obstacles[i];
        Point2f p = obstacles[i].getCenter();
        myfile << p.x << " " << p.y << " ";
        std::vector<cv::Point2f> vertices = poly.getVertices();
        myfile << poly.getNVertices() << " ";

        for(int j = 0; j < poly.getNVertices();j++) {
            myfile << vertices[j].x << " " << vertices[j].y << " ";
        }
        myfile << "\n";
    }

    // iterate every circle and write its center, the number 0 and then its radius and its number
    for(unsigned int i = 0; i < victims.size(); i++) {
        Point2f center = victims[i].getCenter();
        myfile << center.x << " " << center.y << " 0 " << victims[i].getRadius() << " " << victims[i].getNumber();
        myfile << "\n";
    }
    myfile << "\n";

    myfile.close();
}

/*
* Function that reas a txt file in the format of printOnFileSystem() and it loads the content of that
* file in the object, overriding its original content
*
* @argument filename: the filename of the input file
*/
void Map::readFile(string filename) {
    std::fstream myfile(filename, std::ios_base::in);

    // get width and height
    myfile >> width;
    myfile >> height;

    // get the total number of shapes excluding the goal
    int totshapes;
    myfile >> totshapes;

    // get the center of the goal and then its 4 vertices
    float goal_center_x,goal_center_y;
    myfile >> goal_center_x;
    myfile >> goal_center_y;
    Point2f goal_center = Point2f(goal_center_x,goal_center_y);
    vector<Point2f> goal_vertices;
    // get the 4 vertices of the goal
    for( int i = 0 ; i < 4; i++) {
        float x,y;
        myfile >> x;
        myfile >> y;
        goal_vertices.push_back(Point2f(x,y));
    }
    goal = Obstacle(goal_center,goal_vertices);

    // Iterate until all the shapes have been read
    for(int j = 0; j < totshapes;j++) {
        // get the center of the shape and its number of vertices
        float center_x, center_y;
        myfile >> center_x;
        myfile >> center_y;
        Point2f center = Point2f(center_x,center_y);
        int n_vertices;
        myfile >> n_vertices;

        // if the number of vertices is 0 we are reading a circle
        if(n_vertices == 0) {
            int number;
            double radius;
            myfile >> radius;
            myfile >> number;
            Victim c = Victim(number,center,radius);
            victims.push_back(c);
        } else { // otherwise is a normal polygon
            vector<Point2f> vertices;

            // read every vertice of the shape
            for(int i = 0; i < n_vertices; i++) {
                float point_x;
                float point_y;
                myfile >> point_x;
                myfile >> point_y;
                Point2f p = Point2f(point_x,point_y);
                vertices.push_back(p);
            }
            Obstacle poli = Obstacle(center,vertices);
            obstacles.push_back(poli);
        }

    }
    myfile.close();
    sort(victims.begin(),victims.end());
}

// Resize the whole map
void Map::resize(float scale) {
    goal.resize(scale);

    for(unsigned int i = 0; i < obstacles.size(); i++) {
        obstacles[i].resize(scale);
    }

    for(unsigned int i = 0; i < victims.size(); i++) {
        victims[i].resize(scale);
    }

    width = width*scale;
    height = height*scale;

}
