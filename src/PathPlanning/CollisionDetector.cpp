#include "CollisionDetector.hpp"
#include <chrono>
#include <fstream>

#ifdef TREE_COLLISION
    std::ofstream out("performances.txt");
#endif

// Empty constructor
CollisionDetector::CollisionDetector() {
    obstacles = {};
    map_limit = BoundingBox();
}

// Full constructor
CollisionDetector::CollisionDetector(Map m) /*: tree(m.getObstacles())*/{
  auto start = std::chrono::high_resolution_clock::now();
  obstacles = m.getObstacles();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> vector_setup = finish - start;

  #ifdef TREE_COLLISION
    start = std::chrono::high_resolution_clock::now();
    AABBTree tree(m.getObstacles());
    finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> tree_setup = finish - start;
    out << "\t\tSETUP TIME\nVector: " << vector_setup.count() << "\nTree: " << tree_setup.count() <<std::endl;
  #endif

  map_limit = BoundingBox(0,0,m.getWidth(),m.getHeight() );
}

// Collision with dubins curve
bool CollisionDetector::collisionWithDubinsCurve(const DubinsCurve &dc) const {
  bool ret = false;

  #ifdef VECTOR_COLLISION
    auto start = std::chrono::high_resolution_clock::now();
    vector<Obstacle> collidingObstacles = {};

    // First we check the bounding box of every obstacle
    for(unsigned int i = 0; i < obstacles.size(); i++) {
        Obstacle poly = obstacles[i];

        if(poly.isDubinsCurveCollidingWithBoundingBox(dc) ) {
            //cout << "Bounding box of polygon " << i << " is colliding with the curve" << endl;
            collidingObstacles.push_back(poly);
        }

    }

    // Then for the obstacle that have a colliding bounding box we do a more precise collision detection
    for(unsigned int i = 0; i < collidingObstacles.size();i++) {

        Obstacle poly = collidingObstacles[i];

        if(poly.isDubinsCurveColliding(dc)) {
            //cout << "Polygon " << i << " is colliding with the curve" << endl;
            ret = true;
            break;
        }
    }
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> vector_collision_time = finish - start;
    
  #else
    out << "\t\tCOLLISION CHECK\nVector: " << ret << " in " << vector_collision_time.count() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    ret = tree.check_collision(dc);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> tree_collision_time = finish - start;
    out << "\t\tCOLLISION CHECK\nTree: " << ret << " in  " << tree_collision_time.count() << std::endl;
  #endif

  bool map_limit_collision = ( map_limit.isDubinsArcTouching(dc.getArc(0)) || map_limit.isDubinsArcTouching(dc.getArc(1)) || map_limit.isDubinsArcTouching(dc.getArc(2)) );
  //cout << "map limit collision: " << map_limit_collision << endl;

  return (ret || map_limit_collision);
}

// Collision with dubins arc
bool CollisionDetector::collisionWithDubinsArc(const DubinsArc &da) const{

    vector<Obstacle> collidingObstacles = {};

    // First we check the bounding box of every obstacle
    for(unsigned int i = 0; i < obstacles.size(); i++) {

        Obstacle poly = obstacles[i];

        if(poly.isDubinsArcCollidingWithBoundingBox(da) ) {
            //cout << "Bounding box of polygon " << i << " is colliding with the arc" << endl;
            collidingObstacles.push_back(poly);
        }

    } 

    // Then for the obstacle that have a colliding bounding box we do a more precise collision detection
    for(unsigned int i = 0; i < collidingObstacles.size();i++) {

        Obstacle poly = collidingObstacles[i];

        if(poly.isDubinsArcColliding(da)) {
            //cout << "Polygon " << i << " is colliding with the arc" << endl;
            return true;
        }
    }

    return map_limit.isDubinsArcTouching(da);
}

// Method that returns a path from start to finish without collsions
bool CollisionDetector::pathWithoutCollisions(float x0, float y0, float th0, float xf, float yf, float thf, float k,DubinsCurve &dc) const {

    DubinsCurve *results[6];
    int shortest_path_index = DubinsSolver::shortestPath(x0,y0,th0,xf,yf,thf, k,results);
    DubinsCurve shortest_path = *(results[shortest_path_index]);

    if( !( this->collisionWithDubinsCurve(shortest_path) ) ) {
        dc = shortest_path;
        DubinsSolver::freeCurves(results);
        //cout << "Correct best curve found" << endl;
        return true;
    } else {

        
        //cout << "the best solution is not collision free" << endl;
        std::map<float,DubinsCurve> mappedCurves;

        for(unsigned int i = 0; i < 6; i++) {
            if((int)i != shortest_path_index && results[i] != NULL) {
                DubinsCurve current = *(results[i]);
                float current_lenght = current.getTotalLenght();
                mappedCurves.insert(std::pair<float,DubinsCurve>(current_lenght, current) ) ;
                //cout << "element inserted in the map"<< endl;
            }
        }

        map<float, DubinsCurve>::iterator it;

        for ( it = mappedCurves.begin(); it != mappedCurves.end(); it++ ){
            DubinsCurve current = it->second;
            if(!(this->collisionWithDubinsCurve(current))) {

                //cout << "Correct other curve found" << endl;
                dc = current;
                DubinsSolver::freeCurves(results);
                return true;
            }
        }

    }

    DubinsSolver::freeCurves(results);
 
    return false;
}

// return the obstacles
vector<Obstacle> CollisionDetector::getObstacles() const {
    return obstacles; 
}

// return the bounding box representing the map limits
BoundingBox CollisionDetector::getMapLimits() const{
    return map_limit;
}
