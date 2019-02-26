#ifndef __COLLISION_DETECTOR__
#define __COLLISION_DETECTOR__

#include "Map.hpp"
#include "AABBTree.hpp"
#include "DubinsCurve.hpp"
#include "DubinsSolver.hpp"
using namespace std;
#define VECTOR_COLLISION

class CollisionDetector {
private:
    vector<Obstacle> obstacles;
    BoundingBox map_limit;
    //AABBTree tree;

public:
    /**
     * Empty constructor
     */
    CollisionDetector();

    /**
     * Full constructor
     *
     * @argument obstacles : the list of polygons that represents the obstacles
     */
    CollisionDetector(Map m);

    /**
     * Function that returns true is a dubins arc collide with one of the obstacles
     *
     * @argument da : the dubins arc to check
     *
     * @return : true is there is a collision, false otherwise
     */
    bool collisionWithDubinsArc(const DubinsArc &da) const;

    /**
     * Function that returns true is a dubins curve collide with one of the obstacles
     *
     * @argument dc : the dubins curve to check
     *
     * @return : true is there is a collision, false otherwise
     */
    bool collisionWithDubinsCurve(const DubinsCurve &dc) const;

    /**
     * Function that given the initial and final configuration of the robot (x,y,angle) and the curvature of steering
     * computer the shortest dubins curve without collisions from the initial configuration to the final configuration without
     *
     *
     * @return : the collision free dubins curve from the initial to the final configuration
     */
    bool pathWithoutCollisions(float x0, float y0, float th0, float xf, float yf, float thf, float k,DubinsCurve &dc) const;

    /**
     * Method that return the obstacles of the collision detector
     * 
     * @return : the vector containing the obstacles
     */
    vector<Obstacle> getObstacles() const;

    /**
     * Method that returns the bounding box representing the map limits
     * 
     * @return : the bounding box representing the map limits
     */
    BoundingBox getMapLimits() const;

};


#endif
