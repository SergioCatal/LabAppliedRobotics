#include "Decomposer.hpp"
using namespace ClipperLib;
//#define SHOW_DECOMPOSER_STEPS

// Function that will prepare the map for the line sweep algorithm, clipping the obstacles, clipping the map, eliminating vertical lines and getting the new polyogns that do not overlaps
void Decomposer::prepareMap(float clippingQuantity, Map &m, vector<Obstacle> &non_overlapping_polygons){
  std::cout << "PREPARING " << std::endl;
    // clipping of the map and of the obstacles
    if(clippingQuantity > 0) {
        m.clipObstaclesEdges(clippingQuantity);
        std::cout << "PREPARING " << std::endl; 

        reshapeExitingPolygons(m,clippingQuantity);
    }
    std::cout << "PREPARING " << std::endl;

    eliminateVerticalLines(m); // eliminate the vertical lines
    std::cout << "PREPARING " << std::endl;

    // merge the overlpping polygons and create the non_overlapping_polygons vector
    eliminateOverlappingPolygons(m,non_overlapping_polygons);
}

// function that add a noise on the x value of every obstacle vertex
void Decomposer::eliminateVerticalLines(Map &m) {
    vector<Obstacle> new_obstacles = {};
    const vector<Obstacle> &old_obstacles = m.getObstacles();

    for(unsigned int i = 0; i < old_obstacles.size();i++) {
        vector<Point2f> old_vertices = old_obstacles[i].getVertices();
        vector<Point2f> new_vertices = {};

        for(unsigned int j = 0; j < old_vertices.size(); j++) {
            if(old_vertices[j].x != 0 && old_vertices[j].x != m.getWidth() ) {
                float noise = ((float)(rand() % 100)) / 1000 ; // add a random noise, at maximum 0.1
                Point2f new_point = Point2f (old_vertices[j].x + noise, old_vertices[j].y);
                new_vertices.push_back(new_point);
            } else {
                new_vertices.push_back(old_vertices[j]);
            }
        }

        Obstacle new_obstacle = Obstacle(old_obstacles[i].getCenter(),new_vertices);
        new_obstacles.push_back(new_obstacle);
    }

    m.setObstacles(new_obstacles);

}

// Function that clips the map and modify all the coordinates of the objects inside the map to be consistent
void Decomposer::reshapeExitingPolygons(Map &m,float offset) {
    float scale = 10000.0f; // the precision for every point
    #ifdef SHOW_DECOMPOSER_STEPS
        cout << "Reshaping exit polygons" << endl;
    #endif
    vector<Obstacle> &obstacles = m.getObstacles();
    float width = m.getWidth();
    float height = m.getHeight();

    float new_width = width - 2*offset;
    float new_height = height - 2*offset;

    BoundingBox map_limit = BoundingBox(0,0,width,height);
    map_limit.print();
    vector<Obstacle> new_obstacles = {};
;

    vector<Paths> all_obstacles = {};
    // put all the obastacles in the all_path Paths vector
    for(unsigned int i = 0; i < obstacles.size();i++) {
        vector<Point2f> vertices = obstacles[i].getVertices();
        Paths pTemp(1);
        for(unsigned int j = 0; j < vertices.size();j++) {
            int x = (vertices[j].x*scale);
            int y = (vertices[j].y*scale);
            pTemp[0] << IntPoint(x,y);
        }
        all_obstacles.push_back(pTemp);
    }

    Paths mapLimits(1); // create a new path that is the smaller map
    mapLimits[0] << IntPoint(offset*scale,offset*scale) << IntPoint(offset*scale,(height-offset)*scale)
    << IntPoint((width-offset)*scale,(height-offset)*scale) << IntPoint((width-offset)*scale,offset*scale);

    // intersecate the map limit with every obstacle in order to have new obstacles inside the new map
    for(unsigned int i = 0; i < all_obstacles.size();i++) {
        Clipper c;
        c.AddPaths(all_obstacles[i],ptSubject,true);
        c.AddPaths(mapLimits,ptClip,true);
        Paths solution;
        c.Execute(ctIntersection, solution, pftNonZero, pftNonZero);

        for(unsigned int j = 0; j < solution.size();j++) {
            vector<Point2f> tempPoints = {};
            for(unsigned int k = 0; k < solution[j].size();k++) {
                tempPoints.push_back(Point2f(solution[j][k].X/scale-offset,solution[j][k].Y/scale-offset));
            }
            Obstacle tempObstacle = Obstacle(Utility::computePointsCenter(tempPoints),tempPoints );
            new_obstacles.push_back(tempObstacle);
        }
    }

    vector<Victim> old_victims = m.getVictims();
    vector<Victim> new_victims = {};

    BoundingBox new_map_limits = BoundingBox(offset,offset,width-offset,height-offset);

    // modify every victim in order to have the center inside the new map
    for(unsigned int i = 0; i < old_victims.size();i++) {

        if( !(new_map_limits.isPointInside( old_victims[i].getCenter() ) ) ) {
            Point2f new_center = old_victims[i].getCenter();
            new_center.x = new_center.x-offset;
            new_center.y = new_center.y-offset;
            if(new_center.x < 0) {new_center.x = 1; }
            if(new_center.y < 0) {new_center.y = 1; }
            if(new_center.x > new_width) {new_center.x = new_width-1; }
            if(new_center.y > new_height) {new_center.y = new_height-1; }
            Victim v = Victim(old_victims[i].getNumber(),new_center,old_victims[i].getRadius());
            new_victims.push_back(v);

        } else {
            Point2f new_center = old_victims[i].getCenter();
            new_center.x = new_center.x-offset;
            new_center.y = new_center.y-offset;
            Victim v = Victim(old_victims[i].getNumber(),new_center,old_victims[i].getRadius());
            new_victims.push_back(v);
        }

    }

    // modify the goal in order to have it on the new map
    Obstacle old_goal = m.getGoal();
    vector<Point2f> old_goal_vertices = old_goal.getVertices();
    Point2f old_goal_center = old_goal.getCenter();
    for(unsigned int i = 0; i < old_goal_vertices.size();i++) {
        old_goal_vertices[i].x = old_goal_vertices[i].x - offset;
        old_goal_vertices[i].y = old_goal_vertices[i].y - offset;
        if(old_goal_vertices[i].x < 0) {old_goal_vertices[i].x=0; }
        if(old_goal_vertices[i].y < 0) {old_goal_vertices[i].y=0; }
        if(old_goal_vertices[i].x > new_width) {old_goal_vertices[i].x=new_width; }
        if(old_goal_vertices[i].y > new_height) {old_goal_vertices[i].y=new_height; }
    }
    old_goal_center.x = old_goal_center.x - offset;
    old_goal_center.y = old_goal_center.y - offset;
    float delta_x =0;
    float delta_y = 0;
    if(old_goal_center.x < 0) {
        delta_x = old_goal_center.x;
        old_goal_center.x=0.1;
    }
    if(old_goal_center.y < 0) {
        delta_y = old_goal_center.y;
        old_goal_center.y=0.1;
    }
    if(old_goal_center.x > new_width) {
        delta_x = old_goal_center.x - new_width;
        old_goal_center.x=new_width-0.1;
    }
    if(old_goal_center.y > new_height) {
        delta_y = old_goal_center.y - new_height;
        old_goal_center.y=new_height-0.1;
    }

    cout << "Delta real goal center :" << delta_x << " " << delta_y << endl;

    // create the new map
    m = Map(new_width,new_height,Obstacle(old_goal_center,old_goal_vertices),new_obstacles,new_victims);

}

// Function that creates a new list of polygons that do not overlap from a map
void Decomposer::eliminateOverlappingPolygons(Map &m,vector<Obstacle> &non_overlapping_polygons) {
    #ifdef SHOW_DECOMPOSER_STEPS
        cout << "eliminate overlapping polygons " << endl;
    #endif

    float scale = 10000.0f;
    non_overlapping_polygons.clear();
    vector<Paths> all_paths = {};
    vector<Obstacle> &obstacles = m.getObstacles();

    // put all the obastacles in the all_path Paths vector
    for(unsigned int i = 0; i < obstacles.size();i++) {
        Paths pTemp(1);
        vector<Point2f> vertices = obstacles[i].getVertices();

        for(unsigned int j = 0; j < vertices.size();j++) {
            int x = (vertices[j].x*scale);
            int y = (vertices[j].y*scale);
            pTemp[0] << IntPoint(x,y);
        }

        all_paths.push_back(pTemp);
    }

    Clipper c;
	// add all paths to the clipper
    for(unsigned int i = 0; i < all_paths.size();i++) {
        c.AddPaths(all_paths[i], ptSubject, true);
    }

    // compute new polygons
    Paths solution;
    c.Execute(ctUnion, solution, pftNonZero, pftNonZero);

    #ifdef SHOW_DECOMPOSER_STEPS
        cout << "Original obstacles: " << obstacles.size() << ", non overlapping solution " << solution.size() << endl;
    #endif

    // create the new obstacles
    for(unsigned int i = 0; i < solution.size();i++) {

        vector<Point2f> new_vertices = {};

        for(unsigned int j = 0; j < solution[i].size();j++) {
            new_vertices.push_back( Point2f( solution[i][j].X/scale , solution[i][j].Y/scale )  );
        }

        Point2f center = Utility::computePointsCenter(new_vertices);
        non_overlapping_polygons.push_back(Obstacle(center,new_vertices));
    }

}

// get the up collision from the collision map
Segment Decomposer::getUpCollision(const map<float,Segment> &collisions, float target, const Map &m) {

    bool alreadyfound = false;
    Segment upSeg = Segment();

    map<float, Segment>::const_iterator itr;
    for (itr = collisions.cbegin(); itr != collisions.cend(); ++itr) {

        if(itr->first >= target && alreadyfound == false) {
            alreadyfound = true;
            upSeg = itr->second;
        }
    }

    // if no segment is found in the map, return the top segment of the map
    if(!alreadyfound) {
        upSeg = Segment(Point2f(0,m.getHeight()),Point2f(m.getWidth(),m.getHeight() ) ) ;
    }

    return upSeg;
}

// get the down collision from the collision map
Segment Decomposer::getDownCollision(const map<float,Segment> &collisions, float target, const Map &m) {

    Segment downSeg = Segment();
    bool alreadyfound = false;

    map<float, Segment>::const_iterator itr;
    for (itr = collisions.cbegin(); itr != collisions.cend(); ++itr) { 

        if(itr->first <= target) {
            downSeg = itr->second;
            alreadyfound = true;
        }
    }
    // if no segment is found in the map, return the bottom segment of the map
    if(!alreadyfound) {
        downSeg = Segment(Point2f(0,0),Point2f(m.getWidth(),0) ) ;
    }

    return downSeg;
}


// main decompose function
void Decomposer::decompose(PointsGraph &pg, Map m,vector<Obstacle> non_overlapping_polygons,int points_per_cell) {
    // get map information
    float width = m.getWidth();
    float height = m.getHeight();
    BoundingBox map_limit = BoundingBox(0,0,width,height);

    // get all the vertices of the non overlapping polygons in one vector
    vector<Point2f> all_vertices = {};
    vector<Obstacle> obstacles = m.getObstacles();

    for(unsigned int i = 0; i < non_overlapping_polygons.size(); i++){
        Obstacle current_obstacle = non_overlapping_polygons[i];
        vector<Point2f> current_vertices = current_obstacle.getVertices();

        for(unsigned int j = 0; j < current_vertices.size();j++) {
            all_vertices.push_back( current_vertices[j]);
        }
    }

    // sort the vertices
    std::sort (all_vertices.begin(), all_vertices.end(), Utility::greaterXPoint2f);

    #ifdef SHOW_DECOMPOSER_STEPS
        cout << "SORTED VECTOR : "; // print the sorted vertices vector
    for(unsigned int i = 0; i < all_vertices.size();i++) {
        cout << " [";
        printf("%4.4f",all_vertices[i].x);
        cout << "," ;
        printf("%4.4f",all_vertices[i].y);
        cout << "] ";
    } cout << endl;
    #endif


    // get all the segments from the non overlapping polygons
    vector<Segment> all_segments = {};
    for(unsigned int i = 0; i < non_overlapping_polygons.size(); i++) {
        Obstacle current_obstacle = non_overlapping_polygons[i];
        vector<Segment> current_segments = current_obstacle.getSegments();
        for(unsigned int j = 0; j < current_segments.size() ; j++) {
            all_segments.push_back( current_segments[j]);
        }
    }

    // create initial edges (the edges of the map)
    Segment left_edge = Segment(Point2f(0,0),Point2f(0,m.getHeight()));
    Segment right_edge = Segment(Point2f(m.getWidth(),m.getHeight()),Point2f(m.getWidth(),0));
    Segment top_edge = Segment(Point2f(0,m.getHeight()),Point2f(m.getWidth(),m.getHeight()));
    Segment bottom_edge = Segment(Point2f(0,0),Point2f(m.getWidth(),0));

    // put the edges of the map in the vector containing all the segments
    all_segments.push_back( left_edge );
    all_segments.push_back(  top_edge);
    all_segments.push_back( bottom_edge );
    all_segments.push_back( right_edge );

    // cells contains all the cells currently opened
    vector<Cell> cells;
    // the initial cell is the one that has the left edge as a ray and the tob an bottom edges
    vector<Point2f> aa = {};
    Cell initial_cell = Cell(left_edge,bottom_edge,top_edge, 1,aa,points_per_cell);
    cells.push_back(initial_cell);

    // we now have a vector af all the vertexs sorted, a vector with all the segments(with also the map borders) and the bounding box of the map

    for(unsigned int i = 0; i < all_vertices.size();i++) {

        // create the ray, a vertical segment that spans all the map with the x coordinate of the current vertex
        Segment ray = Segment(Point2f(all_vertices[i].x,0), Point2f(all_vertices[i].x,m.getHeight()));

        map<float, Segment> collisions;
        // save how many segments that intersects
        vector<Segment> segments_after = {};
        vector<int> segments_after_pos = {};
        vector<Segment> segments_before = {};

        for(unsigned int j = 0; j < all_segments.size();j++) {

            // the current segment is colliding with the ray
            if(all_segments[j].isCollidingWithSegment(ray)) {
                Point2f collision = all_segments[j].getCollisionPointWithSegment(ray);

                // if the collision correspond to the first point of the segment
                if( abs(collision.x-all_segments[j].getP1().x) < 0.001  && abs(collision.y-all_segments[j].getP1().y) < 0.001 ) {

                    if(all_segments[j].getP2().x > collision.x) { // this segment is after the ray (it starts with the current collision)
                        segments_after.push_back(all_segments[j]);
                        segments_after_pos.push_back(1);
                    } else {
                        segments_before.push_back(all_segments[j]); // this segment finishes with the current vertex
                    }
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "The last collision "  << collision << " was a vertex of the segment" ; all_segments[j].print();
                    #endif

                // if the collision correspond to the last point of the segment
                } else if ( abs(collision.x-all_segments[j].getP2().x) < 0.001  && abs(collision.y-all_segments[j].getP2().y) < 0.001 ) {

                    if(all_segments[j].getP1().x > collision.x) { // this segment is after the ray
                        segments_after.push_back(all_segments[j]);
                        segments_after_pos.push_back(2);
                    } else {
                        segments_before.push_back(all_segments[j]); // this segment finishes with the collision
                    }
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "The last collision "  << collision << " was a vertex of the segment" ; all_segments[j].print();
                    #endif
                } else {
                    // a blu circle represents a collision with a non vertex of a segment
                    // all these collision are saved in a map based on the y-coordinate of the collision
                    collisions.insert( pair<float,Segment>(collision.y,all_segments[j]) );
                }
            }
        }

        // now we have the vector of segments that finish with this point and starts with this point, and a map with all the collision with other segments ordered by y-coordinate
        // CASE 1: one segment after and 1 before
        if(segments_before.size() == 1 && segments_after.size() == 1) {
            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "CASE 1 one open one close" << endl;
                cout << "Segment before: "; segments_before[0].print();
                cout << "Segment after: "; segments_after[0].print();
                cout << "Down coll. segment: "; Decomposer::getDownCollision(collisions,all_vertices[i].y,m).print();
                cout << "Up coll. segment: "; Decomposer::getUpCollision(collisions,all_vertices[i].y,m).print();
            #endif

            // eliminate the cell that contains one the segment that ends with this point
            int cell_to_eliminate = -1;
            for(unsigned int k = 0; k < cells.size();k++) {

                if(cells[k].ContainsSegment(segments_before[0] ) ) {
                    cell_to_eliminate = k;
                }
            }
            vector<Point2f> last_point = {};
            // get the last points of the cell and eliminate it
            if(cell_to_eliminate == -1) {
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "wrong cell to eliminate!!!" << endl << endl;
                    #endif
            } else {
                cells[cell_to_eliminate].addRay2(ray,1,pg);
                last_point = cells[cell_to_eliminate].getLastPoints();
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "Eliminated cell: ";
                    cells[cell_to_eliminate].print();
                #endif
                cells.erase(cells.begin() + cell_to_eliminate);
            }

            // try to understand if the cell to open is on top or on the bottom
            Point2f bottomPoint = Point2f(all_vertices[i].x,all_vertices[i].y-0.1);
            Point2f topPoint = Point2f(all_vertices[i].x,all_vertices[i].y+0.1);
            bool openbottom = true;
            bool opentop = true;

            for(unsigned int k = 0; k < obstacles.size();k++) {
                if(obstacles[k].isPointInside(bottomPoint)) {
                    openbottom = false;
                }
                if(obstacles[k].isPointInside(topPoint)) {
                    opentop = false;
                }
            }

            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "openbottom: " << openbottom << endl;
            #endif
            if(openbottom ) {
                if(map_limit.isPointInside(bottomPoint)) { // if we have to open on the bottom and the point is inside the map open the bottom cell
                    Cell newCell = Cell(ray, segments_after[0],Decomposer::getDownCollision(collisions,all_vertices[i].y,m),1,last_point,points_per_cell);
                    cells.push_back(newCell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "cell bottom created: ";
                        newCell.print();
                    #endif
                } else { // if we have to open on the bottom and the point is not inside the map and if nextpoint is not inside any object we are in the special case for the opening in the map limit
                    Point2f nextpoint = Point2f(all_vertices[i].x+0.01,all_vertices[i].y+0.01);
                    bool openNextPoint = true;
                    for(unsigned int k = 0; k < obstacles.size();k++) {
                        if(obstacles[k].isPointInside(nextpoint)) {
                            openNextPoint = false;
                        }
                    }
                    if(openNextPoint) {
                        Cell newCell = Cell(ray, segments_after[0],Decomposer::getUpCollision(collisions,all_vertices[i].y,m),1,last_point,points_per_cell);
                        cells.push_back(newCell);
                        #ifdef SHOW_DECOMPOSER_STEPS
                            cout << "cell bottom special created: ";
                            newCell.print();
                        #endif
                    }
                }

            } else if(opentop ){
                if(map_limit.isPointInside(topPoint)) { // if we have to open on the top and the point is inside the map open the top cell
                    Cell newCell = Cell(ray, segments_after[0],Decomposer::getUpCollision(collisions,all_vertices[i].y,m),1,last_point,points_per_cell);
                    cells.push_back(newCell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "cell top created: ";
                        newCell.print();
                    #endif
                } else { // if we have to open on the top and the point is not inside the map and if nextpoint is not inside any object we are in the special case for the opening in the map limit
                    Point2f nextpoint = Point2f(all_vertices[i].x+0.01,all_vertices[i].y-0.01);
                    bool openNextPoint = true;
                    for(unsigned int k = 0; k < obstacles.size();k++) {
                        if(obstacles[k].isPointInside(nextpoint)) {
                            openNextPoint = false;
                        }
                    }
                    if(openNextPoint) {
                        Cell newCell = Cell(ray, segments_after[0],Decomposer::getDownCollision(collisions,all_vertices[i].y,m),1,last_point,points_per_cell);
                        cells.push_back(newCell);
                        #ifdef SHOW_DECOMPOSER_STEPS
                            cout << "cell bottom special created: ";
                            newCell.print();
                        #endif
                    }
                }

            }

        // Second case : two segments after and no one before
        }else if(segments_before.size() == 0 && segments_after.size() == 2) {
            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "CASE 2 double open" << endl;
                cout << "Segment after1: "; segments_after[0].print();
                cout << "Segment after2: "; segments_after[1].print();
                cout << "Down coll. segment: "; Decomposer::getDownCollision(collisions,all_vertices[i].y,m).print();
                cout << "Up coll. segment: "; Decomposer::getUpCollision(collisions,all_vertices[i].y,m).print();
            #endif

            int cell_to_eliminate = -1;
            // close old cell
            for(unsigned int k = 0; k < cells.size();k++) {

                if(cells[k].ContainsSegment(Decomposer::getDownCollision(collisions,all_vertices[i].y,m)) &&
                cells[k].ContainsSegment(Decomposer::getUpCollision(collisions,all_vertices[i].y,m))) {
                    cell_to_eliminate = k;
                }

            }
            vector<Point2f> last_point = {};
            if(cell_to_eliminate == -1) {
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "wrong cell to eliminate!!!" << endl << endl << endl;
                #endif
            } else {
                cells[cell_to_eliminate].addRay2(ray,1,pg);
                last_point= cells[cell_to_eliminate].getLastPoints();
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "Cell elimintated: ";
                    cells[cell_to_eliminate].print();
                #endif
                cells.erase(cells.begin() + cell_to_eliminate);
            }

            // uderstand wich of the upcoming segments is in on top of the other
            Segment downSide = Segment();
            Segment upSide = Segment();

            if(segments_after_pos[0] == 1) {
                if(segments_after_pos[1] == 1) { // both segments first point collides with the ray, we have to confront their second point

                    if((segments_after[0].getP2().y - segments_after[1].getP2().y ) > 0.01) {
                        downSide = segments_after[1];
                        upSide = segments_after[0];
                    } else {
                        downSide = segments_after[0];
                        upSide = segments_after[1];
                    }
                } else {
                    if((segments_after[0].getP2().y - segments_after[1].getP1().y ) > 0.01) {
                        downSide = segments_after[1];
                        upSide = segments_after[0];
                    } else {
                        downSide = segments_after[0];
                        upSide = segments_after[1];
                    }
                }


            } else {
                if(segments_after_pos[1] == 1) {
                    if((segments_after[0].getP1().y - segments_after[1].getP2().y ) > 0.01) {
                        downSide = segments_after[1];
                        upSide = segments_after[0];
                    } else {
                        downSide = segments_after[0];
                        upSide = segments_after[1];
                    }
                } else {
                    if((segments_after[0].getP1().y - segments_after[1].getP1().y ) > 0.01) {
                        downSide = segments_after[1];
                        upSide = segments_after[0];
                    } else {
                        downSide = segments_after[0];
                        upSide = segments_after[1];
                    }
                }
            }

            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "downside: "; downSide.print();
                cout << "upside: "; upSide.print();
            #endif

            Point2f bottomPoint = Point2f(all_vertices[i].x,all_vertices[i].y-0.1);
            Point2f topPoint = Point2f(all_vertices[i].x,all_vertices[i].y+0.1);
            bool openbottom = false;
            bool opentop = false;

            if(map_limit.isPointInside(bottomPoint)) {
                openbottom = true;
            }
            if(map_limit.isPointInside(topPoint)) {
                opentop = true;
            }

            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "openbottom " << openbottom << " opentop " << opentop << endl;
            #endif

            if(openbottom && opentop) { 
                bool prevPointInsideObstacle = false;
                Point2f prevPoint = Point2f(all_vertices[i].x-0.1,all_vertices[i].y);
                for(unsigned int k = 0; k < obstacles.size();k++) {
                    if(obstacles[k].isPointInside(prevPoint)) { // before was bottom point here
                        prevPointInsideObstacle = true;
                    }
                }

                // case of a non convex obstacle
                if(prevPointInsideObstacle) {
                    openbottom= false;
                    opentop = false;
                    Cell new_Cell = Cell(ray,upSide,downSide,2,last_point,points_per_cell);
                    cells.push_back(new_Cell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "open strange cell ";
                        new_Cell.print();
                    #endif
                }
            }

            if(openbottom) {
                Cell bottom = Cell(ray, downSide, Decomposer::getDownCollision(collisions,all_vertices[i].y-0.1,m),2,last_point,points_per_cell);
                cells.push_back(bottom);
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "open bottom ";
                    bottom.print();
                #endif
            }
            if(opentop) {
                Cell bottom = Cell(ray, upSide, Decomposer::getUpCollision(collisions,all_vertices[i].y+0.1,m),2,last_point,points_per_cell);
                cells.push_back(bottom);
                #ifdef SHOW_DECOMPOSER_STEPS
                    cout << "open top ";
                    bottom.print();
                #endif
            }

            // thid case: two segments before and one after 
        }else if(segments_before.size() == 2 && segments_after.size() == 0) {
            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "CASE 3 double close" << endl;
                cout << "Segment before1: "; segments_before[0].print();
                cout << "Segment before2: "; segments_before[1].print();
                cout << "Down coll. segment: "; Decomposer::getDownCollision(collisions,all_vertices[i].y,m).print();
                cout << "Up coll. segment: "; Decomposer::getUpCollision(collisions,all_vertices[i].y,m).print();
            #endif

            vector<Point2f> last_points = {};

            // eliminate the cells to close
            for(unsigned int k = 0; k < cells.size();k++) {

                if(cells[k].ContainsSegment(segments_before[1]) || cells[k].ContainsSegment(segments_before[0])) {
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "cell eliminted: ";
                        cells[k].print();
                    #endif
                    cells[k].addRay2(ray,3,pg);
                    vector<Point2f> points_to_insert = cells[k].getLastPoints();
                    last_points.insert(last_points.end(),points_to_insert.begin(),points_to_insert.end());
                    cells.erase(cells.begin() + k);
                    k = k - 1;
                }

            }

            Point2f nextPoint = Point2f(all_vertices[i].x+0.1,all_vertices[i].y);
            bool nextPointInsideObstacle = false;
            for(unsigned int k = 0; k < obstacles.size();k++) {
                if(obstacles[k].isPointInside(nextPoint)) {
                    nextPointInsideObstacle = true;
                }
            }

            #ifdef SHOW_DECOMPOSER_STEPS
                cout << " nextPointinsideObstacle " << nextPointInsideObstacle << endl;
            #endif

            if(nextPointInsideObstacle) {   // do no open anything

            } else {

                Point2f bottomPoint = Point2f(all_vertices[i].x,all_vertices[i].y-0.1);
                Point2f topPoint = Point2f(all_vertices[i].x,all_vertices[i].y+0.1);
                bool openbottom = false;
                bool opentop = false;

                if(map_limit.isPointInside(bottomPoint)) {
                    openbottom = true;
                }
                if(map_limit.isPointInside(topPoint)) {
                    opentop=true;
                }

                if(openbottom && opentop) { // both top and bttom are inside the map, so open a new cell
                    Cell newCell = Cell(ray, Decomposer::getDownCollision(collisions,all_vertices[i].y,m),Decomposer::getUpCollision(collisions,all_vertices[i].y,m),3,last_points,points_per_cell);
                    cells.push_back(newCell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "Cell created :"; newCell.print();
                    #endif
                } else if(openbottom) { // only one of the two point is inside the map, so open the relative cell
                    Cell newCell = Cell(ray, Decomposer::getDownCollision(collisions,bottomPoint.y,m),Decomposer::getUpCollision(collisions,bottomPoint.y,m),3,last_points,points_per_cell);
                    cells.push_back(newCell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "Cell created with bottom point:"; newCell.print();
                    #endif
                } else {
                    Cell newCell = Cell(ray, Decomposer::getDownCollision(collisions,topPoint.y,m),Decomposer::getUpCollision(collisions,topPoint.y,m),3,last_points,points_per_cell);
                    cells.push_back(newCell);
                    #ifdef SHOW_DECOMPOSER_STEPS
                        cout << "Cell created with top point:"; newCell.print();
                    #endif
                }

            }



        } else {
            #ifdef SHOW_DECOMPOSER_STEPS
                cout << "NOT RECOGNIZED CASE" << endl;
                cout << "segments before: " << endl;
                for(unsigned int k = 0; k < segments_before.size();k++) {
                    segments_before[k].print();
                }
                cout << "segments after: " << endl;
                for(unsigned int k = 0; k < segments_after.size();k++) {
                    segments_after[k].print();
                }
            #endif
        }
        #ifdef SHOW_DECOMPOSER_STEPS
        #endif

    }

    #ifdef SHOW_DECOMPOSER_STEPS
        cout << "number remainings cells : " << cells.size() << endl;
    #endif

    // close all the remaining cells, if the initial cell is still open and it is not the only remaining cell do not close it
    for(unsigned int i = 0; i < cells.size();i++) {

        if(cells[i].ContainsSegment(left_edge) && cells[i].ContainsSegment(bottom_edge) && cells[i].ContainsSegment(top_edge) && cells.size() > 1) {
        } else {
            cells[i].addRay2(right_edge,3,pg);
        }

    }

    #ifdef SHOW_DECOMPOSER_STEPS
    #endif
}
