#include "Cell.hpp"

// Public constructor
Cell::Cell(Segment ray1,Segment seg1, Segment seg2,int open_case,vector<Point2f> lastpoints, int cell_points) {
    this->ray1 = ray1;
    this->seg1 = seg1;
    this->seg2 = seg2;
    this->ray2 = Segment();
    this->open_case = open_case;
    this->close_case = -1;
    this->last_points = lastpoints;
    this->points_per_cell = cell_points;
}

// check if the cell contains a segment
bool Cell::ContainsSegment(const Segment &s) const { 

    if(ray1 == s || seg1 == s || seg2 == s || ray2 == s) {
        return true;
    } else {
        return false;
    }

}

// get ray1 method
Segment Cell::getRay1() const {
    return ray1;
}

// get ray2 method
Segment Cell::getRay2() const {
    return ray2;
}

// get seg1 method
Segment Cell::getSeg1() const {
    return seg1;
}

// get seg2 method
Segment Cell::getSeg2() const {
    return seg2;
}

// get open_case method
int Cell::getOpenCase() const {
    return open_case;
}

// print method
void Cell::print() const {
    cout << "Cell composed by ray1: " ;
    ray1.print();
    cout << " seg1 : ";
    seg1.print();
    cout << " seg2 : ";
    seg2.print();
    cout << "ray2 : ";
    ray2.print();

}

// add the second ray and complete the graph
void Cell::addRay2(Segment ray2,int close_case,PointsGraph &pg) {
    this->close_case = close_case;
    this->ray2 = ray2;
    this->completeGraph(pg);
}

// method to get the internal points of the cell
void Cell::getInternalPoints(vector<Point2f> &left_points, vector<Point2f> &center_points, vector<Point2f> &right_points) const {
    vector<Point2f> vertices = {};

    Point2f p1 = ray1.getCollisionPointWithSegment(seg1);
    Point2f p2 = ray1.getCollisionPointWithSegment(seg2);
    Point2f p3 = ray2.getCollisionPointWithSegment(seg1);
    Point2f p4 = ray2.getCollisionPointWithSegment(seg2);

    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
    vertices.push_back(p4);

    float center_x = (p1.x+p2.x+p3.x+p4.x)/4;

    Point2f temp1 = Point2f(center_x, (p3.y+p1.y)/2 );
    Point2f temp2 = Point2f(center_x, (p4.y+p2.y)/2 );

    int n_points = points_per_cell+1;

    for(int i = 1; i < n_points;i++) {
        float temp_l_y = ((n_points-i)*p1.y + i*p2.y )/(n_points);
        Point2f temp_l_point = Point2f(p1.x,temp_l_y);
        left_points.push_back(temp_l_point);

        float temp_c_y = ((n_points-i)*temp1.y + i*temp2.y )/(n_points);
        Point2f temp_c_point = Point2f(temp1.x,temp_c_y);
        center_points.push_back(temp_c_point);

        float temp_r_y = ((n_points-i)*p3.y + i*p4.y )/(n_points);
        Point2f temp_r_point = Point2f(p3.x,temp_r_y);
        right_points.push_back(temp_r_point);
    }
}

// method to get the last points of the cell
vector<Point2f> Cell::getLastPoints() {

    vector<Point2f> left_points;
    vector<Point2f> center_points;
    vector<Point2f> right_points;

    this->getInternalPoints(left_points,center_points,right_points);

    if(open_case == 1) {

        if(close_case == 1) {
            return center_points;
        } else if(close_case == 2) {
            return center_points;
        } else {
            return right_points;
        }

    } else if( open_case == 2) {

        if(close_case == 1) {
            return center_points;
        } else if(close_case == 2) {
            return center_points;
        } else {
            return right_points;
        }

    } else {

        if(close_case == 1) {
            return center_points;
        } else if(close_case == 2) {
            return center_points;
        } else {
            return right_points;
        }

    }

}

// method to add the point of this cell to the graph
void Cell::completeGraph(PointsGraph &pg) {
    vector<Point2f> left_points;
    vector<Point2f> center_points;
    vector<Point2f> right_points;

    this->getInternalPoints(left_points,center_points,right_points);

    /*
    We have two information: the cause of the opening of the cell and the cause of the closing of the cell
    We have three case : case shape of polygon, case double open, case double close.
    Therefore we have nine possibilities(first the open cause and then the close cause): 11, 12, 13, 21, 22, 23, 31, 32, 33.
    */

    if(open_case == 1) {

        if(close_case == 1 ) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);

        } else if(close_case == 2) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);
        } else if(close_case == 3) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);
            this->connectPoints(center_points,right_points,&pg);
        }

    } else if( open_case == 2) {

        if(close_case == 1 ) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);
        } else if(close_case == 2 ) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);
        } else if(close_case == 3) {
            this->connectPoints(last_points,left_points,&pg);
            this->connectPoints(left_points,center_points,&pg);
            this->connectPoints(center_points,right_points,&pg); 
        }

    } else if( open_case == 3) {

        if(close_case == 1 ) {
            this->connectPoints(last_points,center_points,&pg);
        } else if(close_case == 2 ) {
            this->connectPoints(last_points,center_points,&pg);
        } else if(close_case == 3) {
            this->connectPoints(last_points,center_points,&pg);
            this->connectPoints(center_points,right_points,&pg);
        }

    }

}

// Method to add the points in vec1 and vec2 connected in the graph
void Cell::connectPoints(const vector<Point2f> &vec1, const vector<Point2f> &vec2, PointsGraph* pg) const {


    for(unsigned int i = 0; i < vec1.size(); i++) {

        for(unsigned int j = 0; j < vec2.size(); j++) {

            pg->addEdge(vec1[i],vec2[j]);

        }

    }
}
