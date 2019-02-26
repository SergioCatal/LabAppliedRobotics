#ifndef __CELL__
#define __CELL__

#include "Map.hpp"
#include "Plotter.hpp"
#include "Graph.hpp"

/**
 * Class that represents a cell in the line sweep alghoritm
 * 
 * it is composed by an opening ray(vertical line), a closing ray, two other segments, it remembers the circumstances
 * in wich the cell was open and close (open_case and close_case) and also the last poits of the previous cell and how many points
 * this cell will create in the graph
 */
class Cell {
private:
    int points_per_cell;
    Segment ray1;
    Segment seg1;
    Segment seg2;
    Segment ray2;
    int open_case; // 1 = polygon shape  2 = double open 3 = double close
    int close_case;
    vector<Point2f> last_points;

    /**
     * Method that connects the points in vec1 to the point2 in vec2 in the graph
     */
    void connectPoints(const vector<Point2f> &vec1, const vector<Point2f> &vec2, PointsGraph* pg) const;

    /**
     * Method to get the cell internal poins
     * 
     * @argument left_points : the empty vector where the left points will be added
     * @argument center_points : the empty vector where the central points will be added
     * @argument right_points : the empty vector where the right points will be added
     */
    void getInternalPoints(vector<Point2f> &left_points, vector<Point2f> &center_points, vector<Point2f> &right_points) const;

public:
    /**
     * Public constructor
     * 
     * @argument ray1 : the opening ray(vertical line)
     * @argument seg1 : one of the two segments that delimits the cell
     * @argument seg2 : the other segment delimiting the cell
     * @argument open_case : the case that lead to the open of the cell
     * @argument lastpoints : the last points of the previous cell
     * @argument cell_points : the number of points that this cell will put in the graph
     */
    Cell(Segment ray1,Segment seg1, Segment seg2,int open_case,vector<Point2f> lastpoints, int cell_points);

    /**
     * Method that will add the new points of this cell to the graph
     * 
     * @argument close_case : the case that lead to the closing of the cell
     * @argument pg : the graph of points
     */
    void completeGraph(PointsGraph &pg);

    /**
     * Method that returns the last points of this cell
     * 
     * @return : a vector containing the last points of this cell
     */
    vector<Point2f> getLastPoints();

    /**
     * Method that add the second ray to the cell, and it complete the graph with its point
     * 
     * @argument ray2 : the closing ray of the cell
     * @argument close_case : the case that lead to the closing of the cell
     * @argument pg : the graph of points
     */ 
    void addRay2(Segment ray2, int close_case,PointsGraph &pg);

    /**
     * Method that checks if this cell contains a segment
     * 
     * @argument s : the segment to check if it is in the cell
     * 
     * @return : true if the cell contains the segment, false otherwise
     */
    bool ContainsSegment(const Segment &s) const;

    /**
     * Method that prints the cell's information
     */
    void print() const;

    /**
     * Method that returns the first ray
     * 
     * @return : the first ray
     */
    Segment getRay1() const;

    /**
     * Method that returns the second ray
     * 
     * @return : the second ray
     */
    Segment getRay2() const;

    /**
     * Method that returns the first segment of the cell
     * 
     * @return : the first segment of the cell
     */
    Segment getSeg1() const;

    /**
     * Method that returns the second segment of the cell
     * 
     * @return : the second segment of the cell
     */
    Segment getSeg2() const;

    /**
     * Method that returns the open case of the cell
     * 
     * @return : the open case of the cell
     */
    int getOpenCase() const;

};





#endif
