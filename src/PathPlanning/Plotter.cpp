#include "Plotter.hpp"
using namespace ClipperLib;

// Empty constructor
Plotter::Plotter() {
    circle_thickness = 3;
    polygon_thickness = 3;
    line_thickness = 3;
    background_color = Scalar(235,235,235);
    img = Mat(1024, 1024, CV_8UC3, background_color);
    default_poly_color = Scalar(0,0,0);
    default_circle_color = Scalar(0,235,0);
    default_line_color = Scalar(0,125,250);
}

// Partial constructor
Plotter::Plotter(int rows, int culumns) {
    circle_thickness = 3;
    polygon_thickness = 3;
    line_thickness = 3;
    background_color = Scalar(235,235,235);
    img = Mat(rows, culumns, CV_8UC3, background_color);
    default_poly_color = Scalar(0,0,0);
    default_circle_color = Scalar(0,235,0);
    default_line_color = Scalar(0,125,250);
}

// Full constructor
Plotter::Plotter(int rows, int columns, Scalar color) {
    img = Mat(rows, columns, CV_8UC3, color);
}

// Show method
void Plotter::show() {
    imshow("PLOTTER",img);
    waitKey(0);
}

// Get method for the internal image
Mat Plotter::getImg() {
    return img;
}

// Method that draws an obstacle
void Plotter::drawObstacle(const Obstacle &o) {
  vector<Point2f> vertices2f = o.getVertices();
  vector<Point> vertices = Utility::fromFloatPointsToIntPoints(vertices2f);
  vector<vector<Point>> vertices_to_draw = {vertices};
  drawContours(img,vertices_to_draw , -1, default_poly_color, polygon_thickness, LINE_AA);
  //this->drawBoundingBox(o.getBoundingBox());
}

// Method that draws a colored obstacle
void Plotter::drawObstacle(const Obstacle &o, Scalar color) {
  vector<Point2f> vertices2f = o.getVertices();
  vector<Point> vertices = Utility::fromFloatPointsToIntPoints(vertices2f);
  vector<vector<Point>> vertices_to_draw = {vertices};
  drawContours(img,vertices_to_draw , -1, color, polygon_thickness, LINE_AA);
  //this->drawBoundingBox(o.getBoundingBox());
}

// Method that draws a vector of obstacles
void Plotter::drawObstacles(const vector<Obstacle> &obstacles) {
  // for every obstacle
  for(unsigned int i = 0; i < obstacles.size();i++) {
    this->drawObstacle(obstacles[i]);
  }
  this->show();
}

// Method that draws a polygon
void Plotter::drawPolygon(const Polygon &p) {
  vector<Point2f> vertices2f = p.getVertices();
  vector<Point> vertices = Utility::fromFloatPointsToIntPoints(vertices2f);
  vector<vector<Point>> vertices_to_draw = {vertices};
  drawContours(img,vertices_to_draw , -1, default_poly_color, polygon_thickness, LINE_AA);
}

// Method that draws a bounding box
void Plotter::drawBoundingBox(const BoundingBox &bb) {
  Point p1 = Point(bb.getMinX(), bb.getMinY());
  Point p2 = Point(bb.getMaxX(),bb.getMinY());
  Point p3 = Point(bb.getMaxX(), bb.getMaxY());
  Point p4 = Point(bb.getMinX(), bb.getMaxY());

  vector<Point> vertices = {};
  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p3);
  vertices.push_back(p4);

  vector<vector<Point>> vertices_to_draw = {vertices};
  drawContours(img,vertices_to_draw , -1, default_poly_color, polygon_thickness, LINE_4);
}

// Method that draws a circle
void Plotter::drawCircle(const Circle &c) {
  Point2f center2f = c.getCenter();
  Point center = Utility::fromFloatPointToIntPoint(center2f);
  circle(img,center,c.getRadius(),default_circle_color,circle_thickness);
}

// draw a circle with color
void Plotter::drawCircle(const Circle &c,Scalar color) {
  Point2f center2f = c.getCenter();
  Point center = Utility::fromFloatPointToIntPoint(center2f);
  circle(img,center,c.getRadius(),color,circle_thickness);
}

// Method that draws a victim
void Plotter::drawVictim(const Victim &v) {
  Point2f center2f = v.getCenter();
  Point centerPoint = Utility::fromFloatPointToIntPoint(center2f);
  circle(img,centerPoint,v.getRadius(),default_circle_color,circle_thickness);
  // load the number image
  //String s = "./data/draw_template/" + to_string( abs(v.getNumber()) )+ ".png";
  //Mat num_img = imread(s);
  // resize the image
  //resize(num_img,num_img,Size(v.getRadius(),v.getRadius()));
  // draw the image on the bigger image
  //num_img.copyTo(img(cv::Rect(centerPoint.x - v.getRadius()/2,centerPoint.y - v.getRadius()/2,num_img.cols, num_img.rows)));
}

// Method that draws a segment
void Plotter::drawSegment(const Segment &seg) {
  line(this->img,seg.getP1(),seg.getP2(),default_line_color,line_thickness);
}

void Plotter::drawSegment(const Segment &seg,Scalar color) {
  line(this->img,seg.getP1(),seg.getP2(),color,line_thickness);
}

// Method that draws an arrow (a small segment)
void Plotter::drawArrow(Point p, float theta) {
  Point endpoint = Point(p.x + 20*cos(theta),p.y + 20*sin(theta));
  line(img, Point(p.x,p.y), endpoint,default_line_color,line_thickness); // draw the line
  //circle(img,endpoint,3,Scalar(255,255,255),2);
}

// Method that draws a dubins arc
void Plotter::drawDubinsArc(const DubinsArc &da) {
  // get the two orignals starting points
  Point2f startf = Point2f(da.getX0(), da.getY0());
  Point2f finishf = Point2f(da.getXf(), da.getYf());
  //cout << "start point: " << startf << " end point " << finishf << endl;

  // if k == 0 this arc is a line
  if(da.getK() == 0) {
    line(img, Point(startf.x,startf.y), Point(finishf.x,finishf.y), default_line_color,line_thickness); // draw the line
  } else {
    float radius = abs(1 / da.getK()); // radius is 1/k
    Point2f centerf;
    // based on the curvature get the float coordinates of the center
    if( da.getK() < 0) {
        float xc = cos( da.getTh0() - M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
        float yc = sin( da.getTh0() - M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
        centerf = Point(xc + da.getX0(),yc + da.getY0());
    } else {
        float xc = cos( da.getTh0() + M_PI/2) * radius; // xc = cos(th0 - Pi/2) * radius because th0 is perpendicular to the radius
        float yc = sin( da.getTh0() + M_PI/2) * radius; // yc = sin(th0 - Pi/2) * radius
        centerf = Point(xc + da.getX0(),yc + da.getY0());
    }
    // get the integer start and end point of the curve
    // converts all the point in opencv int points
    Point start = Point(startf.x,startf.y);
    Point finish = Point(finishf.x,finishf.y);
    // draw the points

    float thetastart = atan2((startf.y - centerf.y),(startf.x - centerf.x));
    float thetafinish= atan2((finishf.y - centerf.y),(finishf.x - centerf.x));
    thetastart = Utility::mod2pi(thetastart);
    thetafinish = Utility::mod2pi(thetafinish);
    //cout << "arc th start " << thetastart << " thfinish " << thetafinish << endl;
    float passo = 0.0174533; // a degree in radiants

    if(da.getK() > 0) { // clockwise segment drawing
      for(unsigned int i = 0; 0.01 < ((Utility::mod2pi(thetastart + passo*i) - thetafinish)*(Utility::mod2pi(thetastart + passo*i) -thetafinish)); i++) {
        Point startSegment = Point( cos( thetastart + passo*i   )*radius + centerf.x,+ sin( thetastart + passo*i   )*radius + centerf.y);
        Point finishSegment = Point( cos( thetastart + passo*(i+1)   )*radius + centerf.x, sin( thetastart + passo*(i+1) )*radius + centerf.y);
        line(img, startSegment, finishSegment, default_line_color,line_thickness); // draw the line
      }
    } else { // counter clockwise segment drawing
      for(unsigned int i = 0; 0.01 < ((Utility::mod2pi(thetastart - passo*i) - thetafinish)*(Utility::mod2pi(thetastart - passo*i) - thetafinish)); i++) {
        Point startSegment = Point( cos( thetastart - passo*i   )*radius + centerf.x,+ sin( thetastart - passo*i   )*radius + centerf.y);
        Point finishSegment = Point( cos( thetastart - passo*(i+1)   )*radius + centerf.x, sin( thetastart - passo*(i+1) )*radius + centerf.y);
        line(img, startSegment, finishSegment, default_line_color,line_thickness); // draw the line
      }
    }

    circle(img,start,2,default_line_color,line_thickness);
    circle(img,finish,2,default_line_color,line_thickness);
    //circle(img,center,radius,Scalar(0,255,255),1);
    //circle(img,center,1,Scalar(255,255,255),1);
    drawArrow(start,da.getTh0());
    drawArrow(finish,da.getThf());
  }

}

// Method that draws a dubins curve
void Plotter::drawDubinsCurve(const DubinsCurve &dc) {
  for(unsigned int i = 0; i < 3; i++) {
    DubinsArc da = dc.getArc(i);
    this->drawDubinsArc(da);
  }
}

// Mathod that draws an entire map
void Plotter::drawMap(Map map) {
  this->img = Mat(map.getHeight(), map.getWidth(),CV_8UC3, background_color);

  Obstacle goal = map.getGoal();
  this->drawObstacle(goal,Scalar(255,0,0));

  vector<Obstacle> obstacles = map.getObstacles();

  for(unsigned int i = 0; i < obstacles.size();i++) {
    this->drawObstacle(obstacles[i]);
  }

  vector<Victim> victims = map.getVictims();

  for(unsigned int i = 0; i < victims.size();i++) {
    this->drawVictim(victims[i]);
  }

  this->drawCircle(Circle(map.getGoal().getCenter(),3));

  this->show();
}

// draw a points graph
void Plotter::drawPointsGraph(PointsGraph pg){

  vector<PointNode> nodes = pg.getNodes();

  for(unsigned int i = 0; i < nodes.size();i++) {
    PointNode currentNode = nodes[i];
    Point2f currentPoint = Point2f(currentNode.getX(),currentNode.getY());
    vector<int> neighbors = currentNode.getNeighbors();

    for(unsigned int j = 0; j < neighbors.size();j++) {

      Point2f connectedPoint = Point2f( nodes[neighbors[j]].getX(), nodes[neighbors[j]].getY() );
      this->drawSegment(Segment(currentPoint,connectedPoint));
    }


  }
  this->show();
}

// draw a clipping library path object
void Plotter::drawClippingLibPaths(const Paths &paths) {

  for(unsigned int i = 0; i < paths.size();i++) {

    vector<Point> vertices = {};

    for(unsigned int j = 0; j < paths[i].size();j++) {
        Point p = Point(paths[i][j].X,paths[i][j].Y);
        vertices.push_back(p);
    }

    vector<vector<Point>> vertices_to_draw = {vertices};
    drawContours(img,vertices_to_draw , -1, Scalar(0,0,255), polygon_thickness, LINE_AA);

  }

}

// clear the main image
void Plotter::clear() {
  img = Mat(1024, 1024, CV_8UC3, background_color);
}

// draw a vector of dubinsCurve
void Plotter::drawDubinsCurves(const vector<DubinsCurve> &dc) {

  for(unsigned int i = 0; i < dc.size();i++) {
    this->drawDubinsCurve(dc[i]);
  }

}
