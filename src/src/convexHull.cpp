

//An implmentation of Graham's Scan to find the 2D convex hull

#include "convexHull.h"

#include<iostream>
#include<algorithm>
#include<stack>

using namespace std;
using namespace UNL_Robotics;

Point2D p0; //used to another two points

namespace {
  
  Point2D secondTop(stack<Point2D> &stk)
  {
    Point2D tempPoint = stk.top(); 
    stk.pop();
    Point2D res = stk.top();    //get the second top element
    stk.push(tempPoint);      //push previous top again
    return res;
  }

  double squaredDist(Point2D p1, Point2D p2)
  {
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
  }

  int direction(Point2D a, Point2D b, Point2D c)
  {
    double val = (b.y-a.y)*(c.x-b.x)-(b.x-a.x)*(c.y-b.y);
    if(val == 0.0)
      return 0;    //colinear
    else if(val < 0.0)
      return 2;    //anti-clockwise direction
    return 1;    //clockwise direction
  }

  int comp(const void* Point2D1, const void* Point2D2)
  {
    Point2D* p1 = (Point2D*)Point2D1;
    Point2D* p2 = (Point2D*)Point2D2;
    
    int dir = direction(p0, *p1, *p2);
    if(dir == 0)
      return(squaredDist(p0, *p2) >= squaredDist(p0, *p1)) ? -1 : 1;
    return (dir==2) ? -1 : 1;
  }
}

////////////

std::vector<UNL_Robotics::Point2D> UNL_Robotics::findConvexHull(std::vector<UNL_Robotics::Point2D> inputPoints)
{
  if(inputPoints.size() < 3) {

    std::cout << "Error - findConvexHull must be called with at least 3 points.";
    throw("findConvexHull");
  }

  unsigned numPoints = inputPoints.size();
  vector<Point2D> convexHullPoints;
  double minY = inputPoints[0].y;
  double min = 0.0;
  for(int i = 1; i<numPoints; ++i) {
    double y = inputPoints[i].y;
    //find bottom most or left most point
    if((y < minY) || (minY == y) && inputPoints[i].x < inputPoints[min].x) {
      minY = inputPoints[i].y;
      min = i;
    }
   }
   swap(inputPoints[0], inputPoints[min]);    //swap min point to 0th location
   p0 = inputPoints[0];
   qsort(&inputPoints[1], numPoints-1, sizeof(Point2D), comp);    //sort points from 1 place to end
   int arrSize = 1;    //used to locate items in modified array
   for(int i = 1; i<numPoints; ++i) {
      //when the angle of ith and (i+1)th elements are same, remove points
      while(i < numPoints-1 && direction(p0, inputPoints[i], inputPoints[i+1]) == 0)
         i++;
         inputPoints[arrSize] = inputPoints[i];
         arrSize++;
   }
   if(arrSize < 3)
      return convexHullPoints;    //there must be at least 3 points, return empty list.
      //create a stack and add first three points in the stack
      stack<Point2D> stk;
      stk.push(inputPoints[0]); stk.push(inputPoints[1]); stk.push(inputPoints[2]);
   for(int i = 3; i<arrSize; ++i) {    //for remaining vertices
      while(direction(secondTop(stk), stk.top(), inputPoints[i]) != 2)
         stk.pop();    //when top, second top and ith point are not making left turn, remove point
         stk.push(inputPoints[i]);
   }
   while(!stk.empty()) {
      convexHullPoints.push_back(stk.top());    //add points from stack
      stk.pop();
   }

   return convexHullPoints;
}
