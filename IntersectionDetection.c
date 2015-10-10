/**
 * Copyright (c) 2012 the Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/

#include "./IntersectionDetection.h"

#include <assert.h>

#include "./Line.h"
#include "./Vec.h"

#define MIN(a,b) ((a<b)?a:b)
#define MAX(a,b) ((a>b)?a:b)

inline IntersectionType intersect(Line *l1, Line *l2, Vec p1, Vec p2) {
  assert(compareLines(l1, l2) < 0);

  // lines intersect before timestep
  if (intersectLines(l1->p1, l1->p2, l2->p1, l2->p2)) {
    return ALREADY_INTERSECTED;
  }

  // There is a parallelogram formed by the motion of the second line.
  // Passes if the second line completely passes the first line.
  if (pointInParallelogram(l1->p1, l2->p1, l2->p2, p1, p2)
      && pointInParallelogram(l1->p2, l2->p1, l2->p2, p1, p2)) {
    return L1_WITH_L2;
  }

  // Check the number of line intersections, as it is possible
  // for neither of the two l1 endpoints to be within the l2 parallelogram.
  int num_line_intersections = 0;
  bool top_intersected = false;
  bool bottom_intersected = false;
  
  if (intersectLines(l1->p1, l1->p2, p1, p2)) {
    num_line_intersections++;
  }
  if (intersectLines(l1->p1, l1->p2, p1, l2->p1)) {
    num_line_intersections++;
    top_intersected = true;
  }
  if (num_line_intersections == 2) {
    return L2_WITH_L1;
  }
  if (intersectLines(l1->p1, l1->p2, p2, l2->p2)) {
    num_line_intersections++;
    bottom_intersected = true;
  }

  // Calculate angle to determine type of intersection.
  Vec v1 = Vec_makeFromLine(*l1);
  Vec v2 = Vec_makeFromLine(*l2);
  double angle = Vec_angle(v1, v2);

  if (num_line_intersections == 2) {
    return L2_WITH_L1;
  }
  
  if (top_intersected && angle < 0){
    return L2_WITH_L1;
  }
  if (bottom_intersected && angle > 0){
    return L2_WITH_L1;
  }

  return L1_WITH_L2;
}

inline IntersectionType fastIntersect(Line *l1, Line *l2, Vec p1, Vec p2) {
  assert(compareLines(l1, l2) < 0);
  
  // Variables to avoid looking up p1 and p2 for lines 1 and 2 each time.
  Vec l1p1 = l1->p1;
  Vec l1p2 = l1->p2;
  Vec l2p1 = l2->p1;
  Vec l2p2 = l2->p2;

  // Bounding box: check if one line is entirely to one side or the other
  // of the parallelogram created by the movement of line 2 relative to line 1.
  if (MAX(l1p1.x,l1p2.x) < MIN(MIN(l2p1.x,l2p2.x),MIN(p1.x,p2.x))) {
    return false;
  }
  if (MIN(l1p1.x,l1p2.x) > MAX(MAX(l2p1.x,l2p2.x),MAX(p1.x,p2.x))) {
    return false;
  }
  if (MAX(l1p1.y,l1p2.y) < MIN(MIN(l2p1.y,l2p2.y),MIN(p1.y,p2.y))) {
    return false;
  }
  if (MIN(l1p1.y,l1p2.y) > MAX(MAX(l2p1.y,l2p2.y),MAX(p1.y,p2.y))) {
    return false;
  }

  // Check for overlap of line with parallelogram.
  if (pointInParallelogram(l1p1, l2p1, l2p2, p1, p2)) {
    return true;
  }
  if (pointInParallelogram(l1p2, l2p1, l2p2, p1, p2)) {
    return true;
  }
  if (intersectLines(l1p1, l1p2, l2p1, l2p2)) {
    return true;
  }
  if (intersectLines(l1p1, l1p2, p1, p2)) {
    return true;
  }
  if (intersectLines(l1p1, l1p2, p1, l2p1)) {
    return true;
  }
  return false;
}

// Check if a point is in the square.
inline bool pointInSquare(Vec p, Vec p1, Vec p4) {
  return p1.x <= p.x && p.x <= p4.x &&
         p1.y <= p.y && p.y <= p4.y;
}

// Check if a point is in the parallelogram.
inline bool pointInParallelogram(Vec point, Vec p1, Vec p2, Vec p3, Vec p4) {
  double d1 = direction(p1, p2, point);
  double d2 = direction(p3, p4, point);
  double d3 = direction(p1, p3, point);
  double d4 = direction(p2, p4, point);

  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return true;
  }
  return false;
}

// Check if two lines intersect.
inline bool intersectLines(Vec p1, Vec p2, Vec p3, Vec p4) {
  // Relative orientation
  double d1 = direction(p3, p4, p1);
  double d2 = direction(p3, p4, p2);
  double d3 = direction(p1, p2, p3);
  double d4 = direction(p1, p2, p4);

  // If (p1, p2) and (p3, p4) straddle each other, the line segments must
  // intersect.
  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0))
      && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
    return true;
  }
  if (d1 == 0 && onSegment(p3, p4, p1)) {
    return true;
  }
  if (d2 == 0 && onSegment(p3, p4, p2)) {
    return true;
  }
  if (d3 == 0 && onSegment(p1, p2, p3)) {
    return true;
  }
  if (d4 == 0 && onSegment(p1, p2, p4)) {
    return true;
  }
  return false;
}

// Obtain the intersection point for two intersecting line segments.
inline Vec getIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {
  double u;

  u = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x))
      / ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y));

  return Vec_add(p1, Vec_multiply(Vec_subtract(p2, p1), u));
}

// Check the direction of two lines (pi, pj) and (pi, pk).
inline double direction(Vec pi, Vec pj, Vec pk) {
  return crossProduct(pk.x - pi.x, pk.y - pi.y, pj.x - pi.x, pj.y - pi.y);
}

// Check if a point pk is in the line segment (pi, pj).
// pi, pj, and pk must be collinear.
inline bool onSegment(Vec pi, Vec pj, Vec pk) {
  if (((pi.x <= pk.x && pk.x <= pj.x) || (pj.x <= pk.x && pk.x <= pi.x))
      && ((pi.y <= pk.y && pk.y <= pj.y) || (pj.y <= pk.y && pk.y <= pi.y))) {
    return true;
  }
  return false;
}

// Calculate the cross product.
inline double crossProduct(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

