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

inline static bool rectanglesOverlap(Line* l1, Line* l2) {
  return l1->l_x <= l2->u_x && l1->u_x >= l2->l_x &&
         l1->l_y <= l2->u_y && l1->u_y >= l2->l_y;
}

inline static bool side(Vec E, Vec F, Vec P) {
  return (F.x - E.x) * (P.y - F.y) -
         (F.y - E.y) * (P.x - F.x) >= 0;
}

// Detect if lines l1 and l2 will intersect between now and the next time step.
inline IntersectionType intersect(Line *l1, Line *l2, double time) {
  assert(compareLines(l1, l2) < 0);

  if (!rectanglesOverlap(l1, l2)) {
    return NO_INTERSECTION;
  }

  Vec p1 = {.x = l2->p3.x - l1->delta.x, .y = l2->p3.y - l1->delta.y};
  Vec p2 = {.x = l2->p4.x - l1->delta.x, .y = l2->p4.y - l1->delta.y};

  if (intersectLines(l1->p1, l1->p2, l2->p1, l2->p2)) {
    return ALREADY_INTERSECTED;
  }

  int num_line_intersections = 0;
  bool top_intersected = false;
  bool bot_intersected = false;

  if (intersectLines(l1->p1, l1->p2, p1, p2)) {
    num_line_intersections++;
  }
  if (intersectLines(l1->p1, l1->p2, p1, l2->p1)) {
    num_line_intersections++;
    top_intersected = true;
  }
  if (intersectLines(l1->p1, l1->p2, p2, l2->p2)) {
    num_line_intersections++;
    bot_intersected = true;
  }

  if (num_line_intersections == 2) {
    return L2_WITH_L1;
  }

  if (pointInParallelogram(l1->p1, l2->p1, l2->p2, p1, p2) &&
      pointInParallelogram(l1->p2, l2->p1, l2->p2, p1, p2)) {
    return L1_WITH_L2;
  }

  if (num_line_intersections == 0) {
    return NO_INTERSECTION;
  }

  Vec v1 = Vec_makeFromLine(*l1);
  Vec v2 = Vec_makeFromLine(*l2);

  double angle = Vec_angle(v1, v2);

  if ((top_intersected && angle < 0) ||
      (bot_intersected && angle > 0)) {
    return L2_WITH_L1;
  }

  return L1_WITH_L2;
}

// Check if a point is in the parallelogram.
inline bool pointInParallelogram(Vec point, Vec p1, Vec p2, Vec p3, Vec p4) {
  double d1 = direction(p1, p2, point);
  double d2 = direction(p3, p4, point);
  double d3 = direction(p1, p3, point);
  double d4 = direction(p2, p4, point);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

// Check if two lines intersect.
inline bool intersectLines(Vec p1, Vec p2, Vec p3, Vec p4) {
  return side(p1, p2, p3) != side(p1, p2, p4) &&
         side(p3, p4, p1) != side(p3, p4, p2);
}

// Obtain the intersection point for two intersecting line segments.
inline Vec getIntersectionPoint(Vec p1, Vec p2, Vec p3, Vec p4) {
  double u;

  u = ((p4.x - p3.x) * (p1.y - p3.y) -
       (p4.y - p3.y) * (p1.x - p3.x)) /
      ((p4.y - p3.y) * (p2.x - p1.x) -
       (p4.x - p3.x) * (p2.y - p1.y));

  Vec p;
  p.x = p1.x + (p2.x - p1.x) * u;
  p.y = p1.y + (p2.y - p1.y) * u;

  return p;
}

// Check the direction of two lines (pi, pj) and (pi, pk).
inline double direction(Vec pi, Vec pj, Vec pk) {
  return crossProduct(pk.x - pi.x, pk.y - pi.y,
                      pj.x - pi.x, pj.y - pi.y);
}

// Check if a point pk is in the line segment (pi, pj).
// pi, pj, and pk must be collinear.
inline bool onSegment(Vec pi, Vec pj, Vec pk) {
  return ((pi.x <= pk.x && pk.x <= pj.x) ||
          (pj.x <= pk.x && pk.x <= pi.x)) &&
         ((pi.y <= pk.y && pk.y <= pj.y) ||
          (pj.y <= pk.y && pk.y <= pi.y));
}

// Calculate the cross product.
inline double crossProduct(double x1, double y1, double x2, double y2) {
  return x1 * y2 - x2 * y1;
}

