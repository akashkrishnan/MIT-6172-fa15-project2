#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"
#include "./IntersectionEventList.h"

#define N 50
#define MAX_INTERSECTS 5
#define MAX_DEPTH 5
#define PARENT_QUAD 4

typedef struct LineList {
  int count;
  Line* head;
  Line* tail;
} LineList;

void LineList_addLine(LineList* ll, Line* l);

void LineList_concat(LineList* l, LineList* r);

typedef struct QuadTree {
  double x1, x2, y1, y2, x0, y0;
  struct QuadTree** quads;
  LineList* lines;
  bool children, leaf;
} QuadTree;

QuadTree* QuadTree_make(double x1, double x2, double y1, double y2);

void QuadTree_delete(QuadTree* q);

void QuadTree_reset(QuadTree* q);

void QuadTree_build(QuadTree* q, int depth);

int QuadTree_getQuadWithLine(QuadTree* q, Vec p1, Vec p2);

int QuadTree_getQuad(QuadTree* q, Line* l, double t);

void QuadTree_addLines(QuadTree* q, double t);

void QuadTree_detectEvents(QuadTree* q, LineList* lines, double t, IntersectionEventListReducer* iel);

#endif  // QUADTREE_H_

