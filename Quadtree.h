#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"
#include "./IntersectionEventList.h"

#define N 70
#define MAX_INTERSECTS 5
#define MAX_DEPTH 3

typedef struct LineNode {
  Line* line;
  struct LineNode* next;
} LineNode;

LineNode* LineNode_make(Line* line);

void LineNode_delete(LineNode* ln);

typedef struct LineList {
  int count;
  LineNode* head;
  LineNode* tail;
} LineList;

void LineList_addLineNode(LineList* ll, LineNode* ln);

void LineList_concat(LineList* l, LineList* r);

typedef struct QuadTree {
  double x1, x2, y1, y2, x0, y0;
  struct QuadTree** quads;
  LineList* lines;
  int empty;
} QuadTree;

QuadTree* QuadTree_make(double x1, double x2, double y1, double y2);

void QuadTree_delete(QuadTree* q);

void QuadTree_reset(QuadTree* q);

void QuadTree_build(QuadTree* q, int depth);

int QuadTree_getQuadWithLine(double x, double y, Vec p1, Vec p2);

int QuadTree_getQuad(double x, double y, LineNode* ln, double timeStep);

void QuadTree_addLines(QuadTree* q, double timeStep);

void QuadTree_detectEvents(QuadTree* q, LineList* lines, double timeStep, IntersectionEventListReducer* iel);

#endif  // QUADTREE_H_

