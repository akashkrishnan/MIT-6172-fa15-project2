#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"

#define MULTIPLE_QUADS 0

#define N 62

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

LineList* LineList_make();

void LineList_delete(LineList* ll);

void LineList_addLineNode(LineList* ll, LineNode* ln);

void LineList_concat(LineList* l, LineList* r);


typedef struct QuadTree {
  double x1, x2, y1, y2;
  struct QuadTree** quads;
  LineList* lines;
} QuadTree;

QuadTree* QuadTree_make(double x1, double x2, double y1, double y2);

void QuadTree_delete(QuadTree* q);

int QuadTree_getQuadWithLine(QuadTree* q, Vec p1, Vec p2);

int QuadTree_getQuad(QuadTree* q, LineNode* ln, double timeStep);

void QuadTree_addLines(QuadTree* q, LineList* ll, double timeStep);

#endif  // QUADTREE_H_

