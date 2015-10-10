#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <stdlib.h>

#include "./CollisionWorld.h"
#include "./Vec.h"
#include "./Line.h"
#include "./IntersectionEventList.h"

#include <cilk/reducer_opadd.h>

#define MAX_LINES_PER_QUAD 65

typedef struct Quadtree Quadtree;

struct Quadtree {

  // Useful pointers
  CollisionWorld* world;

  // Location of quadrant in window
  Vec topLeft;
  Vec botRight;

  // Four quadrants
  Quadtree** quads;

  // Lines in this quadtree, which includes lines from parent quadtrees
  Line** lines;
  unsigned int numOfLines;

  // Set to true if numLines < MAX_LINES_PER_QUAD
  bool isLeaf;

};

Quadtree* Quadtree_create(CollisionWorld* world,
                          Vec topLeft,
                          Vec botRight);

void Quadtree_delete(Quadtree* q);

bool Quadtree_isDivisible(Quadtree* q);

bool Quadtree_containsLine(Quadtree* q, Line* l);

bool Quadtree_addLine(Quadtree* q, Line* l);

void Quadtree_divide(Quadtree* q);

void Quadtree_detectCollisions(Quadtree* q, IntersectionEventListReducer* iel, CILK_C_REDUCER_OPADD_TYPE(int)* n);

#endif // QUADTREE_H_

