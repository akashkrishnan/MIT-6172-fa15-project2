#ifndef QUADTREE_H_
#define QUADTREE_H_

#include <stdlib.h>

#include "./CollisionWorld.h"
#include "./Vec.h"
#include "./Line.h"
#include "./IntersectionEventList.h"

#define MAX_POINTS_PER_QUAD 3
#define MAX_LINES_PER_QUAD 100

typedef struct Quadtree Quadtree;

struct Quadtree {

  // Useful pointers
  CollisionWorld* world;
  Quadtree* parent;
  unsigned int depth;

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
                          Quadtree* parent,
                          Vec topLeft,
                          Vec botRight);

void Quadtree_delete(Quadtree* q);

void Quadtree_update(Quadtree* q);

void Quadtree_updateLines(Quadtree* q);

bool Quadtree_isDivisible(Quadtree* q);

bool Quadtree_containsLine(Quadtree* q, Line* l);

//void Quadtree_compileLines(Quadtree* q);

bool Quadtree_addLine(Quadtree* q, Line* l);

void Quadtree_divide(Quadtree* q);

unsigned int Quadtree_detectCollisions(Quadtree* q, IntersectionEventList* iel);

#endif // QUADTREE_H_

