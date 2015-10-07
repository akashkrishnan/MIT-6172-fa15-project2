#ifndef _QUADTREE_H
#define _QUADTREE_H

#include <stdlib.h>
#include "./Line.h"
#include "./Vec.h"
#include "./IntersectionDetection.h"

#define N (5)

typedef struct Parallelogram {
  Vec points[4];
} Parallelogram;

typedef struct Quadtree {
  int level;
  int num_objects;
  window_dimension x;
  window_dimension y;
  struct Parallelogram* objects;
  struct Quadtree* nodes;
} Quadtree;

void Quadtree_init(Quadtree* q, int level, window_dimension x, window_dimension y);


void Quadtree_delete(Quadtree* q);

void Quadtree_partition(Quadtree* q);

bool parallelogramInBox(Parallelogram* p, int x, int y, int width, int height);

int Quadtree_getIndex(Quadtree* q, Parallelogram* p);

void Quadtree_insert(Quadtree* q, Parallelogram* p);

bool Quadtree_potentialCollide(Line* l1, Line* l2);

#endif // _QUADTREE_H
