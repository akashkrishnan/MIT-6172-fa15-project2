#include "./Quadtree.h"

#include "./CollisionWorld.h"
#include "./Vec.h"
#include "./Line.h"
#include "./IntersectionEventList.h"
#include "./IntersectionDetection.h"

Quadtree* Quadtree_create(CollisionWorld* world,
                          Quadtree* parent,
                          Vec topLeft,
                          Vec botRight) {
  Quadtree* q = malloc(sizeof(Quadtree));
  if (q == NULL) {
    return NULL;
  }

  // Pointers
  q->world  = world;
  q->parent = parent;
  q->depth  = parent ? parent->depth + 1 : 0;

  // Location
  q->world    = world;
  q->topLeft  = topLeft;
  q->botRight = botRight;

  // Create array of lines
  q->lines    = malloc(sizeof(Line*) * MAX_LINES_PER_QUAD);
  q->numOfLines = 0;
  q->isLeaf   = !Quadtree_isDivisible(q);

  // Check if quadtree is a leaf
  if (q->isLeaf) {
    // We're a leaf, so we need to compile all the lines
    //Quadtree_compileLines(q);
  } else {
    // We're not a leaf, so we need to make four quadrants
    q->quads = malloc(sizeof(Quadtree*) * 4);
    Quadtree_divide(q);
  }
  
  return q;
}

void Quadtree_delete(Quadtree* q) {
  free(q->lines);
  if (!(q->isLeaf)){
    for (int i = 0; i < 4; i++) {
      Quadtree_delete(q->quads[i]);
    }
    free(q->quads);
  }
  free(q);
}

void Quadtree_update(Quadtree* q) {
  // Updating leaves is the base case
  if (q->isLeaf){
    // Base case
    Quadtree_updateLines(q);
  } else {
    // Update four quads
    for (int i = 0; i < 4; i++) {
      Quadtree_update(q->quads[i]);
    }
  }
}

void Quadtree_updateLines(Quadtree* q) {
  // Reset numOfLines because we're going to be adding lines again
  q->numOfLines = 0;

  // Loop through all lines and add lines that are in this quadtree
  int n =  q->world->numOfLines;
  for (int i = 0; i < n; i++) {
    Line* l = q->world->lines[i];
    if (Quadtree_containsLine(q, l)) {
      Quadtree_addLine(q, l);
    }
  }
}

bool Quadtree_isDivisible(Quadtree* q) {
  // Loop through all lines and check to see if a line cannot be added
  int n = q->world->numOfLines;
  for (int i = 0; i < n; i++) {
    Line* l = q->world->lines[i];
    if (Quadtree_containsLine(q, l) && !Quadtree_addLine(q, l)) {
      return true;
    }
  }
  return false;
}

bool Quadtree_containsLine(Quadtree* q, Line* l) {
  // Compute bounding box
  Vec b1 = q->topLeft;
  Vec b4 = q->botRight;
  Vec b2 = Vec_make(b1.x, b4.y);
  Vec b3 = Vec_make(b4.x, b1.y);
  
  // Compute parallelogram
  Vec l1 = l->p1;
  Vec l2 = l->p2;
  Vec l3 = Vec_add(l1, Vec_multiply(l->velocity, q->world->timeStep));
  Vec l4 = Vec_add(l2, Vec_multiply(l->velocity, q->world->timeStep));
  
  return pointInParallelogram(l1, b1, b2, b3, b4) ||
         pointInParallelogram(l2, b1, b2, b3, b4) ||
         pointInParallelogram(l3, b1, b2, b3, b4) ||
         pointInParallelogram(l4, b1, b2, b3, b4) ||
         pointInParallelogram(b1, l1, l2, l3, l4) ||
         pointInParallelogram(b2, l1, l2, l3, l4) ||
         pointInParallelogram(b3, l1, l2, l3, l4) ||
         pointInParallelogram(b4, l1, l2, l3, l4);
}

bool Quadtree_addLine(Quadtree* q, Line* l) {
  if (q->numOfLines >= MAX_LINES_PER_QUAD) {
    return false;
  }
  q->lines[q->numOfLines++] = l;
  return true;
}

void Quadtree_divide(Quadtree* q) {
  // Create new quadrants based on computed midpoint
  Vec mid = Vec_divide(Vec_add(q->topLeft, q->botRight), 2);

  // TOP LEFT
  q->quads[0] = Quadtree_create(
    q->world,
    q,
    q->topLeft,
    mid
  );

  // TOP RIGHT
  q->quads[1] = Quadtree_create(
    q->world,
    q,
    Vec_make(mid.x, q->topLeft.y),
    Vec_make(q->botRight.x, mid.y));

  // BOTTOM LEFT
  q->quads[2] = Quadtree_create(
    q->world,
    q,
    Vec_make(q->topLeft.x, mid.y),
    Vec_make(mid.x, q->botRight.y)
  );

  // BOTTOM RIGHT
  q->quads[3] = Quadtree_create(
    q->world,
    q,
    mid, 
    q->botRight
  );
}

unsigned int Quadtree_detectCollisions(Quadtree* q,
                                       IntersectionEventList* iel) {
  unsigned int n = 0, i, j;
  if (q->isLeaf) {
    // Loop through lines in this quadrant
    for (i = 0; i < q->numOfLines; i++) {
    Line *l1 = q->lines[i];

      // Loop through lines in this quadrant to check for collision
      for (j = i + 1; j < q->numOfLines; j++) {
        Line *l2 = q->lines[j];

        // Swap lines if necessary for intersect function
        if (compareLines(l1, l2) >= 0) {
          Line *tmp = l1;
          l1 = l2;
          l2 = tmp;
        }

        // Check if lines intersect
        IntersectionType type = intersect(l1, l2, q->world->timeStep);
        if (type != NO_INTERSECTION) {
          IntersectionEventList_appendNode(iel, l1, l2, type);
          n++;
        }
      }
    }
  } else {
    // Recurssively add collisions in 4 quadrants
    for (int i = 0; i < 4; i++) {
      n += Quadtree_detectCollisions(q->quads[i], iel);
    }
  }
  return n;
}

