#include "./Quadtree.h"

#include "./CollisionWorld.h"
#include "./Vec.h"
#include "./Line.h"
#include "./IntersectionEventList.h"
#include "./IntersectionDetection.h"

#include <cilk/cilk.h>
#include <cilk/reducer.h>
#include <cilk/reducer_opadd.h>

inline Quadtree* Quadtree_create(CollisionWorld* world,
                          Vec topLeft,
                          Vec botRight) {
  Quadtree* q = malloc(sizeof(Quadtree));
  if (q == NULL) {
    return NULL;
  }

  // Pointers
  q->world  = world;

  // Location
  q->topLeft  = topLeft;
  q->botRight = botRight;

  // Create array of lines
  q->lines      = malloc(sizeof(Line*) * MAX_LINES_PER_QUAD);
  q->numOfLines = 0;
  q->isLeaf     = !Quadtree_isDivisible(q);

  // Check if quadtree is a leaf
  if (q->isLeaf) {
  } else {
    // We're not a leaf, so we need to make four quadrants
    q->quads = malloc(sizeof(Quadtree*) * 4);
    Quadtree_divide(q);
  }
  
  return q;
}

inline void Quadtree_delete(Quadtree* q) {
  free(q->lines);
  if (!(q->isLeaf)){
    Quadtree_delete(q->quads[0]);
    Quadtree_delete(q->quads[1]);
    Quadtree_delete(q->quads[2]);
    Quadtree_delete(q->quads[3]);
    free(q->quads);
  }
  free(q);
}

inline bool Quadtree_isDivisible(Quadtree* q) {
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

inline bool Quadtree_containsLine(Quadtree* q, Line* l) {
  // Compute quadrant box
  Vec b1 = q->topLeft;
  Vec b4 = q->botRight;
  Vec b2 = Vec_make(b4.x, b1.y);
  Vec b3 = Vec_make(b1.x, b4.y);

  // Compute parallelogram
  Vec l1 = l->p1;
  Vec l2 = l->p2;
  Vec l3 = Vec_add(l1, Vec_multiply(l->velocity, q->world->timeStep));
  Vec l4 = Vec_add(l2, Vec_multiply(l->velocity, q->world->timeStep));

  // Check if parallelogram is completely outside of quadrant
  if (l1.x < b1.x && l2.x < b1.x && l3.x < b1.x && l4.x < b1.x) {
    return false;
  }
  if (l1.x > b4.x && l2.x > b4.x && l3.x > b4.x && l4.x > b4.x) {
    return false;
  }
  if (l1.y < b1.y && l2.y < b1.y && l3.y < b1.y && l4.x < b1.y) {
    return false;
  }
  if (l1.y > b4.y && l2.y > b4.y && l3.y > b4.y && l4.y > b4.y) {
    return false;
  }

  return pointInSquare(l1, b1, b4) ||
         pointInSquare(l2, b1, b4) ||
         pointInSquare(l3, b1, b4) ||
         pointInSquare(l4, b1, b4) ||
         intersectLines(b1, b2, l1, l2) ||
         intersectLines(b1, b3, l1, l2) ||
         intersectLines(b2, b4, l1, l2) ||
         intersectLines(b3, b4, l1, l2) ||
         intersectLines(b1, b2, l1, l3) ||
         intersectLines(b1, b3, l1, l3) ||
         intersectLines(b2, b4, l1, l3) ||
         intersectLines(b3, b4, l1, l3);
}

inline bool Quadtree_addLine(Quadtree* q, Line* l) {
  if (q->numOfLines >= MAX_LINES_PER_QUAD) {
    return false;
  }
  q->lines[q->numOfLines++] = l;
  return true;
}

inline void Quadtree_divide(Quadtree* q) {
  // Create new quadrants based on computed midpoint
  Vec mid = Vec_divide(Vec_add(q->topLeft, q->botRight), 2);

  // TOP LEFT
  q->quads[0] = Quadtree_create(
    q->world,
    q->topLeft,
    mid
  );

  // TOP RIGHT
  q->quads[1] = Quadtree_create(
    q->world,
    Vec_make(mid.x, q->topLeft.y),
    Vec_make(q->botRight.x, mid.y));

  // BOTTOM LEFT
  q->quads[2] = Quadtree_create(
    q->world,
    Vec_make(q->topLeft.x, mid.y),
    Vec_make(mid.x, q->botRight.y)
  );

  // BOTTOM RIGHT
  q->quads[3] = Quadtree_create(
    q->world,
    mid, 
    q->botRight
  );
}

inline void Quadtree_detectCollisions(Quadtree* q,
                                          IntersectionEventListReducer* iel,
                                          CILK_C_REDUCER_OPADD_TYPE(int)* n) {
  if (q->isLeaf) {
    // Loop through lines in this quadrant
    for (int i = 0; i < q->numOfLines; i++) {
      Line *l1 = q->lines[i];

      // Loop through lines in this quadrant to check for collision
      for (int j = i + 1; j < q->numOfLines; j++) {
        Line *l2 = q->lines[j];

        // Swap lines if necessary for intersect function
        if (compareLines(l1, l2) >= 0) {
          Line *tmp = l1;
          l1 = l2;
          l2 = tmp;
        }

        Vec p1;
        Vec p2;
        // Get relative velocity.
        Vec shift;
        shift.x = l2->shift.x - l1->shift.x;
        shift.y = l2->shift.y - l1->shift.y;

        // Get the parallelogram.
        p1.x = l2->p1.x + shift.x;
        p1.y = l2->p1.y + shift.y;
  
        p2.x = l2->p2.x + shift.x;
        p2.y = l2->p2.y + shift.y;
        if (fastIntersect(l1, l2, p1, p2)) {
          IntersectionEventList_appendNode(&REDUCER_VIEW(*iel), l1, l2,
                                    intersect(l1, l2, p1, p2));
          REDUCER_VIEW(*n)++;        
        }
      }
    }
  } else {
    // Recurssively add collisions in 4 quadrants
    for (int i = 0; i < 4; i++) {
      Quadtree_detectCollisions(q->quads[i], iel, n);
    }
  }
}

