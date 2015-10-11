/** 
 * CollisionWorld.c -- detect and handle line segment intersections
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

#include "./CollisionWorld.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

#include "./IntersectionDetection.h"
#include "./IntersectionEventList.h"
#include "./Line.h"
#include "./Quadtree.h"

#define MAX_INTERSECTS 20

CollisionWorld* CollisionWorld_new(const unsigned int capacity) {
  assert(capacity > 0);

  CollisionWorld* cw = malloc(sizeof(CollisionWorld));
  if (cw == NULL) {
    return NULL;
  }

  cw->numLineWallCollisions = 0;
  cw->numLineLineCollisions = 0;
  cw->timeStep = 0.5;
  cw->lines = malloc(capacity * sizeof(Line*));
  cw->line_nodes = malloc(capacity * sizeof(LineNode*));
  cw->numOfLines = 0;
  return cw;
}

void CollisionWorld_delete(CollisionWorld* cw) {
  for (int i = 0; i < cw->numOfLines; i++) {
    free(cw->lines[i]);
    free(cw->line_nodes[i]);
  }
  free(cw->lines);
  free(cw->line_nodes);
  free(cw);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* cw, Line *l) {
  cw->lines[cw->numOfLines] = l;
  cw->line_nodes[cw->numOfLines] = LineNode_make(l);
  cw->numOfLines++;
}

Line* CollisionWorld_getLine(CollisionWorld* cw,
                             const unsigned int index) {
  if (index >= cw->numOfLines) {
    return NULL;
  }
  return cw->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* cw) {
  CollisionWorld_detectIntersection(cw);
  CollisionWorld_updatePosition(cw);
  CollisionWorld_lineWallCollision(cw);
}

void CollisionWorld_updatePosition(CollisionWorld* cw) {
  double t = cw->timeStep;
  Vec delta;
  Line* l;
  for (int i = 0; i < cw->numOfLines; i++) {
    l = cw->lines[i];
    delta = Vec_multiply(l->velocity, t);
    l->p1 = Vec_add(l->p1, delta);
    l->p2 = Vec_add(l->p2, delta);
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld* cw) {
  for (int i = 0; i < cw->numOfLines; i++) {
    Line* l = cw->lines[i];
    bool collide = false;

    // Right side
    if ((l->p1.x > BOX_XMAX || l->p2.x > BOX_XMAX)
        && (l->velocity.x > 0)) {
      l->velocity.x = -l->velocity.x;
      collide = true;
    }

    // Left side
    if ((l->p1.x < BOX_XMIN || l->p2.x < BOX_XMIN)
        && (l->velocity.x < 0)) {
      l->velocity.x = -l->velocity.x;
      collide = true;
    }

    // Top side
    if ((l->p1.y > BOX_YMAX || l->p2.y > BOX_YMAX)
        && (l->velocity.y > 0)) {
      l->velocity.y = -l->velocity.y;
      collide = true;
    }

    // Bottom side
    if ((l->p1.y < BOX_YMIN || l->p2.y < BOX_YMIN)
        && (l->velocity.y < 0)) {
      l->velocity.y = -l->velocity.y;
      collide = true;
    }

    // Update total number of collisions.
    if (collide) {
      cw->numLineWallCollisions++;
    }
  }
}

QuadTree* build_quadtree(CollisionWorld* cw) {
  assert(cw);

  QuadTree* q = QuadTree_make(BOX_XMIN, BOX_XMAX, BOX_YMIN, BOX_YMAX);

  LineList* ll = calloc(1, sizeof(LineList));

  int n = cw->numOfLines;
  LineNode* curr;
  for (int i = 0; i < n; i++) {
    curr = cw->line_nodes[i];

    assert(curr);
    assert(curr->line);

    update_box(curr->line, cw->timeStep);
    LineList_addLineNode(ll, curr);
  }

  QuadTree_addLines(q, ll, cw->timeStep);

  free(ll);

  return q;
}

IntersectionEventList CollisionWorld_getIntersectionEvents(QuadTree* q,
                                                           double timeStep,
                                                           LineList* lines) {
  IntersectionEventList iel = IntersectionEventList_make();
  if (q == NULL) {
    return iel;
  }

  LineNode* first_node = q->lines->head;
  LineNode* second_node;

  while (first_node != NULL) {
    second_node = first_node->next;
    while (second_node != NULL) {
      Line* l1 = first_node->line;
      Line* l2 = second_node->line;

      if (compareLines(l1, l2) >= 0) {
        Line *temp = l1;
        l1 = l2;
        l2 = temp;
      }

      IntersectionType type = intersect(l1, l2, timeStep);
      if (type != NO_INTERSECTION) {
        IntersectionEventList_appendNode(&iel, l1, l2, type);
      }

      second_node = second_node->next;
    }
    first_node = first_node->next;
  }

  first_node = q->lines->head;
  while (first_node != NULL) {
    second_node = lines ? lines->head : NULL;
    while (second_node != NULL) {
      Line* l1 = first_node->line;
      Line* l2 = second_node->line;

      if (compareLines(l1, l2) >= 0) {
        Line *temp = l1;
        l1 = l2;
        l2 = temp;
      }

      IntersectionType type = intersect(l1, l2, timeStep);
      if (type != NO_INTERSECTION) {
        IntersectionEventList_appendNode(&iel, l1, l2, type);
      }

      second_node = second_node->next;
    }
    first_node = first_node->next;
  }

  IntersectionEventList ielQ1;
  IntersectionEventList ielQ2;
  IntersectionEventList ielQ3;
  IntersectionEventList ielQ4;

  if (lines) {
    LineList_concat(q->lines, lines);
  }

  /*if (q->lines->count > MAX_INTERSECTS) {
    ielQ1 = cilk_spawn CollisionWorld_getIntersectionEvents(q->quads[0], timeStep, q->lines);
    ielQ2 = cilk_spawn CollisionWorld_getIntersectionEvents(q->quads[1], timeStep, q->lines);
    ielQ3 = cilk_spawn CollisionWorld_getIntersectionEvents(q->quads[2], timeStep, q->lines);
    ielQ4 = CollisionWorld_getIntersectionEvents(q->quads[3], timeStep, q->lines);
    cilk_sync;
  } else {*/
    ielQ1 = CollisionWorld_getIntersectionEvents(q->quads[0], timeStep, q->lines);
    ielQ2 = CollisionWorld_getIntersectionEvents(q->quads[1], timeStep, q->lines);
    ielQ3 = CollisionWorld_getIntersectionEvents(q->quads[2], timeStep, q->lines);
    ielQ4 = CollisionWorld_getIntersectionEvents(q->quads[3], timeStep, q->lines);
  //}

  IntersectionEventList_concat(&iel, &ielQ1);
  IntersectionEventList_concat(&iel, &ielQ2);
  IntersectionEventList_concat(&iel, &ielQ3);
  IntersectionEventList_concat(&iel, &ielQ4);

  return iel;
}

void CollisionWorld_detectIntersection(CollisionWorld* cw) {
  QuadTree* q = build_quadtree(cw);
  IntersectionEventList iel = CollisionWorld_getIntersectionEvents(q,
                                                                   cw->timeStep,
                                                                   NULL);
  cw->numLineLineCollisions += iel.count;
  QuadTree_delete(q);

  // Sort the intersection event list.
  IntersectionEventNode* startNode = iel.head;
  while (startNode != NULL) {
    IntersectionEventNode* minNode = startNode;
    IntersectionEventNode* curNode = startNode->next;
    while (curNode != NULL) {
      if (IntersectionEventNode_compareData(curNode, minNode) < 0) {
        minNode = curNode;
      }
      curNode = curNode->next;
    }
    if (minNode != startNode) {
      IntersectionEventNode_swapData(minNode, startNode);
    }
    startNode = startNode->next;
  }

  // Call the collision solver for each intersection event.
  IntersectionEventNode* curNode = iel.head;

  while (curNode != NULL) {
    CollisionWorld_collisionSolver(cw, curNode->l1, curNode->l2,
                                   curNode->intersectionType);
    curNode = curNode->next;
  }

  IntersectionEventList_deleteNodes(&iel);
}

unsigned int CollisionWorld_getNumLineWallCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineWallCollisions;
}

unsigned int CollisionWorld_getNumLineLineCollisions(
    CollisionWorld* collisionWorld) {
  return collisionWorld->numLineLineCollisions;
}

void CollisionWorld_collisionSolver(CollisionWorld* collisionWorld,
                                    Line *l1, Line *l2,
                                    IntersectionType intersectionType) {
  assert(compareLines(l1, l2) < 0);
  assert(intersectionType == L1_WITH_L2
         || intersectionType == L2_WITH_L1
         || intersectionType == ALREADY_INTERSECTED);

  // Despite our efforts to determine whether lines will intersect ahead
  // of time (and to modify their velocities appropriately), our
  // simplified model can sometimes cause lines to intersect.  In such a
  // case, we compute velocities so that the two lines can get unstuck in
  // the fastest possible way, while still conserving momentum and kinetic
  // energy.
  if (intersectionType == ALREADY_INTERSECTED) {
    Vec p = getIntersectionPoint(l1->p1, l1->p2, l2->p1, l2->p2);

    if (Vec_length(Vec_subtract(l1->p1, p))
        < Vec_length(Vec_subtract(l1->p2, p))) {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p2, p)),
                                  Vec_length(l1->velocity));
    } else {
      l1->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l1->p1, p)),
                                  Vec_length(l1->velocity));
    }
    if (Vec_length(Vec_subtract(l2->p1, p))
        < Vec_length(Vec_subtract(l2->p2, p))) {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p2, p)),
                                  Vec_length(l2->velocity));
    } else {
      l2->velocity = Vec_multiply(Vec_normalize(Vec_subtract(l2->p1, p)),
                                  Vec_length(l2->velocity));
    }
    return;
  }

  // Compute the collision face/normal vectors.
  Vec face;
  Vec normal;
  if (intersectionType == L1_WITH_L2) {
    Vec v = Vec_makeFromLine(*l2);
    face = Vec_normalize(v);
  } else {
    Vec v = Vec_makeFromLine(*l1);
    face = Vec_normalize(v);
  }
  normal = Vec_orthogonal(face);

  // Obtain each line's velocity components with respect to the collision
  // face/normal vectors.
  double v1Face = Vec_dotProduct(l1->velocity, face);
  double v2Face = Vec_dotProduct(l2->velocity, face);
  double v1Normal = Vec_dotProduct(l1->velocity, normal);
  double v2Normal = Vec_dotProduct(l2->velocity, normal);

  // Compute the mass of each line (we simply use its length).
  double m1 = Vec_length(Vec_subtract(l1->p1, l1->p2));
  double m2 = Vec_length(Vec_subtract(l2->p1, l2->p2));

  // Perform the collision calculation (computes the new velocities along
  // the direction normal to the collision face such that momentum and
  // kinetic energy are conserved).
  double newV1Normal = ((m1 - m2) / (m1 + m2)) * v1Normal
      + (2 * m2 / (m1 + m2)) * v2Normal;
  double newV2Normal = (2 * m1 / (m1 + m2)) * v1Normal
      + ((m2 - m1) / (m2 + m1)) * v2Normal;

  // Combine the resulting velocities.
  l1->velocity = Vec_add(Vec_multiply(normal, newV1Normal),
                         Vec_multiply(face, v1Face));
  l2->velocity = Vec_add(Vec_multiply(normal, newV2Normal),
                         Vec_multiply(face, v2Face));

  return;
}
