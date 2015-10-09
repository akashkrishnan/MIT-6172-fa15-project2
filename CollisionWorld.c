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

  CollisionWorld* collisionWorld = malloc(sizeof(CollisionWorld));
  if (collisionWorld == NULL) {
    return NULL;
  }

  collisionWorld->numLineWallCollisions = 0;
  collisionWorld->numLineLineCollisions = 0;
  collisionWorld->timeStep = 0.5;
  collisionWorld->lines = malloc(capacity * sizeof(Line*));
  collisionWorld->line_nodes = malloc(capacity * sizeof(LineNode*));
  collisionWorld->numOfLines = 0;
  return collisionWorld;
}

void CollisionWorld_delete(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    free(collisionWorld->lines[i]);
    free(collisionWorld->line_nodes[i]);
  }
  free(collisionWorld->lines);
  free(collisionWorld->line_nodes);
  free(collisionWorld);
}

unsigned int CollisionWorld_getNumOfLines(CollisionWorld* collisionWorld) {
  return collisionWorld->numOfLines;
}

void CollisionWorld_addLine(CollisionWorld* collisionWorld, Line *line) {
  collisionWorld->lines[collisionWorld->numOfLines] = line;
  collisionWorld->line_nodes[collisionWorld->numOfLines] = LineNode_make(line);
  collisionWorld->numOfLines++;
}

Line* CollisionWorld_getLine(CollisionWorld* collisionWorld,
                             const unsigned int index) {
  if (index >= collisionWorld->numOfLines) {
    return NULL;
  }
  return collisionWorld->lines[index];
}

void CollisionWorld_updateLines(CollisionWorld* collisionWorld) {
  CollisionWorld_detectIntersection(collisionWorld);
  CollisionWorld_updatePosition(collisionWorld);
  CollisionWorld_lineWallCollision(collisionWorld);
}

void CollisionWorld_updatePosition(CollisionWorld* collisionWorld) {
  double t = collisionWorld->timeStep;
  Vec delta;
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    delta = Vec_multiply(line->velocity, t);
    line->p1 = Vec_add(line->p1, delta);
    line->p2 = Vec_add(line->p2, delta);
  }
}

void CollisionWorld_lineWallCollision(CollisionWorld* collisionWorld) {
  for (int i = 0; i < collisionWorld->numOfLines; i++) {
    Line *line = collisionWorld->lines[i];
    bool collide = false;

    // Right side
    if ((line->p1.x > BOX_XMAX || line->p2.x > BOX_XMAX)
        && (line->velocity.x > 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Left side
    if ((line->p1.x < BOX_XMIN || line->p2.x < BOX_XMIN)
        && (line->velocity.x < 0)) {
      line->velocity.x = -line->velocity.x;
      collide = true;
    }
    // Top side
    if ((line->p1.y > BOX_YMAX || line->p2.y > BOX_YMAX)
        && (line->velocity.y > 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Bottom side
    if ((line->p1.y < BOX_YMIN || line->p2.y < BOX_YMIN)
        && (line->velocity.y < 0)) {
      line->velocity.y = -line->velocity.y;
      collide = true;
    }
    // Update total number of collisions.
    if (collide == true) {
      collisionWorld->numLineWallCollisions++;
    }
  }
}

QuadTree* build_quadtree(CollisionWorld* cw) {
  double x1 = BOX_XMIN;
  double x2 = BOX_XMAX;
  double y1 = BOX_YMIN;
  double y2 = BOX_YMAX;
  double timeStep = cw->timeStep;

  QuadTree* q = QuadTree_make(x1, x2, y1, y2);

  int n = cw->numOfLines;
  if (n <= N) {
    for (int i = 0; i < n; i++) {
      LineList_addLineNode(q->lines, cw->line_nodes[i]);
    }
    return q;
  }

  LineList *quad1, *quad2, *quad3, *quad4;
  quad1 = LineList_make();
  quad2 = LineList_make();
  quad3 = LineList_make();
  quad4 = LineList_make();

  LineNode* curr;
  int type;
  for (int i = 0; i < n; i++) {
    update_box(cw->lines[i], timeStep);
    curr = cw->line_nodes[i];
    type = QuadTree_getQuad(q, curr, timeStep);
    switch (type) {
      case 1:
        LineList_addLineNode(quad1, curr);
        break;
      case 2:
        LineList_addLineNode(quad2, curr);
        break;
      case 3:
        LineList_addLineNode(quad3, curr);
        break;
      case 4:
        LineList_addLineNode(quad4, curr);
        break;
      case MULTIPLE_QUADS:
        LineList_addLineNode(q->lines, curr);
        break;
      default:
        return NULL;
    }
  }

  // Calculate midpoint
  double x = (x1 + x2) / 2;
  double y = (y1 + y2) / 2;

  // TOP LEFT
  if (quad1->head) {
    q->quads[0] = QuadTree_make(x1, x, y1, y);
    QuadTree_addLines(q->quads[0], quad1, timeStep);
  }

  // TOP RIGHT
  if (quad2->head) {
    q->quads[1] = QuadTree_make(x, x2, y1, y);
    QuadTree_addLines(q->quads[1], quad2, timeStep);
  }

  // BOTTOM LEFT
  if (quad3->head) {
    q->quads[2] = QuadTree_make(x1, x, y, y2);
    QuadTree_addLines(q->quads[2], quad3, timeStep);
  }

  // BOTTOM RIGHT
  if (quad4->head) {
    q->quads[3] = QuadTree_make(x, x2, y, y2);
    QuadTree_addLines(q->quads[3], quad4, timeStep);
  }

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
    second_node = lines->head;
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

  LineList_concat(q->lines, lines);

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
