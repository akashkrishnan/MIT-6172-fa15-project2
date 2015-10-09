#include "./Quadtree.h"

#include <assert.h>

#include "./Line.h"
#include "./Vec.h"

inline LineNode* LineNode_make(Line* l) {
  LineNode* ln = malloc(sizeof(LineNode));
  ln->line = l;
  ln->next = NULL;
  return ln;
}

inline void LineNode_delete(LineNode* ln) {
  free(ln);
}


inline LineList* LineList_make() {
  return calloc(1, sizeof(LineList));
}

// TOOD: CAN BE OPTIMIZED BY USING FOR LOOP?
inline void LineList_delete(LineList* ll) {
  assert(ll);
  LineNode* ln = ll->head;
  while (ln) {
    LineNode_delete(ln);
    ln = ln->next;
  }
  free(ll);
}

inline void LineList_addLineNode(LineList* ll, LineNode* ln) {
  assert(ll);
  assert(ln);
  ln->next = NULL;
  if (ll->tail) {
    ll->tail->next = ln;
    ll->tail = ln;
  } else {
    ll->head = ll->tail = ln;
  }
  ll->count++;
}

inline void LineList_concat(LineList* l, LineList* r) {
  assert(l);
  assert(r);
  if (l->head) {
    l->count += r->count;
    l->tail->next = r->head;
    if (r->tail) {
      l->count += r->count;
    }
  } else {
    *l = *r;
  }
}


inline QuadTree* QuadTree_make(double x1, double x2, double y1, double y2) {
  QuadTree* q = malloc(sizeof(QuadTree));
  q->quads = calloc(4, sizeof(QuadTree*));
  q->lines = LineList_make();
  q->x1 = x1;
  q->x2 = x2;
  q->y1 = y1;
  q->y2 = y2;
  return q;
}

inline void QuadTree_delete(QuadTree* q) {
  if (q) {
    QuadTree_delete(q->quads[0]);
    QuadTree_delete(q->quads[1]);
    QuadTree_delete(q->quads[2]);
    QuadTree_delete(q->quads[3]);
    free(q->quads);
    LineList_delete(q->lines);
    free(q);
  }
}

inline int QuadTree_getQuadWithLine(QuadTree* q, Vec p1, Vec p2) {
  // Bounding box
  double x1 = q->x1;
  double x2 = q->x2;
  double y1 = q->y1;
  double y2 = q->y2;

  // Midpoint
  double x = (x1 + x2) / 2;
  double y = (y1 + y2) / 2;

  // Determine if line exits quad
  if (!(((p1.x - x) * (p2.x - x) > 0) &&
        ((p1.y - y) * (p2.y - y) > 0))) {
    return MULTIPLE_QUADS;
  }

  // Valid quad values are from 1 to 4
  int xid = p1.x - x > 0;
  int yid = p1.y - y > 0;
  int quad = 2 * yid + xid + 1;
  return quad;
}

inline int QuadTree_getQuad(QuadTree* q, LineNode* ln, double timeStep) {
  assert(q);
  assert(ln);
  assert(ln->line);

  Vec p1_a = ln->line->p1;
  Vec p2_a = ln->line->p2;

  Vec p1_b = Vec_add(p1_a, Vec_multiply(ln->line->velocity, timeStep));
  Vec p2_b = Vec_add(p2_a, Vec_multiply(ln->line->velocity, timeStep));

  int q_a = QuadTree_getQuadWithLine(q, p1_a, p2_a);
  int q_b = QuadTree_getQuadWithLine(q, p1_b, p2_b);
  return q_a == q_b ? q_a : MULTIPLE_QUADS;
}

inline void QuadTree_addLines(QuadTree* q, LineList* ll, double timeStep) {
  assert(q);
  assert(ll);

  // Check if node can fit all the lines
  if (ll->count <= N) {
    q->lines = ll;
    return;
  }

  double x1 = q->x1;
  double x2 = q->x2;
  double y1 = q->y1;
  double y2 = q->y2;

  LineList *quad1, *quad2, *quad3, *quad4;
  quad1 = LineList_make();
  quad2 = LineList_make();
  quad3 = LineList_make();
  quad4 = LineList_make();

  LineNode* curr = ll->head;
  int type;
  while (curr) {
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
        return;
    }
    curr = curr->next;
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
}

