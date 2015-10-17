#include "./Quadtree.h"

#include <string.h>
#include <assert.h>
#include <cilk/cilk.h>
#include <cilk/reducer.h>
//#include <cilk/cilk_stub.h>

#include "./Line.h"
#include "./Vec.h"
#include "./IntersectionEventList.h"

inline LineNode* LineNode_make(Line* l) {
  LineNode* ln = malloc(sizeof(LineNode));
  ln->line = l;
  ln->next = NULL;
  return ln;
}

inline void LineNode_delete(LineNode* ln) {
  assert(ln);
  free(ln);
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
  q->quads = NULL;
  q->lines = calloc(1, sizeof(LineList));
  q->x1 = x1;
  q->x2 = x2;
  q->y1 = y1;
  q->y2 = y2;
  q->x0 = (q->x1 + q->x2) / 2;
  q->y0 = (q->y1 + q->y2) / 2;
  q->empty = true;
  return q;
}

inline void QuadTree_delete(QuadTree* q) {
  assert(q);

  if (q->quads) {
    QuadTree_delete(q->quads[0]);
    QuadTree_delete(q->quads[1]);
    QuadTree_delete(q->quads[2]);
    QuadTree_delete(q->quads[3]);
    free(q->quads);
  }
  free(q->lines);
  free(q);
}

inline void QuadTree_reset(QuadTree* q) {
  assert(q);

  if (q->quads) {
    QuadTree_reset(q->quads[0]);
    QuadTree_reset(q->quads[1]);
    QuadTree_reset(q->quads[2]);
    QuadTree_reset(q->quads[3]);
    //memset(q->quads[0]->lines, 0, sizeof(LineList));
    //memset(q->quads[1]->lines, 0, sizeof(LineList));
    //memset(q->quads[2]->lines, 0, sizeof(LineList));
    //memset(q->quads[3]->lines, 0, sizeof(LineList));
  }
  memset(q->lines, 0, sizeof(LineList));
  q->empty = true;
}

inline void QuadTree_build(QuadTree* q, int depth) {
  assert(q);

  if (depth > 0) {
    q->quads = malloc(4 * sizeof(QuadTree*));
    q->quads[0] = QuadTree_make(q->x1, q->x0, q->y1, q->y0);
    q->quads[1] = QuadTree_make(q->x0, q->x2, q->y1, q->y0);
    q->quads[2] = QuadTree_make(q->x1, q->x0, q->y0, q->y2);
    q->quads[3] = QuadTree_make(q->x0, q->x2, q->y0, q->y2);
    QuadTree_build(q->quads[0], depth - 1);
    QuadTree_build(q->quads[1], depth - 1);
    QuadTree_build(q->quads[2], depth - 1);
    QuadTree_build(q->quads[3], depth - 1);
  }
}

inline int QuadTree_getQuadWithLine(double x, double y, Vec p1, Vec p2) {
  // Determine if the line cannot be in a single child
  if (!(((p1.x - x) * (p2.x - x) > 0) &&
        ((p1.y - y) * (p2.y - y) > 0))) {
    return 4;
  }

  // Determine what child the line is in
  int xid = p1.x - x > 0;
  int yid = p1.y - y > 0;
  return 2 * yid + xid;
}

inline int QuadTree_getQuad(double x, double y, LineNode* ln, double t) {
  assert(ln);
  assert(ln->line);

  Vec p1_a = ln->line->p1;
  Vec p2_a = ln->line->p2;

  // TODO: POSSIBLY STORE IN LINE?
  Vec delta = Vec_multiply(ln->line->velocity, t);
  Vec p1_b = Vec_add(p1_a, delta);
  Vec p2_b = Vec_add(p2_a, delta);

  int q_a = QuadTree_getQuadWithLine(x, y, p1_a, p2_a);
  int q_b = QuadTree_getQuadWithLine(x, y, p1_b, p2_b);
  return q_a == q_b ? q_a : 4;
}

inline void QuadTree_addLines(QuadTree* q, double t) {
  assert(q);

  // Check if node can fit all the lines
  if (q->lines->count <= N) {
    q->empty = true;
    return;
  }

  q->empty = false;

  // Put lines in appropriate line lists
  LineNode* curr = q->lines->head;
  LineNode* next;
  int type;
  //QuadTree_reset(q);
  memset(q->lines, 0, sizeof(LineList));
  while (curr) {
    assert(curr->line);

    next = curr->next;
    type = QuadTree_getQuad(q->x0, q->y0, curr, t);
    assert(0 <= type && type < 5);
    if (type == 4) {
      LineList_addLineNode(q->lines, curr);
    } else {
      LineList_addLineNode(q->quads[type]->lines, curr);
    }
    curr = next;
  }

  for (int i = 0; i < 4; i++) {
    QuadTree_addLines(q->quads[i], t);
  }
}

void QuadTree_detectEvents(QuadTree* q,
                           LineList* lines,
                           double t,
                           IntersectionEventListReducer* iel) {
  if (!q) {
    return;
  }

  LineNode *first_node, *second_node;
  Line *l1, *l2;

  first_node = q->lines->head;
  while (first_node) {
    l1 = first_node->line;
    second_node = first_node->next;
    while (second_node) {
      l2 = second_node->line;

      if (compareLines(l1, l2) < 0) {
        IntersectionType type = intersect(l1, l2, t);
        if (type != NO_INTERSECTION) {
          IntersectionEventList_appendNode(&REDUCER_VIEW(*iel), l1, l2, type);
        }
      } else {
        IntersectionType type = intersect(l2, l1, t);
        if (type != NO_INTERSECTION) {
          IntersectionEventList_appendNode(&REDUCER_VIEW(*iel), l2, l1, type);
        }
      }

      second_node = second_node->next;
    }
    first_node = first_node->next;
  }

  if (lines && lines->count) {
    first_node = q->lines->head;
    while (first_node) {
      l1 = first_node->line;
      second_node = lines->head;
      while (second_node) {
        l2 = second_node->line;

        if (compareLines(l1, l2) < 0) {
          IntersectionType type = intersect(l1, l2, t);
          if (type != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*iel), l1, l2, type);
          }
        } else {
          IntersectionType type = intersect(l2, l1, t);
          if (type != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&REDUCER_VIEW(*iel), l2, l1, type);
          }
        }

        second_node = second_node->next;
      }
      first_node = first_node->next;
    }

    LineList_concat(q->lines, lines);
  }

  if (q->quads) {
    if (q->lines->count > MAX_INTERSECTS) {
      cilk_for (int i = 0; i < 4; i++) {
        QuadTree_detectEvents(q->quads[i], q->lines, t, iel);
      }
    } else {
      QuadTree_detectEvents(q->quads[0], q->lines, t, iel);
      QuadTree_detectEvents(q->quads[1], q->lines, t, iel);
      QuadTree_detectEvents(q->quads[2], q->lines, t, iel);
      QuadTree_detectEvents(q->quads[3], q->lines, t, iel);
    }
  }
}
