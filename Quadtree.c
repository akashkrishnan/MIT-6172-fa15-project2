#include "./Quadtree.h"

#include <string.h>
#include <assert.h>
#include <cilk/cilk.h>
#include <cilk/reducer.h>
//#include <cilk/cilk_stub.h>

#include "./Line.h"
#include "./Vec.h"
#include "./IntersectionEventList.h"

inline void LineList_addLine(LineList* ll, Line* l) {
  assert(ll);
  assert(l);
  l->next = NULL;
  if (ll->tail) {
    ll->tail->next = l;
    ll->tail = l;
  } else {
    ll->head = ll->tail = l;
  }
  ll->count++;
}

inline void LineList_concat(LineList* l, LineList* r) {
  assert(l);
  assert(r);
  if (r->head == NULL) return;
  if (l->head) {
    l->count += r->count;
    l->tail->next = r->head;
    l->tail = r->tail;
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
    memset(q->quads[0]->lines, 0, sizeof(LineList));
    memset(q->quads[1]->lines, 0, sizeof(LineList));
    memset(q->quads[2]->lines, 0, sizeof(LineList));
    memset(q->quads[3]->lines, 0, sizeof(LineList));
  }
  memset(q->lines, 0, sizeof(LineList));
  q->empty = false;
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

inline int QuadTree_getQuad(double x, double y, Line* l, double t) {
  assert(l);

  int q_a = QuadTree_getQuadWithLine(x, y, l->p1, l->p2);
  int q_b = QuadTree_getQuadWithLine(x, y, l->p3, l->p4);
  return q_a == q_b ? q_a : 4;
}

inline void QuadTree_addLines(QuadTree* q, double t) {
  assert(q);

  // Check if node can fit all the lines
  if (q->lines->count <= N) {
    q->empty = true;
    return;
  }

  assert(q->quads);

  // Put lines in appropriate line lists
  Line* curr = q->lines->head;
  Line* next;
  int type;
  QuadTree_reset(q);
  while (curr) {
    next = curr->next;
    type = QuadTree_getQuad(q->x0, q->y0, curr, t);
    assert(0 <= type && type < 5);
    if (type == 4) {
      LineList_addLine(q->lines, curr);
    } else {
      LineList_addLine(q->quads[type]->lines, curr);
    }
    curr = next;
  }
  QuadTree_addLines(q->quads[0], t);
  QuadTree_addLines(q->quads[1], t);
  QuadTree_addLines(q->quads[2], t);
  QuadTree_addLines(q->quads[3], t);
}

void QuadTree_detectEvents(QuadTree* q,
                           LineList* lines,
                           double t,
                           IntersectionEventListReducer* iel) {
  if (!q) {
    return;
  }

  Line *l1, *l2;

  l1 = q->lines->head;
  for (int i = 0; i < q->lines->count; i++, l1 = l1->next) {
    l2 = l1->next;
    for (int j = i + 1; j < q->lines->count; j++, l2 = l2->next) {
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
    }
  }

  if (lines && lines->count) {
    l1 = q->lines->head;
    for (int i = 0; i < q->lines->count; i++, l1 = l1->next) {
      l2 = lines->head;
      for (int j = 0; j < lines->count; j++, l2 = l2->next) {
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
      }
    }

    LineList_concat(q->lines, lines);
  }

  if (!q->empty) {
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
