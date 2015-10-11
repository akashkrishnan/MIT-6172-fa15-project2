#include "./Quadtree.h"

#include <assert.h>
#include <cilk/cilk.h>

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
  q->quads = calloc(4, sizeof(QuadTree*));
  q->lines = NULL;
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
    free(q->lines);
    free(q);
  }
}

inline int QuadTree_getQuadWithLine(double x, double y, Vec p1, Vec p2) {
  // Determine if the line cannot be in a single child
  if (!(((p1.x - x) * (p2.x - x) > 0) &&
        ((p1.y - y) * (p2.y - y) > 0))) {
    return 0;
  }

  // Determine what child the line is in
  int xid = p1.x - x > 0;
  int yid = p1.y - y > 0;
  return 2 * yid + xid + 1;
}

inline int QuadTree_getQuad(double x, double y, LineNode* ln, double t) {
  assert(ln);
  assert(ln->line);

  Vec p1_a = ln->line->p1;
  Vec p2_a = ln->line->p2;

  Vec p1_b = Vec_add(p1_a, Vec_multiply(ln->line->velocity, t));
  Vec p2_b = Vec_add(p2_a, Vec_multiply(ln->line->velocity, t));

  int q_a = QuadTree_getQuadWithLine(x, y, p1_a, p2_a);
  int q_b = QuadTree_getQuadWithLine(x, y, p1_b, p2_b);
  return q_a == q_b ? q_a : 0;
}

inline void QuadTree_addLines(QuadTree* q, LineList* ll, double t) {
  assert(q);
  assert(ll);

  // Check if node can fit all the lines
  if (ll->count <= N) {
    q->lines = ll;
    return;
  }

  // Calculate midpoint
  double x = (q->x1 + q->x2) / 2;
  double y = (q->y1 + q->y2) / 2;

  // Make line lists, four for children and one for parent
  // TODO: potentially malloc this once in cw and memset every time in here
  LineList** lists = malloc(5 * sizeof(LineList*));
  for (int i = 0; i < 5; i++) {
    lists[i] = calloc(1, sizeof(LineList));
  }

  // Put lines in appropriate line lists
  LineNode* curr = ll->head;
  LineNode* next;
  int type;
  while (curr) {
    assert(curr->line);

    next = curr->next;
    type = QuadTree_getQuad(x, y, curr, t);
    LineList_addLineNode(lists[type], curr);
    curr = next;
  }

  // PARENT
  q->lines = lists[0];

  // TOP LEFT CHILD
  if (lists[1]->head) {
    q->quads[0] = QuadTree_make(q->x1, x, q->y1, y);
    QuadTree_addLines(q->quads[0], lists[1], t);
  }

  // TOP RIGHT CHILD
  if (lists[2]->head) {
    q->quads[1] = QuadTree_make(x, q->x2, q->y1, y);
    QuadTree_addLines(q->quads[1], lists[2], t);
  }

  // BOTTOM LEFT CHILD
  if (lists[3]->head) {
    q->quads[2] = QuadTree_make(q->x1, x, y, q->y2);
    QuadTree_addLines(q->quads[2], lists[3], t);
  }

  // BOTTOM RIGHT CHILD
  if (lists[4]->head) {
    q->quads[3] = QuadTree_make(x, q->x2, y, q->y2);
    QuadTree_addLines(q->quads[3], lists[4], t);
  }

  free(ll);
  free(lists);
}

IntersectionEventList QuadTree_detectEvents(QuadTree* q,
                                            LineList* lines,
                                            double t) {
  IntersectionEventList iel = IntersectionEventList_make();

  if (!q) {
    return iel;
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
          IntersectionEventList_appendNode(&iel, l1, l2, type);
        }
      } else {
        IntersectionType type = intersect(l2, l1, t);
        if (type != NO_INTERSECTION) {
          IntersectionEventList_appendNode(&iel, l2, l1, type);
        }
      }

      second_node = second_node->next;
    }
    first_node = first_node->next;
  }

  if (lines && lines->head) {
    first_node = q->lines->head;
    while (first_node) {
      l1 = first_node->line;
      second_node = lines->head;
      while (second_node) {
        l2 = second_node->line;

        if (compareLines(l1, l2) < 0) {
          IntersectionType type = intersect(l1, l2, t);
          if (type != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&iel, l1, l2, type);
          }
        } else {
          IntersectionType type = intersect(l2, l1, t);
          if (type != NO_INTERSECTION) {
            IntersectionEventList_appendNode(&iel, l2, l1, type);
          }
        }

        second_node = second_node->next;
      }
      first_node = first_node->next;
    }

    LineList_concat(q->lines, lines);
  }

  IntersectionEventList iel0, iel1, iel2, iel3;

  if (q->lines->count > MAX_INTERSECTS) {
    iel0 = cilk_spawn QuadTree_detectEvents(q->quads[0], q->lines, t);
    iel1 = cilk_spawn QuadTree_detectEvents(q->quads[1], q->lines, t);
    iel2 = cilk_spawn QuadTree_detectEvents(q->quads[2], q->lines, t);
    iel3 =            QuadTree_detectEvents(q->quads[3], q->lines, t);
    cilk_sync;
  } else {
    iel0 = QuadTree_detectEvents(q->quads[0], q->lines, t);
    iel1 = QuadTree_detectEvents(q->quads[1], q->lines, t);
    iel2 = QuadTree_detectEvents(q->quads[2], q->lines, t);
    iel3 = QuadTree_detectEvents(q->quads[3], q->lines, t);
  }

  IntersectionEventList_concat(&iel, &iel0);
  IntersectionEventList_concat(&iel, &iel1);
  IntersectionEventList_concat(&iel, &iel2);
  IntersectionEventList_concat(&iel, &iel3);

  return iel;
}
