#include "./Quadtree.h"

void Quadtree_init(Quadtree* q, int level, window_dimension x, window_dimension y) {
  q->level = level;
  q->x = x;
  q->y = y;
  q->nodes = (Quadtree*)malloc(5*sizeof(Quadtree));
}

void Quadtree_delete(Quadtree* q) {
  for(int i = 0; i < N; i++) {
    Quadtree_delete((q->nodes)+i);
  }
  if(q->nodes)
    free(q->nodes);
}

void Quadtree_partition(Quadtree* q) {
  int width = WINDOW_WIDTH / (1 << q->level);
  int height = WINDOW_HEIGHT / (1 << q->level);

  q->nodes[0] = { .level = q->level + 1,
                  .num_objects = 0,
                  .x = q->x,
                  .y = q->y,
                  .objects = NULL,
                  .nodes = NULL};

  q->nodes[1] = { .level = q->level + 1,
                  .num_objects = 0,
                  .x = q->x + width / 2,
                  .y = q->y,
                  .objects = NULL,
                  .nodes = NULL};

  q->nodes[2] = { .level = q->level + 1,
                  .num_objects = 0,
                  .x = q->x,
                  .y = q->y + height / 2,
                  .objects = NULL,
                  .nodes = NULL};

  q->nodes[1] = { .level = q->level + 1,
                  .num_objects = 0,
                  .x = q->x + width / 2,
                  .y = q->y + height / 2,
                  .objects = NULL,
                  .nodes = NULL};
}

bool paralellogramInBox(Parallelogram* p, int x, int y, int width, int height) {
  Vec p1 = {.x = x, .y = y};
  Vec p2 = {.x = x + width, .y = y};
  Vec p3 = {.x = x, .y = y + height};
  Vec p4 = {.x = x + width, .y = y + height};
  for(int i = 0; i < 4; i++) {
    if(!pointInParallelogram(p->points[i], p1, p2, p3, p4))
      return false;
  }
  return true;
}

int Quadtree_getIndex(Quadtree* q, Parallelogram* p) {
  int index = -1;
  int width = WINDOW_WIDTH / (1 << q->level);
  int height = WINDOW_HEIGHT / (1 << q->level);

  if(parallelogramInBox(p, q->x, q->y, width/2, height/2))
    return 0;
  if(parallelogramInBox(p, q->x + width/2, q->y, width/2, height/2))
    return 1;
  if(parallelogramInBox(p, q->x, q->y + width/2, width/2, height/2))
    return 2;
  if(parallelogramInBox(p, q->x + width/2, q->y + width/2, width/2, height/2))
    return 3;
  return -1;
}

void Quadtree_insert(Quadtree* q, Parallelogram* p) {
  int index = Quadtree_getIndex(q, p);
  if(index != -1) {
    Quadtree_insert(q->nodes+index, p);
    return;
  }
  q->objects[q->num_objects] = *p;
  q->num_objects++;

  if(q->num_objects >= N) {
    Quadtree_partition(q);
    for(int i = 0; i <= N; i++) {
      int index = Quadtree_getIndex(q, q->objects+i);
      if(index != -1) {
        Quadtree_insert(q->nodes+index, q->objects+i);
        q->num_objects--;
      }
    }
  }
}

void Quadt

