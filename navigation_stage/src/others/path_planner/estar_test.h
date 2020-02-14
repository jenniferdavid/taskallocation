#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


#define CALC_KEY(cell) ((cell)->rhs < (cell)->phi ? (cell)->rhs : (cell)->phi)
#define estar_grid_at(grid,ix,iy) (&(grid)->cell[(ix)+(iy)*(grid)->dimx])

enum 
{
  ESTAR_FLAG_GOAL     = 1,
  ESTAR_FLAG_OBSTACLE = 2
};


typedef struct estar_cell_s 
{
  double cost;			 /* set this to 1/speed for "sensible" values */
  double phi;
  double rhs;
  double key;			 /* managed by pqueue */
  size_t pqi;			 /* managed by pqueue; pqi==0 means "not on queue" */
  int flags;
  struct estar_cell_s * nbor[5]; /* null-terminated array of neighbors */
  struct estar_cell_s * prop[9]; /* null-terminated array of pairwise propagators */
} estar_cell_t;

typedef struct 
{
  estar_cell_t * cell;
  size_t dimx, dimy;
} estar_grid_t;

typedef struct 
{
  estar_cell_t ** heap;
  size_t len, cap;
} estar_pqueue_t;

typedef struct 
{
  estar_grid_t grid;   // estar_grid_t is structure included in grid.h
  estar_pqueue_t pq;   // estar_pqueue_t is structure included in pqueue.h
} estar_t;

void estar_dump_queue (estar_t * estar, char const * pfx);
void estar_propagate (estar_t * estar);
void estar_update (estar_t * estar, estar_cell_t * cell);
void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed);
void estar_set_goal (estar_t * estar, size_t ix, size_t iy);
void estar_reset (estar_t * estar);
void estar_init(estar_t * estar, size_t dimx, size_t dimy);
void estar_fini (estar_t * estar);

void estar_pqueue_init (estar_pqueue_t * pq, size_t cap);
void estar_pqueue_fini (estar_pqueue_t * pq);
void estar_pqueue_insert_or_update (estar_pqueue_t * pq, estar_cell_t * cell);
void estar_pqueue_remove_or_ignore (estar_pqueue_t * pq, estar_cell_t * cell);

void estar_grid_init (estar_grid_t * grid, size_t dimx, size_t dimy);
void estar_grid_fini (estar_grid_t * grid);
void calc_rhs (estar_cell_t * cell, double phimax);
void estar_grid_dump_cell (estar_grid_t * grid, estar_cell_t const * cell, char const * pfx);
void swap (estar_cell_t ** aa, estar_cell_t ** bb);
void bubble_up (estar_cell_t ** heap, size_t index);
void bubble_down (estar_cell_t ** heap, size_t len, size_t index);

int estar_check (estar_t * estar, char const * pfx);
int estar_cell_calc_gradient (estar_cell_t * cell, double * gx, double * gy);

double estar_pqueue_topkey (estar_pqueue_t * pq);
double interpolate (double cost, double primary, double secondary);

estar_cell_t * estar_pqueue_extract (estar_pqueue_t * pq);