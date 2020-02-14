#include "estar_test.h"

void estar_dump_queue (estar_t * estar, char const * pfx)
{
  size_t ii;
  for (ii = 1; ii <= estar->pq.len; ++ii) 
  {
    printf ("%s[%zu %zu]  pqi:  %zu  key: %g  phi: %g  rhs: %g\n",
	    pfx,
	    (estar->pq.heap[ii] - estar->grid.cell) % estar->grid.dimx,
	    (estar->pq.heap[ii] - estar->grid.cell) / estar->grid.dimx,
	    estar->pq.heap[ii]->pqi, estar->pq.heap[ii]->key,
	    estar->pq.heap[ii]->phi, estar->pq.heap[ii]->rhs);
  }
}

void estar_propagate (estar_t * estar)
{
  estar_cell_t * cell;
  estar_cell_t ** nbor;
  
  cell = estar_pqueue_extract (&estar->pq);
  if (NULL == cell) {
    return;
  }
  
  // The chunk below could be placed into a function called expand,
  // but it is not needed anywhere else.
  
  if (cell->phi > cell->rhs) {
    cell->phi = cell->rhs;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update (estar, *nbor);
    }
  }
  else {
    cell->phi = INFINITY;
    for (nbor = cell->nbor; *nbor != 0; ++nbor) {
      estar_update (estar, *nbor);
    }
    estar_update (estar, cell);
  }
}


void estar_update (estar_t * estar, estar_cell_t * cell)
{
  /* XXXX check whether obstacles actually can end up being
     updated. Possibly due to effects of estar_set_speed? */
  if (cell->flags & ESTAR_FLAG_OBSTACLE) {
    estar_pqueue_remove_or_ignore (&estar->pq, cell);
    return;
  }
  
  /* Make sure that goal cells remain at their rhs, which is supposed
     to be fixed and only serve as source for propagation, never as
     sink. */
  if ( ! (cell->flags & ESTAR_FLAG_GOAL)) {
    calc_rhs (cell, estar_pqueue_topkey (&estar->pq));
  }
  
  if (cell->phi != cell->rhs) {
    estar_pqueue_insert_or_update (&estar->pq, cell);
  }
  else {
    estar_pqueue_remove_or_ignore (&estar->pq, cell);
  }
}

void estar_set_speed (estar_t * estar, size_t ix, size_t iy, double speed)
{
  double cost;
  estar_cell_t * cell;
  estar_cell_t ** nbor;

  cell = estar_grid_at (&estar->grid, ix, iy);
  
  // XXXX I'm undecided yet whether this check here makes the most
  // sense. The other option is to make sure that the caller doesn't
  // place obstacles into a goal cell. The latter somehow makes more
  // sense to me at the moment, so in gestar.c there is code to filter
  // goal cells from the obstacle setting routines.
  ////  if (cell->flags & ESTAR_FLAG_GOAL) {
  ////    return;
  ////  }
  
  if (speed <= 0.0) {
    cost = INFINITY;
  }
  else {
    cost = 1.0 / speed;
  }
  if (cost == cell->cost) {
    return;
  }
  
  cell->cost = cost;
  if (speed <= 0.0) {
    cell->phi = INFINITY;
    cell->rhs = INFINITY;
    cell->flags |= ESTAR_FLAG_OBSTACLE;
  }
  else {
    cell->flags &= ~ESTAR_FLAG_OBSTACLE;
  }
  
  estar_update (estar, cell);
  for (nbor = cell->nbor; *nbor != 0; ++nbor) {
    estar_update (estar, *nbor);
  }
}



void estar_set_goal (estar_t * estar, size_t ix, size_t iy)
{
  estar_cell_t * goal = estar_grid_at (&estar->grid, ix, iy);
  goal->rhs = 0.0;
  goal->flags |= ESTAR_FLAG_GOAL;
  goal->flags &= ~ESTAR_FLAG_OBSTACLE;
  estar_pqueue_insert_or_update (&estar->pq, goal);
}

void estar_reset (estar_t * estar)
{
  size_t const ncells = estar->grid.dimx * estar->grid.dimy;
  size_t ii;
  estar_cell_t * cell;
  
  for (ii = 0, cell = estar->grid.cell; ii < ncells; ++ii, ++cell) {
    cell->phi = INFINITY;
    cell->rhs = INFINITY;
    cell->key = INFINITY;
    cell->pqi = 0;
    cell->flags &= ~ESTAR_FLAG_GOAL;
  }
  
  estar->pq.len = 0;
}

void estar_fini (estar_t * estar)
{
  estar_grid_fini (&estar->grid);
  estar_pqueue_fini (&estar->pq);
}

void estar_init (estar_t * estar, size_t dimx, size_t dimy)
{
  estar_grid_init (&estar->grid, dimx, dimy);
  estar_pqueue_init (&estar->pq, dimx + dimy);
}


void estar_pqueue_init (estar_pqueue_t * pq, size_t cap)
{
  pq->heap = malloc (sizeof(estar_cell_t*) * (cap+1));
  if (NULL == pq->heap) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc", __func__);
  }
  pq->len = 0;
  pq->cap = cap;
}


void estar_pqueue_fini (estar_pqueue_t * pq)
{
  free (pq->heap);
  pq->len = 0;
  pq->cap = 0;
}


void estar_pqueue_insert_or_update (estar_pqueue_t * pq, estar_cell_t * cell)
{
  size_t len;
  estar_cell_t ** heap;
  
  if (0 != cell->pqi) {
    cell->key = CALC_KEY(cell);
    // could probably make it more efficient by only bubbling down when
    // the bubble up did not change cell->pqi
    bubble_up (pq->heap, cell->pqi);
    bubble_down (pq->heap, pq->len, cell->pqi);
    return;
  }
  
  // grow heap, realloc if necessary
  
  len = pq->len + 1;
  if (len <= pq->cap) {
    heap = pq->heap;
  }
  else {
    size_t cap;
    cap = 2 * pq->cap;
    heap = realloc (pq->heap, sizeof(estar_cell_t*) * (cap+1));
    if (NULL == heap) {
      errx (EXIT_FAILURE, __FILE__": %s: realloc", __func__);
    }
    pq->heap = heap;
    pq->cap = cap;
  }
  pq->len = len;
  
  // append cell to heap and bubble up
  
  cell->key = CALC_KEY(cell);
  heap[len] = cell;
  cell->pqi = len;		/* initialize pqi */
  bubble_up (heap, len);
}


void estar_pqueue_remove_or_ignore (estar_pqueue_t * pq, estar_cell_t * cell)
{
  if (0 == cell->pqi) {
    // This could be done by the caller for efficiency, but it is much
    // more convenient to do it here.
    return;
  }
  
  pq->heap[cell->pqi] = pq->heap[pq->len];
  pq->heap[cell->pqi]->pqi = cell->pqi; /* keep pqi consistent! */
  --pq->len;
  bubble_down (pq->heap, pq->len, cell->pqi);
  cell->pqi = 0;		/* mark cell as not on queue */
}

void estar_grid_init (estar_grid_t * grid, size_t dimx, size_t dimy)
{
  size_t ix, iy;
  estar_cell_t * cell;
  estar_cell_t ** nbor;
  
  grid->cell = malloc (sizeof(estar_cell_t) * dimx * dimy);
  if (NULL == grid->cell) {
    errx (EXIT_FAILURE, __FILE__": %s: malloc", __func__);
  }
  grid->dimx = dimx;
  grid->dimy = dimy;
  
  for (ix = 0; ix < dimx; ++ix) {
    for (iy = 0; iy < dimy; ++iy) {
      cell = estar_grid_at(grid, ix, iy);

      cell->cost = 1.0;
      cell->phi = INFINITY;
      cell->rhs = INFINITY;
      cell->key = INFINITY;
      cell->pqi = 0;
      cell->flags = 0;

      nbor = cell->nbor;
      if (ix > 0) {		/* west */
	*(nbor++) = cell - 1;
      }
      if (ix < dimx - 1) {	/* east */
	*(nbor++) = cell + 1;
      }
      if (iy > 0) {		/* south */
	*(nbor++) = cell - dimx;
      }
      if (iy < dimy - 1) {	/* north */
	*(nbor++) = cell + dimx;
      }
      *nbor = 0;
      
      nbor = cell->prop;
      if (ix > 0) {
	if (iy > 0) {		/* south-west */
	  *(nbor++) = cell - 1;
	  *(nbor++) = cell - dimx;
	}
	if (iy < dimy - 1) {	/* north-west */
	  *(nbor++) = cell - 1;
	  *(nbor++) = cell + dimx;
	}
      }
      if (ix < dimx - 1) {
	if (iy > 0) {		/* south-east */
	  *(nbor++) = cell + 1;
	  *(nbor++) = cell - dimx;
	}
	if (iy < dimy - 1) {	/* north-east */
	  *(nbor++) = cell + 1;
	  *(nbor++) = cell + dimx;
	}
      }
      *nbor = 0;
    }
  }
}


void estar_grid_fini (estar_grid_t * grid)
{
  free (grid->cell);
  grid->dimx = 0;
  grid->dimy = 0;
}

void calc_rhs (estar_cell_t * cell, double phimax)
{
  estar_cell_t ** prop;
  estar_cell_t * primary;
  estar_cell_t * secondary;
  double rr;
  
  cell->rhs = INFINITY;
  prop = cell->prop;
  while (NULL != *prop) {
    
    primary = *(prop++);
    if (primary->rhs <= (*prop)->rhs)  {
      secondary = *(prop++);
    }
    else {
      secondary = primary;
      primary = *(prop++);
    }
    
    // do not propagate from obstacles, queued cells, cells above the
    // wavefront, or cells at infinity
    if (primary->flags & ESTAR_FLAG_OBSTACLE
	|| primary->pqi != 0
	|| primary->phi > phimax
	|| isinf(primary->phi)) {
      continue;
    }
    
    // the same goes from the secondary, but if that fails at least we
    // can fall back to the non-interpolated update equation.
    if (secondary->flags & ESTAR_FLAG_OBSTACLE
	|| secondary->pqi != 0
	|| secondary->phi > phimax
	|| isinf(secondary->phi)) {
      rr = primary->rhs + cell->cost;
    }
    else {
      rr = interpolate (cell->cost, primary->phi, secondary->phi);
    }
    
    if (rr < cell->rhs) {
      cell->rhs = rr;
    }
  }
  
  if (isinf (cell->rhs)) {
    // None of the above worked, we're probably done... but I have
    // lingering doubts about about the effects of in-place primary /
    // secondary sorting above, it could be imagined to create
    // situations where we overlook something. So, just to be on the
    // safe side, let's retry all non-interpolated options.
    for (prop = cell->nbor; *prop != 0; ++prop) {
      rr = (*prop)->phi;
      if (rr < cell->rhs) {
	cell->rhs = rr;
      }
    }
    cell->rhs += cell->cost;
  }
}


void estar_grid_dump_cell (estar_grid_t * grid, estar_cell_t const * cell, char const * pfx)
{
  size_t ix, iy;
  ix = (cell - grid->cell) % grid->dimx;
  iy = (cell - grid->cell) / grid->dimx;
  printf ("%s[%3zu  %3zu]  k: %4g  r: %4g  p: %4g\n",
	  pfx, ix, iy, cell->key, cell->rhs, cell->phi);
}

void swap (estar_cell_t ** aa, estar_cell_t ** bb)
{
  size_t ti;
  estar_cell_t *tc;
  ti = (*aa)->pqi;
  (*aa)->pqi = (*bb)->pqi;
  (*bb)->pqi = ti;
  tc = (*aa);
  (*aa) = (*bb);
  (*bb) = tc;
}


void bubble_up (estar_cell_t ** heap, size_t index)
{
  size_t parent;
  parent = index / 2;
  while ((parent > 0) && (heap[index]->key < heap[parent]->key)) {
    swap (&heap[index], &heap[parent]);
    index = parent;
    parent = index / 2;
  }
}


void bubble_down (estar_cell_t ** heap, size_t len, size_t index)
{
  size_t child, target;
  
  target = index;
  while (1) {
    child = 2 * index;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    ++child;
    if (child <= len && heap[child]->key < heap[target]->key) {
      target = child;
    }
    if (index == target) {
      break;
    }
    swap (&heap[target], &heap[index]);
    index = target;
  }
}

int estar_check (estar_t * estar, char const * pfx)
{
  int status;
  size_t ii, jj, kk;
  
  status = 0;
  
  for (ii = 0; ii < estar->grid.dimx; ++ii) {
    for (jj = 0; jj < estar->grid.dimy; ++jj) {
      estar_cell_t * cell;
      cell = estar_grid_at (&estar->grid, ii, jj);
      
      if (cell->rhs == cell->phi) {
	// consistent
	if (0 != cell->pqi) {
	  printf ("%sconsistent cell should not be on queue\n", pfx);
	  status |= 1;
	}
      }
      else {
	// inconsistent
	if (0 == cell->pqi) {
	  printf ("%sinconsistent cell should be on queue\n", pfx);
	  status |= 2;
	}
      }
      
      if (0 == cell->pqi) {
	// not on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    printf ("%scell with pqi == 0 should not be on queue\n", pfx);
	    status |= 4;
	    break;
	  }
	}
      }
      else {
	// on queue
	for (kk = 1; kk <= estar->pq.len; ++kk) {
	  if (cell == estar->pq.heap[kk]) {
	    break;
	  }
	}
	if (kk > estar->pq.len) {
	  printf ("%scell with pqi != 0 should be on queue\n", pfx);
	  status |= 8;
	}
      }
    }
  }
  
  for (ii = 1; ii <= estar->pq.len; ++ii) {
    if (estar->pq.heap[ii]->pqi != ii) {
      printf ("%sinconsistent pqi\n", pfx);
      estar_dump_queue (estar, pfx);
      status |= 16;
      break;
    }
  }
  
  return status;
}

int estar_cell_calc_gradient (estar_cell_t * cell, double * gx, double * gy)
{
  estar_cell_t ** nn;
  estar_cell_t * n1;
  estar_cell_t * n2;
  int direction;
  
  n1 = NULL;
  for (nn = cell->nbor; *nn != NULL; ++nn) {
    if (isfinite ((*nn)->rhs)
	&& (*nn)->rhs < cell->rhs
	&& (n1 == NULL || (*nn)->rhs < n1->rhs)) {
      n1 = *nn;
    }
  }
  if (NULL == n1) {
    return 0;
  }
  
  direction = n1 - cell;
  // +1 means right
  // -1 means left
  // +dimx means up (grid is arranged like pixels on a screen)
  // -dimx means down
  
  n2 = NULL;
  for (nn = cell->nbor; *nn != NULL; ++nn) {
    if (isfinite ((*nn)->rhs)
	&& (*nn) != n1
	&& direction != cell - *nn /* check it is not opposite n1 */
	&& (n2 == NULL || (*nn)->rhs < n2->rhs)) {
      n2 = *nn;
    }
  }

  if (NULL == n2) {
    if (direction == -1) {
      *gx = n1->rhs - cell->rhs; /* some negative value */
      *gy = 0.0;
    }
    else if (direction == 1) {
      *gx = cell->rhs - n1->rhs; /* some positive value */
      *gy = 0.0;
    }
    else if (direction < 0) {
      *gx = 0.0;
      *gy = n1->rhs - cell->rhs; /* some negative value */
    }
    else {
      *gx = 0.0;
      *gy = cell->rhs - n1->rhs; /* some positive value */
    }
    return 1;
  }
  
  if (direction == -1) {
    *gx = n1->rhs - cell->rhs;
    if (cell - n2 > 0) {
      *gy = n2->rhs - cell->rhs;
    }
    else {
      *gy = cell->rhs - n2->rhs;
    }
  }
  else if (direction == 1) {
    *gx = cell->rhs - n1->rhs;
    if (cell - n2 > 0) {
      *gy = n2->rhs - cell->rhs;
    }
    else {
      *gy = cell->rhs - n2->rhs;
    }
  }
  else if (direction < 0) {
    if (cell - n2 > 0) {
      *gx = n2->rhs - cell->rhs;
    }
    else {
      *gx = cell->rhs - n2->rhs;
    }
    *gy = n1->rhs - cell->rhs;
  }
  else {
    if (cell - n2 > 0) {
      *gx = n2->rhs - cell->rhs;
    }
    else {
      *gx = cell->rhs - n2->rhs;
    }
    *gy = cell->rhs - n1->rhs;
  }
  
  return 2;
}


double estar_pqueue_topkey (estar_pqueue_t * pq)
{
  if (pq->len > 0) {
    return pq->heap[1]->key;
  }
  return INFINITY;
}

double interpolate (double cost, double primary, double secondary)
{
  double tmp;
  
  if (cost <= secondary - primary) {
    return primary + cost;
  }
  
  // pow(cost,2) could be cached inside estar_set_speed. And so could
  // the other squared terms. That might speed things up, but it would
  // certainly make hearier caching code.
  
  tmp = primary + secondary;
  return (tmp + sqrt(pow(tmp, 2.0)
		     - 2.0 * (pow(primary, 2.0)
			      + pow(secondary, 2.0)
			      - pow(cost, 2.0)))) / 2.0;
}


estar_cell_t * estar_pqueue_extract (estar_pqueue_t * pq)
{
  estar_cell_t * cell;
  
  if (0 == pq->len) {
    return NULL;
  }
  
  cell = pq->heap[1];
  cell->pqi = 0;		/* mark cell as not on queue */
  
  if (1 == pq->len) {
    pq->len = 0;
    return cell;
  }
  
  pq->heap[1] = pq->heap[pq->len];
  pq->heap[1]->pqi = 1;		/* keep pqi consistent */
  --pq->len;
  // here would be a good place to shrink the heap
  
  bubble_down (pq->heap, pq->len, 1);
  
  return cell;
}


int main()
{
	printf("Hello world\n");
	return 1;
}
