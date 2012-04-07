#include <Python.h>
#include <structmember.h>


typedef struct _Cell
{
    int x;
    int y;
    float h_score;
    float g_score;
    float f_score;
    struct _Cell* came_from;
    struct _Cell* next;
} Cell;


typedef int (*LessFunction)(Cell*, Cell*);


int cell_f_score_less(Cell* a, Cell* b)
{
    return a->f_score < b->f_score;
}


int cell_pointer_less(Cell* a, Cell* b)
{
    return a < b;
}


int cell_contains(Cell* self, Cell* other)
{
    Cell* current = NULL;

    for (current = self; current != NULL; current = current->next) {
        if (current == other) {
            return 1;
        }
    }

    return 0;
}


int cell_contains_sorted(Cell* self, Cell* other, LessFunction less)
{
    Cell* current = NULL;

    for (current = self; current != NULL; current = current->next) {
        if (current == other) {
            return 1;
        }
        if (!less(current, other)) {
            return 0;
        }
    }

    return 0;
}


Cell* cell_insert_sorted(Cell* self, Cell* other, LessFunction less)
{
    Cell* previous = NULL;
    Cell* current = NULL;

    /* The list is empty */
    if (self == NULL) {
        other->next = NULL;
        return other;
    }

    /* Middle of the list */
    for (current = self; current != NULL; current = current->next) {
        if (!less(current, other)) {
            if (previous != NULL) {
                previous->next = other;
                other->next = current;
                return self;
            } else {
                other->next = self;
                return other;
            }
        }
        previous = current;
    }

    /* other is greater than all list items */
    previous->next = other;
    other->next = NULL;
    return self;
}


typedef struct _Wall
{
    int x1;
    int y1;
    int x2;
    int y2;
    struct _Wall* next;
} Wall;


typedef struct _PenalizedZone
{
    int x1;
    int y1;
    int x2;
    int y2;
    float cost;
    struct _PenalizedZone* next;
} PenalizedZone;


/********************/
/* Pathfinder class */
/********************/


typedef struct
{
    PyObject_HEAD
    int map_y_size;
    int map_x_size;
    float effective_vs_heuristic_tradeoff;
    int main_opponent_avoidance_cells;
    int secondary_opponent_avoidance_cells;
    int main_opponent_x;
    int main_opponent_y;
    float main_opponent_collision_cost;
    int secondary_opponent_x;
    int secondary_opponent_y;
    float secondary_opponent_collision_cost;
    float** distance_map;
    Cell** map;
    Wall* walls;
    PenalizedZone* penalized_zones;
} PathFinder;


static void pathfinder_dealloc(PathFinder* self)
{
    int x = 0;

    for (x = 0; x < self->map_x_size; ++x) {
        free(self->distance_map[x]);
        free(self->map[x]);
    }
    free(self->distance_map);
    free(self->map);

    Wall* wall = self->walls;
    while (wall != NULL) {
        Wall* tmp = wall->next;
        free(wall);
        wall = tmp;
    }

    PenalizedZone* penalized_zone = self->penalized_zones;
    while (penalized_zone != NULL) {
        PenalizedZone* tmp = penalized_zone->next;
        free(penalized_zone);
        penalized_zone = tmp;
    }

    self->ob_type->tp_free((PyObject*) self);
}


static int pathfinder_init(PathFinder* self, PyObject* args, PyObject* kwds)
{
    int x = 0;
    int y = 0;

    if (!PyArg_ParseTuple(args, "iifiiff",
                          &self->map_x_size,
                          &self->map_y_size,
                          &self->effective_vs_heuristic_tradeoff,
                          &self->main_opponent_avoidance_cells,
                          &self->secondary_opponent_avoidance_cells,
                          &self->main_opponent_collision_cost,
                          &self->secondary_opponent_collision_cost)) {
        return -1;
    }

    self->main_opponent_x = -1;
    self->main_opponent_y = -1;
    self->secondary_opponent_x = -1;
    self->secondary_opponent_y = -1;

    /* Setup the map and the distance cache */
    self->distance_map = (float**) malloc(sizeof(float*) * self->map_x_size);
    self->map = (Cell**) malloc(sizeof(Cell*) * self->map_x_size);
    for (x = 0; x < self->map_x_size; ++x) {
        float* distance_row = (float*) malloc(sizeof(float) * self->map_y_size);
        Cell* row = (Cell*) malloc(sizeof(Cell) * self->map_y_size);
        float x_squared = x * x;
        self->distance_map[x] = distance_row;
        self->map[x] = row;
        for (y = 0; y < self->map_y_size; ++y) {
            distance_row[y] = sqrt(x_squared + y * y);
            row[y].x = x;
            row[y].y = y;
        }
    }

    self->walls = NULL;
    self->penalized_zones = NULL;

    return 0;
}


Cell** pathfinder_append_if_valid(PathFinder* self, Cell** it, int x, int y)
{
    Wall* wall = NULL;
    if (x < 0 || x >= self->map_x_size) {
        return it;
    }
    if (y < 0 || y >= self->map_y_size) {
        return it;
    }
    for (wall = self->walls; wall != NULL; wall = wall->next) {
        if (x >= wall->x1 && x <= wall->x2 && y >= wall->y1 && y <= wall->y2) {
            return it;
        }
    }
    *it = &self->map[x][y];
    ++it;
    return it;
}


static void pathfinder_find_neighbor_nodes(PathFinder* self, Cell* node, Cell** neighbor_nodes)
{
    Cell** it = neighbor_nodes;
    int prev_x = node->x - 1;
    int next_x = node->x + 1;
    int prev_y = node->y - 1;
    int next_y = node->y + 1;

    if (prev_x >= 0) {
        if (prev_y >= 0) {
            it = pathfinder_append_if_valid(self, it, prev_x, prev_y);
        }
        it = pathfinder_append_if_valid(self, it, prev_x, node->y);
        if (next_y < self->map_y_size) {
            it = pathfinder_append_if_valid(self, it, prev_x, next_y);
        }
    }
    if (prev_y >= 0) {
        it = pathfinder_append_if_valid(self, it, node->x, prev_y);
    }
    if (next_y < self->map_y_size) {
        it = pathfinder_append_if_valid(self, it, node->x, next_y);
    }
    if (next_x < self->map_x_size) {
        if (prev_y >= 0) {
            it = pathfinder_append_if_valid(self, it, next_x, prev_y);
        }
        it = pathfinder_append_if_valid(self, it, next_x, node->y);
        if (next_y < self->map_y_size) {
            it = pathfinder_append_if_valid(self, it, next_x, next_y);
        }
    }

    *it = NULL;
}


static float pathfinder_heuristic_cost_estimate(PathFinder* self, int x1, int y1, int x2, int y2)
{
    return self->distance_map[abs(x2 - x1)][abs(y2 - y1)] * self->effective_vs_heuristic_tradeoff;
}


static float pathfinder_effective_cost(PathFinder* self, int x1, int y1, int x2, int y2)
{
    return self->distance_map[abs(x2 - x1)][abs(y2 - y1)];
}


static float pathfinder_penalized_cost(PathFinder* self, int x, int y)
{
    float cost = 0.0;
    PenalizedZone* penalized_zone = NULL;

    for (penalized_zone = self->penalized_zones; penalized_zone != NULL; penalized_zone = penalized_zone->next) {
        if (x >= penalized_zone->x1 && x <= penalized_zone->x2 && y >= penalized_zone->y1 && y <= penalized_zone->y2) {
            if (penalized_zone->cost > cost) {
                cost = penalized_zone->cost;
            }
        }
    }
    if (self->main_opponent_x != -1 && self->distance_map[abs(x - self->main_opponent_x)][abs(y - self->main_opponent_y)] < self->main_opponent_avoidance_cells) {
        if (self->main_opponent_collision_cost > cost) {
            cost = self->main_opponent_collision_cost;
        }
    }
    if (self->secondary_opponent_x != -1 && self->distance_map[abs(x - self->secondary_opponent_x)][abs(y - self->secondary_opponent_y)] < self->secondary_opponent_avoidance_cells) {
        if (self->secondary_opponent_collision_cost > cost) {
            cost = self->secondary_opponent_collision_cost;
        }
    }

    return cost;
}


static PyObject* pathfinder_find(PathFinder* self, PyObject* args)
{
    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    Cell* closed_set = NULL;
    Cell* open_set = NULL;

    if (!PyArg_ParseTuple(args, "iiii", &start_x, &start_y, &goal_x, &goal_y)) {
        return NULL;
    }

    if (start_x < 0 || start_x >= self->map_x_size || goal_x < 0 || goal_x >= self->map_x_size ||
        start_y < 0 || start_y >= self->map_y_size || goal_y < 0 || goal_y >= self->map_y_size) {
        /* Coordinates out of range */
        return PyList_New(0);
    }

    open_set = &self->map[start_x][start_y];
    open_set->h_score = pathfinder_heuristic_cost_estimate(self, start_x, start_y, goal_x, goal_y);
    open_set->g_score = 0.0;
    open_set->f_score = open_set->h_score;
    open_set->next = NULL;
    open_set->came_from = NULL;

    while (open_set != NULL) {
        Cell* current = open_set;
        Cell* neighbor_nodes[9];
        Cell** nit = NULL;

        if (current->x == goal_x && current->y == goal_y) {
            PyObject* path = PyList_New(0);
            for (; current != NULL; current = current->came_from) {
                PyList_Insert(path, 0, Py_BuildValue("(ii)", current->x, current->y));
            }
            return path;
        }

        open_set = open_set->next;
        closed_set = cell_insert_sorted(closed_set, current, cell_pointer_less);

        pathfinder_find_neighbor_nodes(self, current, neighbor_nodes);
        for (nit = neighbor_nodes; *nit != NULL; ++nit) {
            Cell* neighbor = *nit;
            if (!cell_contains_sorted(closed_set, neighbor, cell_pointer_less)) {
                float tentative_g_score = current->g_score + pathfinder_effective_cost(self, current->x, current->y, neighbor->x, neighbor->y);
                tentative_g_score += pathfinder_penalized_cost(self, neighbor->x, neighbor->y);
                if (!cell_contains(open_set, neighbor)) {
                    neighbor->h_score = pathfinder_heuristic_cost_estimate(self, neighbor->x, neighbor->y, goal_x, goal_y);
                    neighbor->g_score = tentative_g_score;
                    neighbor->f_score = neighbor->g_score + neighbor->h_score;
                    neighbor->came_from = current;
                    open_set = cell_insert_sorted(open_set, neighbor, cell_f_score_less);
                } else if (tentative_g_score < neighbor->g_score) {
                    neighbor->g_score = tentative_g_score;
                    neighbor->f_score = neighbor->g_score + neighbor->h_score;
                    neighbor->came_from = current;
                }
            }
        }
    }

    return PyList_New(0);
}


static void pathfinder_normalize_rectangle(int* x1, int* y1, int* x2, int* y2)
{
    if (*x1 > *x2) {
        int tmp = *x1;
        *x1 = *x2;
        *x2 = tmp;
    }
    if (*y1 > *y2) {
        int tmp = *y1;
        *y1 = *y2;
        *y2 = tmp;
    }
}


static PyObject* pathfinder_add_wall(PathFinder* self, PyObject* args)
{
    Wall* wall = (Wall*) malloc(sizeof(Wall));

    if (!PyArg_ParseTuple(args, "iiii", &wall->x1, &wall->y1, &wall->x2, &wall->y2)) {
        return NULL;
    }

    pathfinder_normalize_rectangle(&wall->x1, &wall->y1, &wall->x2, &wall->y2);

    wall->next = self->walls;
    self->walls = wall;

    Py_RETURN_NONE;
}


static PyObject* pathfinder_add_penalized_zone(PathFinder* self, PyObject* args)
{
    PenalizedZone* penalized_zone = (PenalizedZone*) malloc(sizeof(PenalizedZone));

    if (!PyArg_ParseTuple(args, "iiiif", &penalized_zone->x1, &penalized_zone->y1, &penalized_zone->x2, &penalized_zone->y2, &penalized_zone->cost)) {
        return NULL;
    }

    pathfinder_normalize_rectangle(&penalized_zone->x1, &penalized_zone->y1, &penalized_zone->x2, &penalized_zone->y2);

    penalized_zone->next = self->penalized_zones;
    self->penalized_zones = penalized_zone;

    Py_RETURN_NONE;
}


static PyObject* pathfinder_set_main_opponent_position(PathFinder* self, PyObject* args)
{
    if (!PyArg_ParseTuple(args, "ii", &self->main_opponent_x, &self->main_opponent_y)) {
        return NULL;
    }

    Py_RETURN_NONE;
}


static PyObject* pathfinder_clear_main_opponent_position(PathFinder* self)
{
    self->main_opponent_x = -1;
    self->main_opponent_y = -1;

    Py_RETURN_NONE;
}


static PyObject* pathfinder_set_secondary_opponent_position(PathFinder* self, PyObject* args)
{
    if (!PyArg_ParseTuple(args, "ii", &self->secondary_opponent_x, &self->secondary_opponent_y)) {
        return NULL;
    }

    Py_RETURN_NONE;
}


static PyObject* pathfinder_clear_secondary_opponent_position(PathFinder* self)
{
    self->secondary_opponent_x = -1;
    self->secondary_opponent_y = -1;

    Py_RETURN_NONE;
}


/****************************/
/* Python object definition */
/****************************/


/* Object methods */
static PyMethodDef PathFinder_methods[] = {
    { "find"                             , (PyCFunction) pathfinder_find                             , METH_VARARGS, "Find the route from (x1, y1) to (x2, y2)" },
    { "add_wall"                         , (PyCFunction) pathfinder_add_wall                         , METH_VARARGS, "Add a wall (x1, y1) to (x2, y2)" },
    { "add_penalized_zone"               , (PyCFunction) pathfinder_add_penalized_zone               , METH_VARARGS, "Add a penalized zone (x1, y1) to (x2, y2) with the given cost" },
    { "set_main_opponent_position"       , (PyCFunction) pathfinder_set_main_opponent_position       , METH_VARARGS, "Sets the main opponent position" },
    { "clear_main_opponent_position"     , (PyCFunction) pathfinder_clear_main_opponent_position     , METH_NOARGS , "Clears the main opponent position" },
    { "set_secondary_opponent_position"  , (PyCFunction) pathfinder_set_secondary_opponent_position  , METH_VARARGS, "Sets the secondary opponent position" },
    { "clear_secondary_opponent_position", (PyCFunction) pathfinder_clear_secondary_opponent_position, METH_NOARGS , "Clears the main opponent position" },
    {NULL}  /* Sentinel */
};


/* Object members */
static PyMemberDef PathFinder_members[] = {
    {NULL}  /* Sentinel */
};


/* Object definition */
static PyTypeObject PathFinderType = {
    PyObject_HEAD_INIT(NULL)
    0,                                        /* ob_size */
    "pathfinding.PathFinder",                 /* tp_name */
    sizeof(PathFinder),                       /* tp_basicsize */
    0,                                        /* tp_itemsize */
    (destructor) pathfinder_dealloc,          /* tp_dealloc */
    0,                                        /* tp_print */
    0,                                        /* tp_getattr */
    0,                                        /* tp_setattr */
    0,                                        /* tp_compare */
    0,                                        /* tp_repr */
    0,                                        /* tp_as_number */
    0,                                        /* tp_as_sequence */
    0,                                        /* tp_as_mapping */
    0,                                        /* tp_hash */
    0,                                        /* tp_call */
    0,                                        /* tp_str */
    0,                                        /* tp_getattro */
    0,                                        /* tp_setattro */
    0,                                        /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags */
    "Path finder object",                     /* tp_doc */
    0,                                        /* tp_traverse */
    0,                                        /* tp_clear */
    0,                                        /* tp_richcompare */
    0,                                        /* tp_weaklistoffset */
    0,                                        /* tp_iter */
    0,                                        /* tp_iternext */
    PathFinder_methods,                       /* tp_methods */
    PathFinder_members,                       /* tp_members */
    0,                                        /* tp_getset */
    0,                                        /* tp_base */
    0,                                        /* tp_dict */
    0,                                        /* tp_descr_get */
    0,                                        /* tp_descr_set */
    0,                                        /* tp_dictoffset */
    (initproc) pathfinder_init,               /* tp_init */
    0,                                        /* tp_alloc */
    PyType_GenericNew,                        /* tp_new */
};


static PyMethodDef module_methods[] = {
    {NULL}  /* Sentinel */
};


PyMODINIT_FUNC initpathfinding(void)
{
    PyObject* m;

    if (PyType_Ready(&PathFinderType) < 0)
        return;

    m = Py_InitModule3("pathfinding", module_methods, "Path finding algorithm.");

    if (m == NULL)
      return;

    Py_INCREF(&PathFinderType);
    PyModule_AddObject(m, "PathFinder", (PyObject*) &PathFinderType);
}
