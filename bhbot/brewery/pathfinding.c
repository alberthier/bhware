#include <Python.h>
#include <structmember.h>


typedef struct _Cell
{
    int x;
    int y;
    float h_score;
    float g_score;
    float f_score;
    int is_forbidden;
    float cost;
    struct _Cell* came_from;
    struct _Cell* next;
} Cell;


typedef int (*LessFunction)(Cell*, Cell*);


static int cell_f_score_less(Cell* a, Cell* b)
{
    return a->f_score < b->f_score;
}


static int cell_pointer_less(Cell* a, Cell* b)
{
    return a < b;
}


static int cell_contains(Cell* self, Cell* other)
{
    Cell* current = NULL;

    for (current = self; current != NULL; current = current->next) {
        if (current == other) {
            return 1;
        }
    }

    return 0;
}


static int cell_contains_sorted(Cell* self, Cell* other, LessFunction less)
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


static Cell* cell_insert_sorted(Cell* self, Cell* other, LessFunction less)
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


typedef enum
{
    ZoneForbidden,
    ZonePenalized
} ZoneType;


typedef enum
{
  ZoneCircle,
  ZoneRect
} ZoneShape;


typedef struct _Zone
{
    union {
        struct {
            /* Rect definition */
            int x1;
            int y1;
            int x2;
            int y2;
        };
        struct {
            /* Circle definition */
            int x;
            int y;
            int radius;
        };
    };
    int id;
    float cost;
    ZoneType type;
    ZoneShape shape;
    float** distance_map;
    struct _Zone* next;
} Zone;


static int zone_contains(Zone* self, int x, int y)
{
    if (self->shape == ZoneCircle) {
        int dx = abs(self->x - x);
        int dy = abs(self->y - y);
        return self->distance_map[dx][dy] <= self->radius;
    }
    return x >= self->x1 && x <= self->x2 && y >= self->y1 && y <= self->y2;
}


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
    Zone* zones;
    int next_zone_id;
} PathFinder;


static void pathfinder_dealloc(PathFinder* self)
{
    Zone* zone = NULL;
    int x = 0;

    for (x = 0; x < self->map_x_size; ++x) {
        free(self->distance_map[x]);
        free(self->map[x]);
    }
    free(self->distance_map);
    free(self->map);

    zone = self->zones;
    while (zone != NULL) {
        Zone* tmp = zone->next;
        free(zone);
        zone = tmp;
    }

    Py_TYPE(self)->tp_free((PyObject*) self);
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
            row[y].is_forbidden = 0;
            row[y].cost = 0.0;
        }
    }

    self->zones = NULL;
    self->next_zone_id = 0;

    return 0;
}


static Cell** pathfinder_append_if_valid(PathFinder* self, Cell* node, Cell** it, int x, int y)
{
    Cell* neighbor = &self->map[x][y];
    if (x < 0 || x >= self->map_x_size) {
        return it;
    }
    if (y < 0 || y >= self->map_y_size) {
        return it;
    }
    if (!node->is_forbidden && neighbor->is_forbidden) {
        return it;
    }
    *it = neighbor;
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
            it = pathfinder_append_if_valid(self, node, it, prev_x, prev_y);
        }
        it = pathfinder_append_if_valid(self, node, it, prev_x, node->y);
        if (next_y < self->map_y_size) {
            it = pathfinder_append_if_valid(self, node, it, prev_x, next_y);
        }
    }
    if (prev_y >= 0) {
        it = pathfinder_append_if_valid(self, node, it, node->x, prev_y);
    }
    if (next_y < self->map_y_size) {
        it = pathfinder_append_if_valid(self, node, it, node->x, next_y);
    }
    if (next_x < self->map_x_size) {
        if (prev_y >= 0) {
            it = pathfinder_append_if_valid(self, node, it, next_x, prev_y);
        }
        it = pathfinder_append_if_valid(self, node, it, next_x, node->y);
        if (next_y < self->map_y_size) {
            it = pathfinder_append_if_valid(self, node, it, next_x, next_y);
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
    float cost = self->map[x][y].cost;

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
    Zone* zone = NULL;

    if (!PyArg_ParseTuple(args, "iiii", &start_x, &start_y, &goal_x, &goal_y)) {
        return NULL;
    }

    if (start_x < 0 || start_x >= self->map_x_size || goal_x < 0 || goal_x >= self->map_x_size ||
        start_y < 0 || start_y >= self->map_y_size || goal_y < 0 || goal_y >= self->map_y_size) {
        /* Coordinates out of range */
        Py_RETURN_NONE;
    }
    if (start_x == goal_x && start_y == goal_y) {
        /* Already arrived */
        Py_RETURN_NONE;
    }

    for (zone = self->zones; zone != NULL; zone = zone->next) {
        /* If the destination point is in a wall, it is unreachable */
        if (zone->type == ZoneForbidden) {
            if (zone_contains(zone, goal_x, goal_y)) {
                /* Unreachable point */
                Py_RETURN_NONE;
            }
        }
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
            float cost = current->f_score;
            PyObject* path = PyList_New(0);
            for (; current != NULL; current = current->came_from) {
                PyList_Insert(path, 0, Py_BuildValue("(ii)", current->x, current->y));
            }
            return Py_BuildValue("(fN)", cost, path);
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

    Py_RETURN_NONE;
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


static void pathfinder_rebuild_map(PathFinder* self)
{
    int x = 0;
    int y = 0;

    for (x = 0; x < self->map_x_size; ++x) {
        for (y = 0; y < self->map_y_size; ++y) {
            Zone* zone = NULL;
            Cell* current = &self->map[x][y];
            current->is_forbidden = 0;
            current->cost = 0.0;
            for (zone = self->zones; zone != NULL; zone = zone->next) {
                if (zone_contains(zone, x, y)) {
                    current->is_forbidden |= zone->type == ZoneForbidden;
                    if (current->cost < zone->cost) {
                        current->cost = zone->cost;
                    }
                }
            }
        }
    }
}


static PyObject* pathfinder_add_zone(PathFinder* self, PyObject* args, ZoneType zone_type, ZoneShape zone_shape)
{
    Zone* zone = (Zone*) malloc(sizeof(Zone));

    if (zone_shape == ZoneRect) {
        if (!PyArg_ParseTuple(args, "iiiif", &zone->x1, &zone->y1, &zone->x2, &zone->y2, &zone->cost)) {
            return NULL;
        }
        pathfinder_normalize_rectangle(&zone->x1, &zone->y1, &zone->x2, &zone->y2);
    } else {
        if (!PyArg_ParseTuple(args, "iiif", &zone->x, &zone->y, &zone->radius, &zone->cost)) {
            return NULL;
        }
    }

    zone->type = zone_type;
    zone->shape = zone_shape;
    zone->next = self->zones;
    zone->id = self->next_zone_id;
    zone->distance_map = self->distance_map;
    ++self->next_zone_id;
    self->zones = zone;

    pathfinder_rebuild_map(self);

    return Py_BuildValue("i", zone->id);
}


static PyObject* pathfinder_add_forbidden_rect_zone(PathFinder* self, PyObject* args)
{
    return pathfinder_add_zone(self, args, ZoneForbidden, ZoneRect);
}


static PyObject* pathfinder_add_forbidden_circle_zone(PathFinder* self, PyObject* args)
{
    return pathfinder_add_zone(self, args, ZoneForbidden, ZoneCircle);
}


static PyObject* pathfinder_add_penalized_rect_zone(PathFinder* self, PyObject* args)
{
    return pathfinder_add_zone(self, args, ZonePenalized, ZoneRect);
}


static PyObject* pathfinder_add_penalized_circle_zone(PathFinder* self, PyObject* args)
{
    return pathfinder_add_zone(self, args, ZonePenalized, ZoneCircle);
}


static PyObject* pathfinder_remove_zone(PathFinder* self, PyObject* args)
{
    int id = 0;
    Zone* zone = NULL;
    Zone* prev_zone = NULL;

    if (!PyArg_ParseTuple(args, "i", &id)) {
        return NULL;
    }

    for (zone = self->zones; zone != NULL; zone = zone->next) {
        if (zone->id == id) {
            if (zone == self->zones) {
                self->zones = zone->next;
                free(zone);
            } else {
                prev_zone->next = zone->next;
                free(zone);
            }
            break;
        }
        prev_zone = zone;
    }

    pathfinder_rebuild_map(self);

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
    { "add_forbidden_rect"               , (PyCFunction) pathfinder_add_forbidden_rect_zone          , METH_VARARGS, "Add a forbidden rect (x1, y1) to (x2, y2)" },
    { "add_forbidden_circle"             , (PyCFunction) pathfinder_add_forbidden_circle_zone        , METH_VARARGS, "Add a forbidden circle centered at (x, y), radius r" },
    { "add_penalized_rect"               , (PyCFunction) pathfinder_add_penalized_rect_zone          , METH_VARARGS, "Add a penalized rect (x1, y1) to (x2, y2) with the given cost" },
    { "add_penalized_circle"             , (PyCFunction) pathfinder_add_penalized_circle_zone        , METH_VARARGS, "Add a penalized circle centered at (x, y), radius r" },
    { "remove_zone"                      , (PyCFunction) pathfinder_remove_zone                      , METH_VARARGS, "Remove a penalized zone" },
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
    PyVarObject_HEAD_INIT(NULL, 0)
    "pathfinding.PathFinder",                 /* tp_name           */
    sizeof(PathFinder),                       /* tp_basicsize      */
    0,                                        /* tp_itemsize       */
    (destructor) pathfinder_dealloc,          /* tp_dealloc        */
    0,                                        /* tp_print          */
    0,                                        /* tp_getattr        */
    0,                                        /* tp_setattr        */
    0,                                        /* tp_compare        */
    0,                                        /* tp_repr           */
    0,                                        /* tp_as_number      */
    0,                                        /* tp_as_sequence    */
    0,                                        /* tp_as_mapping     */
    0,                                        /* tp_hash           */
    0,                                        /* tp_call           */
    0,                                        /* tp_str            */
    0,                                        /* tp_getattro       */
    0,                                        /* tp_setattro       */
    0,                                        /* tp_as_buffer      */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags          */
    "Path finder object",                     /* tp_doc            */
    0,                                        /* tp_traverse       */
    0,                                        /* tp_clear          */
    0,                                        /* tp_richcompare    */
    0,                                        /* tp_weaklistoffset */
    0,                                        /* tp_iter           */
    0,                                        /* tp_iternext       */
    PathFinder_methods,                       /* tp_methods        */
    PathFinder_members,                       /* tp_members        */
    0,                                        /* tp_getset         */
    0,                                        /* tp_base           */
    0,                                        /* tp_dict           */
    0,                                        /* tp_descr_get      */
    0,                                        /* tp_descr_set      */
    0,                                        /* tp_dictoffset     */
    (initproc) pathfinder_init,               /* tp_init           */
    0,                                        /* tp_alloc          */
    PyType_GenericNew,                        /* tp_new            */
};


static PyModuleDef pathfindingmodule = {
    PyModuleDef_HEAD_INIT,
    "pathfinding",                   /* m_name     */
    "Grid based pathfinding module", /* m_doc      */
    -1,                              /* m_size     */
    NULL,                            /* m_methods  */
    NULL,                            /* m_reload   */
    NULL,                            /* m_traverse */
    NULL,                            /* m_clear    */
    NULL                             /* m_free     */
};


PyMODINIT_FUNC PyInit_pathfinding(void)
{
    PyObject* m;

    if (PyType_Ready(&PathFinderType) < 0)
        return;

    m = PyModule_Create(&pathfindingmodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&PathFinderType);
    PyModule_AddObject(m, "PathFinder", (PyObject*) &PathFinderType);

    return m;
}
