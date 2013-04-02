#include <Python.h>
#include <structmember.h>

/*****************************************************************************/
/* General purpose features                                                  */
/*****************************************************************************/


/* Arrays */

#define ARRAY_CHUNK_CAPACITY 16
#define array_capacity(count) (((count / ARRAY_CHUNK_CAPACITY) + (count % ARRAY_CHUNK_CAPACITY > 0)) * ARRAY_CHUNK_CAPACITY)
#define array_new(type, count) (type*) malloc(array_capacity(count) * sizeof(type))

static void* array_ensure_capacity_(void* self, size_t item_size, size_t count, size_t required)
{
    size_t current_capacity = array_capacity(count);
    size_t required_capacity = array_capacity(required);
    if (current_capacity != required_capacity) {
        return realloc(self, item_size * required_capacity);
    }
    return self;
}

#define array_ensure_capacity(self, type, count, required) (type*) array_ensure_capacity_(self, sizeof(type), count, required)


/* Tools */

#define TOOLS_EPSILON 1e-6

static int tools_quasi_equal(float a, float b)
{
    return fabs(a - b) < TOOLS_EPSILON;
}

static int tools_is_between(float a, float b, float x)
{
    if (tools_quasi_equal(a, x) || tools_quasi_equal(b, x)) {
        return 0;
    }
    if (a < b) {
        return a < x && x < b;
    } else {
        return b < x && x < a;
    }
}


/*****************************************************************************/
/* Path finding code                                                         */
/*****************************************************************************/


/* Node */

typedef struct _Edge Edge;

typedef struct _Node
{
    float x;
    float y;
    float g_score;
    float h_score;
    float f_score;
    struct _Edge** edges;
    int edges_count;
} Node;


static Node* node_new(float x, float y)
{
    Node* self = (Node*) malloc(sizeof(Node));
    self->x = x;
    self->y = y;
    self->g_score = 0.0;
    self->h_score = 0.0;
    self->f_score = 0.0;
    self->edges = NULL;
    self->edges_count = 0;
    return self;
}


static void node_free(Node* self)
{
    if (self->edges != NULL) {
        free(self->edges);
    }
    free(self);
}


static void node_create_edges_array(Node* self, int size)
{
    self->edges = (Edge**) malloc(size * sizeof(Edge*));
}


static void node_add_edge(Node* self, Edge* edge)
{
    self->edges[self->edges_count++] = edge;
}


/* Edge */

typedef struct _Edge
{
    Node* node1;
    Node* node2;
    float a;
    float b;
    int enabled;
    int allowed;
} Edge;


static Edge* edge_new(Node* node1, Node* node2)
{
    Edge* self = (Edge*) malloc(sizeof(Edge));

    self->node1 = node1;
    self->node2 = node2;
    self->a = INFINITY;
    self->b = INFINITY;
    self->enabled = 1;
    self->allowed = 1;

    return self;
}


static void edge_free(Edge* self)
{
    free(self);
}


static int edge_links(Edge* self, Node* node1, Node* node2)
{
    return (self->node1 == node1 && self->node2 == node2) || (self->node1 == node2 && self->node2 == node1);
}


static void edge_update(Edge* self)
{
    self->allowed = self->enabled;
    if (tools_quasi_equal(self->node1->x, self->node2->x)) {
        /* Vertical edge */
        self->a = INFINITY;
        self->b = INFINITY;
    } else {
        /* General case */
        self->a = (self->node2->y - self->node1->y) / (self->node2->x - self->node1->x);
        self->b = self->node1->y - self->a * self->node1->x;
    }
}


static int edge_contains(Edge* self, float x, float y)
{
    int ok = 0;
    if (!isfinite(self->a)) {
        /* Vertical edge */
        ok = tools_quasi_equal(x, self->node1->x) &&
             tools_is_between(self->node1->y, self->node2->y, y);
    } else if (tools_quasi_equal(self->a, 0.0)) {
        /* Horizontal edge */
        ok = tools_quasi_equal(self->b, y) &&
            tools_is_between(self->node1->x, self->node2->x, x);
    } else {
        /* General case */
        /* Check that the point is on the line */
        ok = tools_quasi_equal(self->a * x + self->b, y);
        /* Check that the point is in the segment bounds */
        ok &= tools_is_between(self->node1->x, self->node2->x, x);
        ok &= tools_is_between(self->node1->y, self->node2->y, y);
    }
    return ok;
}


static int edge_intersects(Edge* self, Edge* other)
{
    float cross_x = 0.0;
    float cross_y = 0.0;

    if ( self->node1 == other->node1 || self->node1 == other->node2 || self->node2 == other->node1 || self->node2 == other->node2) {
        return 0;
    }
    if (!isfinite(self->a) && !isfinite(other->a)) {
        /* Two vertical lines */
        return tools_quasi_equal(self->node1->x, other->node1->x) &&
                (tools_is_between(self->node1->y, self->node2->y, other->node1->y) ||
                 tools_is_between(self->node1->y, self->node2->y, other->node2->y));
    }
    if (!isfinite(self->a)) {
        cross_x = self->node1->x;
        cross_y = other->a * cross_x + other->b;
    } else if (!isfinite(other->a)) {
        cross_x = other->node1->x;
        cross_y = self->a * cross_x + self->b;
    } else if (!tools_quasi_equal(self->a, other->a)) {
        cross_x = (other->b - self->b) / (self->a - other->a);
        cross_y = self->a * cross_x + self->b;
    } else {
        /* Two segments on the same line */
        return tools_quasi_equal(self->b, other->b) &&
                (tools_is_between(self->node1->x, self->node2->x, other->node1->x) ||
                 tools_is_between(self->node1->x, self->node2->x, other->node2->x));
    }

    return edge_contains(self, cross_x, cross_y) && edge_contains(other, cross_x, cross_y);
}


/* Zone */

typedef struct _Zone
{
    Node** nodes;
    Edge** edges;
    int nodes_count;
    int id;
    int enabled;
} Zone;


static Zone* zone_new(int id, PyObject* points_list)
{
    Zone* self = (Zone*) malloc(sizeof(Zone));
    PyObject* iterator = PyObject_GetIter(points_list);
    PyObject* item = NULL;
    self->id = id;
    self->enabled = 1;
    self->nodes = (Node**) malloc(PySequence_Length(points_list) * sizeof(Node*));
    self->nodes_count = 0;
    self->edges = (Edge**) malloc(PySequence_Length(points_list) * sizeof(Edge*));

    while (item = PyIter_Next(iterator)) {
        float x = 0.0;
        float y = 0.0;
        if (!PyArg_ParseTuple(item, "ff", &x, &y)) {
            return NULL;
        }
        self->nodes[self->nodes_count++] = node_new(x, y);
        Py_DECREF(item);
    }

    Py_DECREF(iterator);

    return self;
}


static void zone_free(Zone* self)
{
    free(self->edges);
    free(self->nodes);
    free(self);
}


/* Pathfinder methods */

typedef struct _PathFinder
{
    PyObject_HEAD
    int is_field_config_done;
    Node* start_node;
    Node* end_node;
    Node** nodes;
    int nodes_count;
    Zone** zones;
    int zones_count;
    Edge** edges;
    int edges_count;
} PathFinder;


static int pathfinder_init(PathFinder* self, PyObject* args, PyObject* kwds)
{
    float field_x1 = 0.0;
    float field_y1 = 0.0;
    float field_x2 = 0.0;
    float field_y2 = 0.0;
    if (!PyArg_ParseTuple(args, "ffff", &field_x1, &field_y1, &field_x2, &field_y2)) {
        return -1;
    }

    self->is_field_config_done = 0;
    self->nodes = array_new(Node*, 6);
    self->nodes_count = 0;
    self->zones = NULL;
    self->zones_count = 0;
    self->edges = NULL;
    self->edges_count = 0;

    /* Field nodes */
    self->nodes[self->nodes_count++] = node_new(field_x1, field_y1);
    self->nodes[self->nodes_count++] = node_new(field_x1, field_y2);
    self->nodes[self->nodes_count++] = node_new(field_x2, field_y2);
    self->nodes[self->nodes_count++] = node_new(field_x2, field_y1);
    self->start_node = node_new(1.0, 0.5);
    self->nodes[self->nodes_count++] = self->start_node;
    self->end_node = node_new(1.0, 2.0);
    self->nodes[self->nodes_count++] = self->end_node;

    return 0;
}


static void pathfinder_dealloc(PathFinder* self)
{
    int i = 0;

    for (i = 0; i < self->nodes_count; ++i) {
        node_free(self->nodes[i]);
    }
    free(self->nodes);

    for (i = 0; i < self->zones_count; ++i) {
        zone_free(self->zones[i]);
    }
    free(self->zones);

    for (i = 0; i < self->edges_count; ++i) {
        edge_free(self->edges[i]);
    }
    free(self->edges);

    Py_TYPE(self)->tp_free((PyObject*) self);
}


static PyObject* pathfinder_add_zone(PathFinder* self, PyObject* args)
{
    int i = 0;
    PyObject* points_list = NULL;

    if (self->is_field_config_done) {
        PyErr_SetString(PyExc_RuntimeError, "Setup already finished. Adding new zones is forbidden");
        return NULL;
    }

    if (!PyArg_ParseTuple(args, "O", &points_list)) {
        return NULL;
    }

    Zone* zone = zone_new(self->zones_count, points_list);
    self->zones = array_ensure_capacity(self->zones, Zone*, self->zones_count, self->zones_count + 1);
    self->zones[self->zones_count] = zone;
    ++self->zones_count;

    self->nodes = array_ensure_capacity(self->nodes, Node*, self->nodes_count, self->nodes_count + zone->nodes_count);
    for (i = 0; i < zone->nodes_count; ++i) {
        self->nodes[self->nodes_count++] = zone->nodes[i];
    }

    return PyLong_FromLong(zone->id);
}


static PyObject* pathfinder_get_edges(PathFinder* self)
{
    int i = 0;
    PyObject* edge_list = PyList_New(0);

    for (i = 0; i < self->edges_count; ++i) {
        Edge* edge = self->edges[i];
        if (edge->allowed) {
            PyList_Append(edge_list, Py_BuildValue("f", edge->node1->x));
            PyList_Append(edge_list, Py_BuildValue("f", edge->node1->y));
            PyList_Append(edge_list, Py_BuildValue("f", edge->node2->x));
            PyList_Append(edge_list, Py_BuildValue("f", edge->node2->y));
            PyList_Append(edge_list, Py_BuildValue("f", 0.0));
        }
    }

    return edge_list;
}


static Edge* pathfinder_fetch_edge(PathFinder* self, Node* node1, Node* node2)
{
    Edge* edge = NULL;
    int i = 0;
    for (i = 0; i < self->edges_count; ++i) {
        edge = self->edges[i];
        if (edge_links(edge, node1, node2)) {
            return edge;
        }
    }
    edge = edge_new(node1, node2);
    self->edges[self->edges_count++] = edge;

    return edge;
}


static void pathfinder_synchronize(PathFinder* self)
{
    int i = 0;
    int j = 0;
    int k = 0;

    for (i = 0; i < self->edges_count; ++i) {
        edge_update(self->edges[i]);
    }

    for (i = 0; i < self->edges_count; ++i) {
        Edge* edge1 = self->edges[i];
        if (edge1->allowed) {
            for (j = 0; j < self->zones_count && edge1->allowed; ++j) {
                Zone* zone = self->zones[j];
                if (zone->enabled) {
                    for (k = 0; k < zone->nodes_count && edge1->allowed; ++k) {
                        Edge* edge2 = zone->edges[k];
                        if (edge1 != edge2) {
                            if (edge_intersects(edge1, edge2)) {
                                edge1->allowed = 0;
                            }
                        }
                    }
                }
            }
        }
    }
}


static PyObject* pathfinder_field_config_done(PathFinder* self)
{
    int i = 0;
    int j = 0;
    int max_edges = 0;
    Node* node = NULL;
    Zone* zone = NULL;

    if (self->is_field_config_done) {
        Py_RETURN_NONE;
    }

    max_edges = self->nodes_count * (self->nodes_count - 1) / 2;
    self->edges = (Edge**) malloc(max_edges * sizeof(Edge*));
    for (i = 0; i < self->nodes_count; ++i) {
        node_create_edges_array(self->nodes[i], self->nodes_count - 1);
    }
    for (i = 0; i < self->nodes_count; ++i) {
        Node* node1 = self->nodes[i];
        for (j = 0; j < self->nodes_count; ++j) {
            Node* node2 = self->nodes[j];
            if (node1 != node2) {
                Edge* edge = pathfinder_fetch_edge(self, node1, node2);
                node_add_edge(node1, edge);
            }
        }
    }
    for (i = 0; i < self->zones_count; ++i) {
        Zone* zone = self->zones[i];
        Node* previous_node = zone->nodes[zone->nodes_count - 1];
        for (j = 0; j < zone->nodes_count; ++j) {
            Node* node = zone->nodes[j];
            Edge* edge = pathfinder_fetch_edge(self, previous_node, node);
            zone->edges[j] = edge;
            previous_node = node;
        }
    }

    pathfinder_synchronize(self);

    self->is_field_config_done = 1;

    Py_RETURN_NONE;
}


/****************************/
/* Python object definition */
/****************************/


/* Object methods */
static PyMethodDef PathFinder_methods[] = {
    { "add_zone"                         , (PyCFunction) pathfinder_add_zone                         , METH_VARARGS, "Add a zone. Takes a list of points (x, y) as argument" },
    { "field_config_done"                , (PyCFunction) pathfinder_field_config_done                , METH_NOARGS , "Field config done. prepare data for pathfinding requests" },
    { "get_edges"                        , (PyCFunction) pathfinder_get_edges                        , METH_NOARGS , "returns the list of edges [x1, y1, x2, y2, ...]" },
/*
    { "find"                             , (PyCFunction) pathfinder_find                             , METH_VARARGS, "Find the route from (x1, y1) to (x2, y2)" },
    { "find"                             , (PyCFunction) pathfinder_find                             , METH_VARARGS, "Find the route from (x1, y1) to (x2, y2)" },
    { "add_forbidden_rect"               , (PyCFunction) pathfinder_add_forbidden_rect_zone          , METH_VARARGS, "Add a forbidden rect (x1, y1) to (x2, y2)" },
    { "add_forbidden_circle"             , (PyCFunction) pathfinder_add_forbidden_circle_zone        , METH_VARARGS, "Add a forbidden circle centered at (x, y), radius r" },
    { "add_penalized_rect"               , (PyCFunction) pathfinder_add_penalized_rect_zone          , METH_VARARGS, "Add a penalized rect (x1, y1) to (x2, y2) with the given cost" },
    { "add_penalized_circle"             , (PyCFunction) pathfinder_add_penalized_circle_zone        , METH_VARARGS, "Add a penalized circle centered at (x, y), radius r" },
    { "set_main_opponent_position"       , (PyCFunction) pathfinder_set_main_opponent_position       , METH_VARARGS, "Sets the main opponent position" },
    { "clear_main_opponent_position"     , (PyCFunction) pathfinder_clear_main_opponent_position     , METH_NOARGS , "Clears the main opponent position" },
    { "set_secondary_opponent_position"  , (PyCFunction) pathfinder_set_secondary_opponent_position  , METH_VARARGS, "Sets the secondary opponent position" },
    { "clear_secondary_opponent_position", (PyCFunction) pathfinder_clear_secondary_opponent_position, METH_NOARGS , "Clears the main opponent position" },
*/
    {NULL}  /* Sentinel */
};


/* Object members */
static PyMemberDef PathFinder_members[] = {
    {NULL}  /* Sentinel */
};


/* Object definition */
static PyTypeObject PathFinderType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "graphpathfinding.PathFinder",            /* tp_name           */
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


static PyModuleDef graphpathfindingmodule = {
    PyModuleDef_HEAD_INIT,
    "graphpathfinding",               /* m_name     */
    "Graph based pathfinding module", /* m_doc      */
    -1,                               /* m_size     */
    NULL,                             /* m_methods  */
    NULL,                             /* m_reload   */
    NULL,                             /* m_traverse */
    NULL,                             /* m_clear    */
    NULL                              /* m_free     */
};


PyMODINIT_FUNC PyInit_graphpathfinding(void)
{
    PyObject* m;

    if (PyType_Ready(&PathFinderType) < 0)
        return;

    m = PyModule_Create(&graphpathfindingmodule);
    if (m == NULL)
        return NULL;

    Py_INCREF(&PathFinderType);
    PyModule_AddObject(m, "PathFinder", (PyObject*) &PathFinderType);

    return m;
}