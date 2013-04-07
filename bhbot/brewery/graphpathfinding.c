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

#define TOOLS_EPSILON 0.001


static int tools_quasi_equal(float a, float b)
{
    return fabs(a - b) < TOOLS_EPSILON;
}


static int tools_is_between(float a, float b, float x)
{
    if (tools_quasi_equal(a, x) || tools_quasi_equal(b, x)) {
        return 1;
    }
    if (a < b) {
        return a < x && x < b;
    } else {
        return b < x && x < a;
    }
}


static float tools_distance(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
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
    int is_in_openset;
    int is_in_closedset;
    int enabled;
    struct _Node* next;
    struct _Node* came_from;
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
    self->is_in_openset = 0;
    self->is_in_closedset = 0;
    self->enabled = 1;
    self->next = NULL;
    self->came_from = NULL;
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


static int node_coords_equal(Node* self, Node* other)
{
    return tools_quasi_equal(self->x, other->x) && tools_quasi_equal(self->y, other->y);
}


static Node* node_list_insert_sorted(Node* self, Node* other)
{
    Node* previous = NULL;
    Node* current = NULL;

    /* The list is empty */
    if (self == NULL) {
        other->next = NULL;
        return other;
    }

    /* Middle of the list */
    for (current = self; current != NULL; current = current->next) {
        if (current->f_score >= other->f_score) {
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


static Node* node_list_pop(Node* self)
{
    Node* popped = self;
    self = self->next;
    popped->next = NULL;
    return self;
}


/* Edge */

typedef struct _Edge
{
    Node* node1;
    Node* node2;
    float a;
    float b;
    int allowed;
    float length;
    int zone_internal;
} Edge;


static Edge* edge_new(Node* node1, Node* node2)
{
    Edge* self = (Edge*) malloc(sizeof(Edge));

    self->node1 = node1;
    self->node2 = node2;
    self->a = INFINITY;
    self->b = INFINITY;
    self->allowed = 1;
    self->length = 0.0;
    self->zone_internal = 0;

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


static Node* edge_other_node(Edge* self, Node* node)
{
    if (self->node1 == node) {
        return self->node2;
    } else if (self->node2 == node) {
        return self-> node1;
    }
    return NULL;
}

static void edge_update(Edge* self)
{
    self->allowed = (!self->zone_internal) && self->node1->enabled && self->node2->enabled;
    if (tools_quasi_equal(self->node1->x, self->node2->x)) {
        /* Vertical edge */
        self->a = INFINITY;
        self->b = INFINITY;
    } else {
        /* General case */
        self->a = (self->node2->y - self->node1->y) / (self->node2->x - self->node1->x);
        self->b = self->node1->y - self->a * self->node1->x;
    }
    self->length = tools_distance(self->node1->x, self->node1->y, self->node2->x, self->node2->y);
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
    if (node_coords_equal(self->node1, other->node1) ||
        node_coords_equal(self->node1, other->node2) ||
        node_coords_equal(self->node2, other->node1) ||
        node_coords_equal(self->node2, other->node2)) {
        return 0;
    }
    if (!isfinite(self->a) && !isfinite(other->a)) {
        /* Two vertical lines */
        return tools_quasi_equal(self->node1->x, other->node1->x) &&
                (edge_contains(self, other->node1->x, other->node1->y) ||
                 edge_contains(self, other->node2->x, other->node2->y));
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
    float dx;
    float dy;
} Zone;


static Zone* zone_new(int id, PyObject* points_list)
{
    Zone* self = (Zone*) malloc(sizeof(Zone));
    PyObject* iterator = PyObject_GetIter(points_list);
    PyObject* item = NULL;
    self->nodes = (Node**) malloc(PySequence_Length(points_list) * sizeof(Node*));
    self->edges = (Edge**) malloc(PySequence_Length(points_list) * sizeof(Edge*));
    self->nodes_count = 0;
    self->id = id;
    self->enabled = 1;
    self->dx = 0.0;
    self->dy = 0.0;

    while ((item = PyIter_Next(iterator))) {
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


static int zone_is_internal_edge(Zone* self, Edge* edge)
{
    int i;
    int node1InZone = 0;
    int node2InZone = 0;
    Node* previous_node = self->nodes[self->nodes_count - 1];

    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        if (edge_links(edge, previous_node, node)) {
            return 0;
        }
        node1InZone |= edge->node1 == node;
        node2InZone |= edge->node2 == node;
        if (node1InZone && node2InZone) {
            return 1;
        }

        previous_node = node;
    }

    return 0;
}


static int zone_contains_node(Zone* self, Node* node)
{
    /* WARNING ! This function works only for convex polygon zones */

    Node* node1 = NULL;
    Node* node2 = NULL;
    int i = 0;
    int sign = 0;
    int k = 0;
    float dx1 = 0.0;
    float dy1 = 0.0;
    float dx2 = 0.0;
    float dy2 = 0.0;
    float cross_product = 0.0;

    node1 = self->nodes[self->nodes_count - 1];
    for (i = 0; i < self->nodes_count; ++i) {
        node2 = self->nodes[i];
        if (node1 == node) {
            return 0;
        }
        dx1 = node2->x - node1->x;
        dy1 = node2->y - node1->y;
        dx2 = node->x - node1->x;
        dy2 = node->y - node1->y;
        cross_product = dx1 * dy2 - dy1 * dx2;
        k = cross_product >= 0.0 ? 1 : -1;
        if (sign == 0) {
            sign = k;
        } else if (sign != k) {
            return 0;
        }
        node1 = node2;
    }
    return 1;
}

/* Pathfinder methods */

typedef struct _PathFinder
{
    PyObject_HEAD
    float field_x1;
    float field_y1;
    float field_x2;
    float field_y2;
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
    if (!PyArg_ParseTuple(args, "ffff", &self->field_x1, &self->field_y1, &self->field_x2, &self->field_y2)) {
        return -1;
    }

    self->is_field_config_done = 0;
    self->nodes = array_new(Node*, 2);
    self->nodes_count = 0;
    self->zones = NULL;
    self->zones_count = 0;
    self->edges = NULL;
    self->edges_count = 0;

    /* Field nodes */
    self->start_node = node_new(0.0, 0.0);
    self->nodes[self->nodes_count++] = self->start_node;
    self->end_node = node_new(0.0, 0.0);
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


static PyObject* pathfinder_enable_zone(PathFinder* self, PyObject* args)
{
    int id = 0;
    int enabled = 0;

    if (!PyArg_ParseTuple(args, "ii", &id, &enabled)) {
        return NULL;
    }

    if (id >= 0 && id < self->zones_count) {
        self->zones[id]->enabled = enabled;
    }

    Py_RETURN_NONE;
}


static PyObject* pathfinder_move_zone(PathFinder* self, PyObject* args)
{
    int id = 0;
    float dx = 0.0;
    float dy = 0.0;

    if (!PyArg_ParseTuple(args, "iff", &id, &dx, &dy)) {
        return NULL;
    }

    if (id >= 0 && id < self->zones_count) {
        Zone* zone = self->zones[id];
        zone->dx += dx;
        zone->dy += dy;
    }

    Py_RETURN_NONE;
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


static int pathfinder_is_node_in_field(PathFinder* self, Node* node)
{
    return tools_is_between(self->field_x1, self->field_x2, node->x) &&
           tools_is_between(self->field_y1, self->field_y2, node->y);
}


static void pathfinder_synchronize(PathFinder* self)
{
    int i = 0;
    int j = 0;
    int k = 0;

    /* Apply zone translations */
    for (i = 0; i < self->zones_count; ++i) {
        Zone* zone = self->zones[i];
        for (j = 0; j < zone->nodes_count; ++j) {
            Node* node = zone->nodes[j];
            node->x += zone->dx;
            node->y += zone->dy;
        }
        zone->dx = 0.0;
        zone->dy = 0.0;
    }
    /* Remove nodes outside of field */
    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        node->enabled = pathfinder_is_node_in_field(self, node);
    }
    /* Remove disabled zones nodes */
    for (i = 0; i < self->zones_count; ++i) {
        Zone* zone = self->zones[i];
        for (j = 0; j < zone->nodes_count; ++j) {
            Node* node = zone->nodes[j];
            node->enabled &= zone->enabled;
        }
    }
    /* Remove nodes in a zone*/
    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        if (node->enabled) {
            for (j = 0; j < self->zones_count; ++j) {
                Zone* zone = self->zones[j];
                if (zone->enabled && zone_contains_node(zone, node)) {
                    node->enabled = 0;
                    break;
                }
            }
        }
    }
    /* Start node is always enabled */
    self->start_node->enabled = 1;

    /* Update edge affine params */
    for (i = 0; i < self->edges_count; ++i) {
        edge_update(self->edges[i]);
    }

    /* Remove intersecting edges */
    for (i = 0; i < self->edges_count; ++i) {
        Edge* edge1 = self->edges[i];
        for (j = 0; j < self->zones_count && edge1->allowed; ++j) {
            Zone* zone = self->zones[j];
            if (zone->enabled) {
                for (k = 0; k < zone->nodes_count && edge1->allowed; ++k) {
                    Edge* edge2 = zone->edges[k];
                    if (edge1 != edge2 && edge_intersects(edge1, edge2)) {
                        edge1->allowed = 0;
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

    if (self->is_field_config_done) {
        Py_RETURN_NONE;
    }

    /* Number of edges in a graph of N nodes: N * (N - 1) / 2 */
    max_edges = self->nodes_count * (self->nodes_count - 1) / 2;
    self->edges = (Edge**) malloc(max_edges * sizeof(Edge*));
    /* Create edges arrays on nodes */
    for (i = 0; i < self->nodes_count; ++i) {
        node_create_edges_array(self->nodes[i], self->nodes_count - 1);
    }
    /* Create all edges */
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
    /* Register surrounding edges of each zone */
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
    /* Mark zones internal edges */
    for (i = 0; i < self->edges_count; ++i) {
        Edge* edge = self->edges[i];
        for (j = 0; j < self->zones_count; ++j) {
            Zone* zone = self->zones[j];
            if (zone_is_internal_edge(zone, edge)) {
                edge->zone_internal = 1;
                break;
            }
        }
    }

    pathfinder_synchronize(self);

    self->is_field_config_done = 1;

    Py_RETURN_NONE;
}


float pathfinder_effective_cost(PathFinder* self, Edge* edge)
{
    return edge->length;
}


float pathfinder_heuristic_cost_estimate(PathFinder* self, Node* neighbor)
{
    return tools_distance(neighbor->x, neighbor->y, self->end_node->x, self->end_node->y);
}


static PyObject* pathfinder_find_path(PathFinder* self, PyObject* args)
{
    int i = 0;
    Node* openset = self->start_node;

    if (!PyArg_ParseTuple(args, "ffff", &self->start_node->x,
                                        &self->start_node->y,
                                        &self->end_node->x,
                                        &self->end_node->y)) {
        return NULL;
    }

    pathfinder_synchronize(self);

    for (i = 0; i < self->nodes_count; ++i) {
        Node* node = self->nodes[i];
        node->is_in_openset = 0;
        node->is_in_closedset = 0;
    }
    self->start_node->g_score = 0.0;
    self->start_node->f_score = 0.0;
    self->start_node->h_score = 0.0;
    self->start_node->came_from = NULL;
    self->start_node->next = NULL;
    self->start_node->is_in_openset = 1;

    while (openset != NULL) {
        Node* current = openset;
        openset = node_list_pop(openset);
        if (current == self->end_node) {
            float cost = current->f_score;
            PyObject* path = PyList_New(0);
            for (; current != NULL; current = current->came_from) {
                cost += current->f_score;
                PyList_Insert(path, 0, Py_BuildValue("(ff)", current->x, current->y));
            }
            return Py_BuildValue("(fN)", cost, path);
        }
        current->is_in_openset = 0;
        current->is_in_closedset = 1;

        for (i = 0; i < current->edges_count; ++i) {
            Edge* edge = current->edges[i];
            if (edge->allowed) {
                Node* neighbor = edge_other_node(edge, current);
                if (!neighbor->is_in_closedset) {
                    float tentative_g_score = current->g_score + pathfinder_effective_cost(self, edge);
                    if (!neighbor->is_in_openset) {
                        neighbor->h_score = pathfinder_heuristic_cost_estimate(self, neighbor);
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                        openset = node_list_insert_sorted(openset, neighbor);
                        neighbor->is_in_openset = 1;
                    } else if (tentative_g_score < neighbor->g_score) {
                        neighbor->g_score = tentative_g_score;
                        neighbor->f_score = neighbor->g_score + neighbor->h_score;
                        neighbor->came_from = current;
                    }
                }
            }
        }
    }
    return Py_BuildValue("(fN)", 0.0, PyList_New(0));
}


/****************************/
/* Python object definition */
/****************************/


/* Object methods */
static PyMethodDef PathFinder_methods[] = {
    { "add_zone"                         , (PyCFunction) pathfinder_add_zone                         , METH_VARARGS, "Add a zone. Takes a list of points (x, y) as argument" },
    { "enable_zone"                      , (PyCFunction) pathfinder_enable_zone                      , METH_VARARGS, "Enables/disables a zone identified by id" },
    { "move_zone"                        , (PyCFunction) pathfinder_move_zone                        , METH_VARARGS, "Moves the zone identified by id of (dx, dy)" },
    { "field_config_done"                , (PyCFunction) pathfinder_field_config_done                , METH_NOARGS , "Field config done. prepare data for pathfinding requests" },
    { "get_edges"                        , (PyCFunction) pathfinder_get_edges                        , METH_NOARGS , "returns the list of edges [x1, y1, x2, y2, ...]" },
    { "find_path"                        , (PyCFunction) pathfinder_find_path                        , METH_VARARGS, "Finds a path from (x1, y1) to (x2, y2)" },
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

    if (PyType_Ready(&PathFinderType) < 0) {
        return NULL;
    }

    m = PyModule_Create(&graphpathfindingmodule);
    if (m == NULL) {
        return NULL;
    }

    Py_INCREF(&PathFinderType);
    PyModule_AddObject(m, "PathFinder", (PyObject*) &PathFinderType);

    return m;
}
