#ifndef HEADER_SKELETON_GRAPH_H_
#define HEADER_SKELETON_GRAPH_H_

#include <locale.h>
#include <wrapper.h>
#include <container/datavector.h>
#include <container/vector3d.h>
#include <container/complexgraph.h>

#define buffer 500

char *filename_skeleton = "/mnt/data/stud-lifa1015/membrane3/skeleton.dat";
char *filename_output = "/mnt/data/stud-lifa1015/membrane3/graphs2/";

typedef struct voxel Voxel;

typedef struct pre_node Pre_Node;

typedef struct edge Edge;

typedef struct vertex Vertex;

typedef declareVectorStructure(Voxel) Voxel_Vector;

typedef declareVectorStructure(Pre_Node) Pre_Node_Vector;

typedef declareVectorStructure(Edge*) Edge_Vector;

typedef declareVectorStructure(Vertex *) Vertex_Vector;

typedef declareVectorStructure(Edge*) Edgeptr_Vector;

typedef declareVectorStructure(ComplexGraph *) Graph_Vector;

int global_nodeID = 0;

int global_edgeID = 0;

struct voxel {
    int x;
    int y;
    int z;
    double r;
};

 struct pre_node {
    Voxel voxel;
    bool visited;
};

enum vertex_type {
    Endpoint, Junction
};

struct edge {
    Voxel_Vector slabs;
    double length;
    int startNodeID;
};
//
struct vertex {
    enum vertex_type type;
    Voxel_Vector voxel;
};

Pre_Node_Vector pre_nodes;
Graph_Vector graph_vector;

void print_voxel(Voxel voxel);

Voxel calculate_center(Vertex vertex) {
    int x = 0;
    int y = 0;
    int z = 0;
    for (int i = 0; i < vertex.voxel.size; ++i) {
        x += Datavector_at(vertex.voxel, i).x;
        y += Datavector_at(vertex.voxel, i).y;
        z += Datavector_at(vertex.voxel, i).z;
    }
    double center_x = (double) x / vertex.voxel.size;
    double center_y = (double) y / vertex.voxel.size;
    double center_z = (double) z / vertex.voxel.size;
    double min_distance = DBL_MAX;
    Voxel current_center;
    for (int i = 0; i < vertex.voxel.size; ++i) {
        Voxel node = Datavector_at(vertex.voxel, i);
        double distance = sqrt(pow(center_x - node.x, 2) + pow(center_y - node.y, 2) + pow(center_z - node.z, 2));
        if (distance < min_distance) {
            min_distance = distance;
            current_center = node;
        }
    }
    return current_center;
}

int x_dimensions;
int y_dimensions;
int z_dimensions;
Pre_Node ****stage;

void init_stage(int x_max, int y_max, int z_max) {
    x_dimensions = x_max;
    y_dimensions = y_max;
    z_dimensions = z_max;
    stage = calloc(x_dimensions, sizeof(Pre_Node ***));
    for (int x = 0; x < x_dimensions; ++x) {
        stage[x] = calloc(y_dimensions, sizeof(Pre_Node **));
        for (int y = 0; y < x_dimensions; ++y) {
            stage[x][y] = calloc(z_dimensions, sizeof(Pre_Node *));
        }
    }
}

Pre_Node *get_StageNode(int x, int y, int z) {
    if ((x > 0) && (y > 0) && (z > 0) && (x <= x_dimensions) && (y <= y_dimensions) && (y <= y_dimensions)) {
        return stage[x - 1][y - 1][z - 1];
    }
    return NULL;
}

typedef declareVectorStructure(Pre_Node *)
        Neighbors_Vector;

void visit_voxel(Voxel voxel);

bool is_visited(Voxel voxel);

bool contains_neighbor(Neighbors_Vector vector, Voxel voxel);

void printEdgeStartpoint(Edge_Vector *edges2);

Edge_Vector build_subgraph(ComplexGraph *graph, Edge *edge);

void calculate_edge_length(ComplexGraph *graph, Edge *edge);

Vertex_Vector get_allVertices(ComplexGraph *graph);


double distance2;

double distance3;

void init_distance() {
    distance2 = 2 * sqrt(2);
    distance3 = 2 * sqrt(3);
}

int pattern_dist1[6][3] = {
        {1,  0,  0},
        {-1, 0,  0},
        {0,  1,  0},
        {0,  -1, 0},
        {0,  0,  1},
        {0,  0,  -1}
};


int pattern_dist2[12][3] = {
        {1,  1,  0},
        {1,  -1, 0},
        {-1, 1,  0},
        {-1, -1, 0},
        {1,  0,  1},
        {1,  0,  -1},
        {-1, 0,  1},
        {-1, 0,  -1},
        {0,  1,  1},
        {0,  1,  -1},
        {0,  -1, 1},
        {0,  -1, -1}
};
int pattern_dist3[8][3] = {
        {1,  1,  1},
        {1,  1,  -1},
        {1,  -1, 1},
        {1,  -1, -1},
        {-1, 1,  1},
        {-1, 1,  -1},
        {-1, -1, 1},
        {-1, -1, -1}
};

void create_csv(Graph_Vector graph_vector, char *filename_output);

void readFile(int *x_max, int *y_max, int *z_max);

double calculate_angle_y(Vector3D vector);

int getEdgeID(){
    return global_edgeID++;
}

int getNodeID(){
    return global_nodeID++;
}

void resetNodeID(){
    global_nodeID = 0;
}

void resetEdgeID(){
    global_edgeID = 0;
}

graphedge_t * getEdge(ComplexGraph * graph,int edgeID){
    graphedge_t * edge;
    Hashmap_get(graph->edges,edgeID,(void**)&edge);
    return edge;
}

#endif /* HEADER_SKELETON_GRAPH_H_ */
