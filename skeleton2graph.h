#ifndef HEADER_SKELETON_GRAPH_H_
#define HEADER_SKELETON_GRAPH_H_

#include <locale.h>
#include <wrapper.h>
#include <container/datavector.h>
#include <container/vector3d.h>
#include <container/complexgraph.h>

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

typedef declareVectorStructure(Pre_Node *) Neighbors_Vector;

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

int x_size;
int y_size;
int z_size;

Pre_Node ****stage;

double distance2;

double distance3;

Voxel calculate_center(Vertex vertex);

void init_stage(int x_max, int y_max, int z_max);

Pre_Node *get_StageNode(int x, int y, int z) {
    if ((x > 0) && (y > 0) && (z > 0) && (x <= x_size) && (y <= y_size) && (z <= z_size)) {
        return stage[x - 1][y - 1][z - 1];
    }
    return NULL;
}

void visit_voxel(Voxel voxel);

bool contains_neighbor(Neighbors_Vector vector, Voxel voxel);

Edge_Vector build_subgraph(ComplexGraph *graph, Edge *edge);

void calculate_edge_length(ComplexGraph *graph, Edge *edge);

Vertex_Vector get_allVertices(ComplexGraph *graph);

void init_distance() {
    distance2 = 2 * sqrt(2);
    distance3 = 2 * sqrt(3);
}

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

Vertex_Vector get_junctions(ComplexGraph *graph);

Edge_Vector get_allEdges(ComplexGraph *graph);

double euklid_distance(Voxel voxel1, Voxel voxel2);

ComplexGraph *build_graph(Pre_Node *pre_node);

Edge_Vector generate_edges(ComplexGraph *graph, int nodeID);

Pre_Node *follow_edge(ComplexGraph *graph, Edge *edge);

Voxel *get_next_edgeNode(Voxel node);

void generate_vertex(ComplexGraph *graph, Pre_Node *pre_node, int nodeID);

Neighbors_Vector find_new_neighbors(Voxel node);

Neighbors_Vector find_all_neighbors(Voxel voxel);

long findNodeID(ComplexGraph *graph, Voxel voxel);

#endif /* HEADER_SKELETON_GRAPH_H_ */
