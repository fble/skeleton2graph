#include "skeleton2graph.h"

#define buffer 500

//static char *filein = "/data/stud-lama1053/skeletons/Membran_1_Porenraum/skeleton.dat";
//static char* filein = "/data/stud-lama1053/skeletons/Isotropie_Gitter_Cube_20_20_Skelett_Matthieu/Skelett_Stege/skeleton.dat";
static char* filein;
static char *fileout = "/data/stud-lifa1015/output/";
static long threshold = 0;

argument_t arguments[] = {
        {NULL,        ' ', PARAM_REQUIRED, "", NULL, "Skeleton file to process",             PARAM_FILEIN,  &filein},
//        {NULL,        ' ', PARAM_REQUIRED, "", NULL, "Folder for the result output",          PARAM_FILEOUT, &fileout},
        {"threshold",  't', PARAM_OPTIONAL, "", NULL, "Threshold for the result, writes only graphs with nodes > threshold",PARAM_PERIODICFLAG, &threshold},
};

toolparam_t tool = {
        .description   = "",
        .example       = "",
        .istested      = 0,
        .arguments     = arguments
};

//searches the id of an already existing node
long findNodeID(ComplexGraph *graph, Voxel voxel) {
    Hashmapelement *currentelement;
    HASHMAP_FOREACH(graph->nodes, currentelement){
        complexnode_t *value = currentelement->value;
        Vertex *vertex = value->nodeinfo.data;
        for (int j = 0; j < vertex->voxel.size; ++j) {
            Voxel vertexVoxel = Datavector_at(vertex->voxel, j);
            if (vertexVoxel.x == voxel.x && vertexVoxel.y == voxel.y && vertexVoxel.z == voxel.z) {
                return currentelement->key;
            }
        }
    }
    return -1;
}

void init_stage(int x_max, int y_max, int z_max) {
    x_size = x_max;
    y_size = y_max;
    z_size = z_max;

    stage = Calloc(x_size * y_size * z_size, sizeof(Pre_Node *));
}

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

Neighbors_Vector find_all_neighbors(Voxel voxel) {
    Neighbors_Vector neighbors;
    Datavector_init(neighbors, 10);
    int x = voxel.x;
    int y = voxel.y;
    int z = voxel.z;
    double r = voxel.r;
    //distance 1
    for (int i = 0; i < 6; ++i) {
        int patternX = x + pattern_dist1[i][0];
        int patternY = y + pattern_dist1[i][1];
        int patternZ = z + pattern_dist1[i][2];
        Pre_Node *stageNode = get_StageNode(patternX, patternY, patternZ);
        if (stageNode != NULL) {
            Datavector_pushBack(neighbors, stageNode);

        }
    }
    //distance sqrt 2
    for (int i = 0; i < 12; ++i) {
        int patternX = x + pattern_dist2[i][0];
        int patternY = y + pattern_dist2[i][1];
        int patternZ = z + pattern_dist2[i][2];
        Pre_Node *stageNode = get_StageNode(patternX, patternY, patternZ);
        if (stageNode != NULL) {
            if (stageNode->voxel.r + r >= distance2) {
                Datavector_pushBack(neighbors, stageNode);
            }
        }
    }
    //distance sqrt 3
    for (int i = 0; i < 8; ++i) {
        int patternX = x + pattern_dist3[i][0];
        int patternY = y + pattern_dist3[i][1];
        int patternZ = z + pattern_dist3[i][2];
        Pre_Node *stageNode = get_StageNode(patternX, patternY, patternZ);
        if (stageNode != NULL) {
            if (stageNode->voxel.r + r >= distance3) {
                Datavector_pushBack(neighbors, stageNode);
            }
        }
    }
    return neighbors;
}

Neighbors_Vector find_new_neighbors(Voxel node) {
    Neighbors_Vector allNeighbors = find_all_neighbors(node);
    Neighbors_Vector newNeighbors;
    Datavector_init(newNeighbors, 2);
    for (int i = 0; i < allNeighbors.size; ++i) {
        Pre_Node *preNode = Datavector_at(allNeighbors, i);
        if (preNode->visited == false) {
            Datavector_pushBack(newNeighbors, preNode);
        }
    }
    Datavector_deinit(allNeighbors);
    return newNeighbors;
}

//the input node is already part of the vertex, searches vor all other voxels and adds them to the vertex
void generate_vertex(ComplexGraph *graph, Pre_Node *pre_node, int nodeID) {
    Vertex *vertex = malloc(sizeof(Vertex));
    Datavector_init(vertex->voxel, 7);
    Neighbors_Vector allNeighbors = find_all_neighbors(pre_node->voxel);
    if(allNeighbors.size != 2){
        if(allNeighbors.size == 1){
            vertex->type = Endpoint;
        }else{
            vertex->type = Junction;
        }
        Neighbors_Vector neighbors = find_new_neighbors(pre_node->voxel);
        Datavector_pushBack(vertex->voxel, pre_node->voxel);
        visit_voxel(pre_node->voxel);
        while (neighbors.size > 0) {
            Pre_Node *node = Datavector_at(neighbors, 0);
            Datavector_erase(neighbors, 0);
            Neighbors_Vector allNeighbors = find_all_neighbors(node->voxel);
            if(allNeighbors.size != 2){
                Datavector_pushBack(vertex->voxel,node->voxel);
                visit_voxel(node->voxel);
                Neighbors_Vector newNeighbors = find_new_neighbors(node->voxel);
                for (int i = 0; i < newNeighbors.size; ++i) {
                    Pre_Node * node = Datavector_at(newNeighbors,i);
                    if(!contains_neighbor(neighbors,node->voxel) && node->visited == false){
                        printf("%d;%d;%d\n",node->voxel.x,node->voxel.y,node->voxel.z);
                        Datavector_pushBack(neighbors,node);
                    }
                }
                Datavector_deinit(newNeighbors);
            }
        }
        ComplexGraph_addData2Node(graph, nodeID, vertex);
        for (int i = 0; i < vertex->voxel.size; ++i) {
            Voxel voxel = Datavector_at(vertex->voxel, i);
            Neighbors_Vector allNeighbors = find_all_neighbors(voxel);
            if(allNeighbors.size == 2){
                printf("Vertex Error\n");
            }
        }
        Datavector_deinit(neighbors);
    }else{
        printf("Edge Error");
    }
    Datavector_deinit(allNeighbors);

}


void visit_voxel(Voxel voxel) {
    int x = voxel.x;
    int y = voxel.y;
    int z = voxel.z;
    Pre_Node *stageNode = get_StageNode(x, y, z);
    stageNode->visited = true;
}

//gets the next node on the edge
Voxel *get_next_edgeNode(Voxel node) {
    Neighbors_Vector newNeighbors = find_new_neighbors(node);
    if (newNeighbors.size == 1) {
        Voxel *voxel = &Datavector_at(newNeighbors, 0)->voxel;
        Neighbors_Vector allNeighbors = find_all_neighbors(*voxel);
        if (allNeighbors.size == 2) {
            Datavector_deinit(newNeighbors);
            Datavector_deinit(allNeighbors);
            return voxel;
        }
    }
    return NULL;
}

//the return is the first voxel of the vertex which is the anchor of the edge
Pre_Node *follow_edge(ComplexGraph *graph, Edge *edge) {
    Voxel node = Datavector_at(edge->slabs, 1);
    Neighbors_Vector allNeighbors = find_all_neighbors(node);
    Voxel *previousNode = NULL;
    // an edge voxel has exactly 2 neighbors
    if (allNeighbors.size == 2) {
        Datavector_deinit(allNeighbors);
        Voxel *edgeNode = &node;
        while(edgeNode != NULL){
            previousNode = edgeNode;
            visit_voxel(*edgeNode);
            Datavector_pushBack(edge->slabs,*edgeNode);
            edgeNode = get_next_edgeNode(*edgeNode);
        }
        Neighbors_Vector potentialVertices = find_all_neighbors(*previousNode);
        for (int i = 0; i < potentialVertices.size; ++i) {
            Pre_Node *potentialVertex = Datavector_at(potentialVertices, i);
            Neighbors_Vector neighbors = find_all_neighbors(potentialVertex->voxel);
            if(neighbors.size != 2){
                Datavector_deinit(potentialVertices);
                Datavector_deinit(neighbors);
                return potentialVertex;
            }
        }
    } else{
        return get_StageNode(node.x, node.y, node.z);
    }
}

bool contains_neighbor(Neighbors_Vector vector, Voxel voxel) {
    for (int i = 0; i < vector.size; ++i) {
        Pre_Node *tmp = Datavector_at(vector, i);
        if (tmp->voxel.x == voxel.x && tmp->voxel.y == voxel.y && tmp->voxel.z == voxel.z) {
            return true;
        }
    }
    return false;
}

//searches for all outgoing edges from the vertex
Edge_Vector generate_edges(ComplexGraph *graph, int nodeID) {
    Edge_Vector edge_vector;
    Datavector_init(edge_vector, 10);
    Neighbors_Vector neighbors;
    Datavector_init(neighbors, 6);
    Vertex *vertex = (Vertex *) ComplexGraph_getDataFromNode(graph, nodeID);
    for (int i = 0; i < vertex->voxel.size; ++i) {
        Neighbors_Vector new_neighbors = find_new_neighbors(Datavector_at(vertex->voxel, i));
        for (int j = 0; j < new_neighbors.size; ++j) {
            Pre_Node *edge_node = Datavector_at(new_neighbors, j);
            if (edge_node->visited == false && !contains_neighbor(neighbors, edge_node->voxel)) {
                Datavector_pushBack(neighbors, edge_node);
                Edge *edge = malloc(sizeof(Edge));
                edge->startNodeID = nodeID;
                Datavector_init(edge->slabs, 10);
                Datavector_pushBack(edge->slabs, Datavector_at(vertex->voxel, i));
                Datavector_pushBack(edge->slabs, edge_node->voxel);
                Datavector_pushBack(edge_vector, edge);
            }
        }
        Datavector_deinit(new_neighbors);
    }
    Datavector_deinit(neighbors);

    return edge_vector;
}

ComplexGraph *build_graph(Pre_Node *pre_node) {
    ComplexGraph *graph = malloc(sizeof(ComplexGraph));
    ComplexGraph_init(graph, 0, 0);
    resetNodeID();
    resetEdgeID();
    int nodeID = getNodeID();
    ComplexGraph_addNode(graph, nodeID);
    generate_vertex(graph, pre_node, nodeID);
    Edge_Vector edges = generate_edges(graph, nodeID);
    while (edges.size > 0) {
        Edge_Vector new_edges = build_subgraph(graph, Datavector_at(edges, 0));
        Datavector_erase(edges, 0);
        for (int i = 0; i < new_edges.size; ++i) {
            Datavector_pushBack(edges, Datavector_at(new_edges, i));
        }
    }
    Datavector_deinit(edges);
    return graph;
}

//builds from one vertex over an edge to an other vertex
Edge_Vector build_subgraph(ComplexGraph *graph, Edge *edge) {
    Pre_Node *node = follow_edge(graph, edge);
    Edge_Vector edges;
    edges.size = 0;
    int edgeID = getEdgeID();
    int startNodeID = edge->startNodeID;
    int endNodeID;
    if (node != NULL) {
        if (node->visited == false) {
            endNodeID = getNodeID();
            ComplexGraph_addNode(graph, endNodeID);
            generate_vertex(graph, node, endNodeID);
            Vertex *vertex = (Vertex *) ComplexGraph_getDataFromNode(graph, endNodeID);
            if (vertex->type == Junction) {
                edges = generate_edges(graph, endNodeID);
            }
        } else {
            endNodeID = findNodeID(graph, node->voxel);
        }
        if (endNodeID != -1) {
            ComplexGraph_addEdge(graph, startNodeID, endNodeID, edgeID);
            ComplexGraph_addData2Edge(graph, edgeID, edge);
            calculate_edge_length(graph, edge);
        } else {
            printf("error\n");
        }
    }
    return edges;
}

int cmp_x(const void *a, const void *b) {
    Pre_Node *node1 = (Pre_Node *) a;
    Pre_Node *node2 = (Pre_Node *) b;
    return node1->voxel.x - node2->voxel.x;
}

void readFile(int *x_max, int *y_max, int *z_max) {
    (*x_max) = 0;
    (*y_max) = 0;
    (*z_max) = 0;
    Datavector_init(pre_nodes, 50);
    FILE *dat_skeleton;
    char *split = ", ";
    char line[250];
    dat_skeleton = fopen(filein, "r");
    if (dat_skeleton == NULL) {
        printf("File not found\n");
    }
    while (fgets(line, sizeof(line), dat_skeleton) != NULL) {
        Voxel voxel;
        char *x = strtok(line, split);
        char *y = strtok(NULL, split);
        char *z = strtok(NULL, split);
        char *r = strtok(NULL, split);
        voxel.x = strtol(x, NULL, 10);
        voxel.y = strtol(y, NULL, 10);
        voxel.z = strtol(z, NULL, 10);
        voxel.r = strtod(r, NULL);
        Pre_Node pre_node;
        pre_node.voxel = voxel;
        pre_node.visited = false;
        Datavector_pushBack(pre_nodes, pre_node);
        if (voxel.x > (*x_max))
            (*x_max) = voxel.x;
        if (voxel.y > (*y_max))
            (*y_max) = voxel.y;
        if (voxel.z > (*z_max))
            (*z_max) = voxel.z;
    }
    fclose(dat_skeleton);
}

double euklid_distance(Voxel voxel1, Voxel voxel2) {
    return sqrt(pow(voxel1.x - voxel2.x, 2) + pow(voxel1.y - voxel2.y, 2) + pow(voxel1.z - voxel2.z, 2));
}

void calculate_edge_length(ComplexGraph *graph, Edge *edge) {
    double edgeLength = 0;
    if (edge->slabs.size >= 2) {
        for (int i = 0; i < edge->slabs.size - 1; ++i) {
            Voxel voxel1 = Datavector_at(edge->slabs, i);
            Voxel voxel2 = Datavector_at(edge->slabs, i + 1);
            edgeLength += euklid_distance(voxel1, voxel2);
        }
    }
    edge->length = edgeLength;
}

int cmp_graph(const void *a, const void *b) {
    ComplexGraph *graph_a = *(ComplexGraph **) a;
    ComplexGraph *graph_b = *(ComplexGraph **) b;
    return (graph_b->nodes->allelements - graph_a->nodes->allelements);
}

double calculate_angle_x(Vector3D vector) {
    return fabs(asin(vector.x / Vector3D_abs(vector)) * 180 / M_PI);
}

double calculate_angle_y(Vector3D vector) {
    return fabs(asin(vector.y / Vector3D_abs(vector)) * 180 / M_PI);
}

double calculate_angle_z(Vector3D vector) {
    return fabs(asin(vector.z / Vector3D_abs(vector)) * 180 / M_PI);
}

/**
 * @brief Calculates the minimum, maximum and average radius of an Edge
 * @param edge Input Edge
 * @param r_min Output minimum radius
 * @param r_max Output maximum radius
 * @param r_avg Output average radius
 */
void calc_minmaxavg_radius(Edge *edge, double *r_min, double *r_max, double *r_avg) {
    *r_min = DBL_MAX;
    *r_max = 0;
    *r_avg = 0;

    for (int i = 0; i < edge->slabs.size; ++i) {
        Voxel node = Datavector_at(edge->slabs, i);
        *r_avg += node.r;
        if (node.r > *r_max) {
            *r_max = node.r;
        }
        if (node.r < *r_min) {
            *r_min = node.r;
        }
    }

    *r_avg /= edge->slabs.size;
}

/**
 *
 * @param radius
 * @return
 */
inline double calc_sphere_volume(double radius) {
    return 4.0/3.0 * M_PI * pow(radius, 3);
}

Vertex_Vector get_allVertices(ComplexGraph *graph) {
    Vertex_Vector vertices;
    Datavector_init(vertices, 100);
    complexnode_t **values;
    long size = graph->nodes->allelements;
    Hashmap_getAllValues(graph->nodes, (void ***) &values, &size);
    for (int i = 0; i < size; ++i) {
        Datavector_pushBack(vertices, (Vertex *) values[i]->nodeinfo.data);
    }
    return vertices;
}

Edge_Vector get_allEdges(ComplexGraph *graph) {
    Edge_Vector edges;
    Datavector_init(edges, 100);
    graphedge_t **values;
    long size = graph->edges->allelements;
    Hashmap_getAllValues(graph->edges, (void ***) &values, &size);
    for (int i = 0; i < size; ++i) {
        Datavector_pushBack(edges, (Edge *) values[i]->data);
    }
    return edges;
}

Vertex_Vector get_junctions(ComplexGraph *graph) {
    Vertex_Vector junctions;
    Datavector_init(junctions, 100);
    Vertex_Vector vertices = get_allVertices(graph);
    for (int i = 0; i < vertices.size; ++i) {
        Vertex *vertex = Datavector_at(vertices, i);
        if (vertex->type == Junction) {
            Datavector_pushBack(junctions, vertex);
        }
    }
    return junctions;
}


void create_csv(Graph_Vector graph_vector, char *filename_path) {
    struct stat st = {0};
    if (stat(filename_path, &st) == -1) {
        mkdir(filename_path, 0700);
    }
    for (int i = 0; i < graph_vector.size; ++i) {
        ComplexGraph *graph = Datavector_at(graph_vector, i);
        if(get_allVertices(graph).size > threshold){
            setlocale(LC_NUMERIC, "");
            char file[buffer];
            sprintf(file, "%sGraph_%d", filename_path, i);
            FILE *csv_overview = fopen(strcat(file, "_overview.csv"), "w+");
            int number_of_endpoints = 0;
            int number_of_junctions = 0;
            int number_of_junction_voxels = 0;
            int number_of_slabs = 0;
            Vertex_Vector vertices = get_allVertices(graph);
            Edge_Vector edges = get_allEdges(graph);
            for (int i = 0; i < vertices.size; ++i) {
                Vertex *vertex = Datavector_at(vertices, i);
                if (vertex->type == Junction) {
                    number_of_junctions++;
                    number_of_junction_voxels += vertex->voxel.size;
                }
                if (vertex->type == Endpoint) {
                    number_of_endpoints++;
                }
            }
            for (int i = 0; i < edges.size; ++i) {
                Edge *edge = Datavector_at(edges, i);
                number_of_slabs += edge->slabs.size - 2;
            }
            fprintf(csv_overview, "#Vertices;#Junctions;#Junction Voxels;#Endpoints;#Edges;#Slab Voxels\n");
            fprintf(csv_overview, "%zu;%d;%d;%d;%zu;%d\n", vertices.size, number_of_junctions,
                    number_of_junction_voxels,
                    number_of_endpoints, edges.size, number_of_slabs);
            fprintf(csv_overview, "\n");
            fprintf(csv_overview, "Average length;Median length\n");
            fprintf(csv_overview, "=MITTELWERT(A7:A%zu);=MEDIAN(A7:A%zu)\n\n", edges.size + 6,
                    edges.size + 6);
            fprintf(csv_overview,
                    "Edges Length;Average radius;Min radius;Max radius;Start;End;Normalized orientation;Angle yz-plane;Angle xz-plane;Angle xy-plane;Junction center;Junction radius\n");
            Vertex_Vector junctions = get_junctions(graph);
            double max_size = fmax(edges.size, junctions.size);
            for (int i = 0; i < max_size; ++i) {
                if (i < edges.size) {
                    Edge *edge = Datavector_at(edges, i);
                    double average_radius = 0;
                    double max_radius = 0;
                    double min_radius = DBL_MAX;
                    for (int i = 0; i < edge->slabs.size; ++i) {
                        Voxel node = Datavector_at(edge->slabs, i);
                        average_radius += node.r;
                        if (node.r > max_radius) {
                            max_radius = node.r;
                        }
                        if (node.r < min_radius) {
                            min_radius = node.r;
                        }
                    }
                    average_radius /= edge->slabs.size;
                    fprintf(csv_overview, "%f;%f;%f;%f;", edge->length, average_radius, min_radius, max_radius);
                    Voxel start = Datavector_at(edge->slabs, 0);
                    fprintf(csv_overview, "%d,%d,%d;", start.x, start.y, start.z);
                    Voxel end = Datavector_at(edge->slabs, edge->slabs.size - 1);
                    fprintf(csv_overview, "%d,%d,%d;", end.x, end.y, end.z);
                    double distance = euklid_distance(start, end);
                    Vector3D vector;
                    vector.x = (end.x - start.x) / distance;
                    vector.y = (end.y - start.y) / distance;
                    vector.z = (end.z - start.z) / distance;
                    fprintf(csv_overview, "%.2f %.2f %.2f;%.2f;%.2f;%.2f;", vector.x, vector.y, vector.z,
                            calculate_angle_x(vector), calculate_angle_y(vector), calculate_angle_z(vector));
                } else {
                    fprintf(csv_overview, ";;;;;;;;;;");
                }
                if (i < junctions.size) {
                    Vertex *junction = Datavector_at(junctions, i);
                    Voxel node = calculate_center(*junction);
                    fprintf(csv_overview, "%d,%d,%d;%f\n", node.x, node.y, node.z, node.r);
                } else {
                    fprintf(csv_overview, "\n");
                }
            }
            fclose(csv_overview);
        }
    }
    Datavector_deinit(graph_vector);
}


void write_node1(ComplexGraph* graph, FILE* node1) {
    fprintf(node1, "%zu %d %d %d\n", graph->nodes->allelements, x_size, y_size, z_size);

    for (long node_id = 0; node_id < graph->nodes->allelements; ++node_id) {
        complexnode_t* current_complexnode;
        Hashmap_get(graph->nodes, node_id, (void **) &current_complexnode);
        node_t node = current_complexnode->nodeinfo;
        Vertex* vertex = node.data;

        // get center voxel of vertex
        Voxel centervoxel = calculate_center(*vertex);

        // write id, pos and conn number
        fprintf(node1, "%li %d %d %d %li",
                node_id + 1,                // pore id
                centervoxel.x,  // pore x coordinate
                centervoxel.y,  // pore y coordinate
                centervoxel.z,  // pore z coordinate
                node.neighbourscount);  // amount of direct neighbours of the pore

        // write id of every neighbour pore
        for (int i = 0; i < node.neighbourscount; ++i) {
            long id_a = current_complexnode->neighbours[i]->a->nodeinfo.id;
            long id_b = current_complexnode->neighbours[i]->b->nodeinfo.id;

            // only write the id of the neighbour and not of the current pore
            if (id_a != node_id) {
                fprintf(node1, " %li", id_a + 1);
            } else {
                fprintf(node1, " %li", id_b + 1);
            }
        }

        // write inlet and outlet status
        fprintf(node1, " %i %i",
                0,   // TODO: Kriterium, um inlet zu definieren fehlt noch
                0);  // TODO: Kriterium, um outlet zu definieren fehlt noch

        // write throat ids
        for (int i = 0; i < node.neighbourscount; ++i) {
            fprintf(node1, " %li", current_complexnode->neighbours[i]->id + 1);
        }

        // write newline
        fprintf(node1, "\n");

    }
}

void write_node2(ComplexGraph *graph, FILE *node2) {
    for (long node_id = 0; node_id < graph->nodes->allelements; ++node_id) {
        complexnode_t *current_complexnode;
        Hashmap_get(graph->nodes, node_id, (void **) &current_complexnode);
        node_t node = current_complexnode->nodeinfo;
        Vertex *vertex = node.data;

        // get center voxel of vertex
        Voxel centervoxel = calculate_center(*vertex);

        // write id and other properties
        fprintf(node2, "%li %f %f %f %f\n",
                node_id + 1,                        // pore id
                calc_sphere_volume(centervoxel.r),  // pore volume
                centervoxel.r,                      // pore radius
                1.0 / (4.0 * M_PI),                 // shape factor: spherical/circular: 1 / (4*PI)
                calc_sphere_volume(centervoxel.r)); // TODO: clay volume
    }
}

void write_link1(ComplexGraph* graph, FILE* link1) {
    // write total amount of throats
    fprintf(link1, "%zu\n", graph->edges->allelements);

    graphedge_t* current_edge;
    long max_edge_index = graph->edges->allelements;

    // iterate over every edge in the graph
    // this might not be the most efficient ( O(n^2) )
    for (long edge_id = 0; edge_id < max_edge_index; ++edge_id) {
        if (false == Hashmap_contains(graph->edges, edge_id)) {
            max_edge_index += 1; // current edge_id is not in the map, so bump the max index.
            continue;
        }

        Hashmap_get(graph->edges, edge_id, (void **) &current_edge);
        Vertex* vertex_a = current_edge->a->nodeinfo.data;
        Vertex* vertex_b = current_edge->b->nodeinfo.data;
        // edge->node_a->nodeinfo.data
        //     !=
        // node->nodeinfo.data

        Edge* current_simple_edge = current_edge->data;
        double radius_min = 0, radius_max = 0, radius_avg = 0;
        calc_minmaxavg_radius(current_simple_edge, &radius_min, &radius_max, &radius_avg);

        fprintf(link1, "%li %li %li %f %f %f\n",
                current_edge->id + 1,                                                       // throat id
                current_edge->a->nodeinfo.id + 1,                                           // pore a id
                current_edge->b->nodeinfo.id + 1,                                           // pore b id
                radius_avg,                                                                 // throat radius
                1.0 / (4.0 * M_PI),                                                         // throat shape factor: cylindrical/circular: 1 / (4*PI)
                euklid_distance(calculate_center(*vertex_a), calculate_center(*vertex_b))); // total throat length (pore center to pore center)
    }
}

void write_link2(ComplexGraph* graph, FILE* link2) {
    graphedge_t* current_edge;
    long max_edge_index = graph->edges->allelements;

    for (long edge_id = 0; edge_id < max_edge_index; ++edge_id) {
        if (false == Hashmap_contains(graph->edges, edge_id)) {
            max_edge_index += 1; // current edge_id is not in the map, so bump the max index.
            continue;
        }

        Hashmap_get(graph->edges, edge_id, (void **) &current_edge);
        Vertex* vertex_a = current_edge->a->nodeinfo.data;
        Vertex* vertex_b = current_edge->b->nodeinfo.data;
        double throat_len = euklid_distance(calculate_center(*vertex_a), calculate_center(*vertex_b));

        Edge* current_simple_edge = current_edge->data;
        double radius_min = 0, radius_max = 0, radius_avg = 0;
        calc_minmaxavg_radius(current_simple_edge, &radius_min, &radius_max, &radius_avg);

        double throat_vol = M_PI * pow(radius_avg, 2) * throat_len;

        fprintf(link2, "%li %li %li %f %f %f %f %f\n",
                current_edge->id + 1,              // throat id
                current_edge->a->nodeinfo.id + 1,  // pore a id
                current_edge->b->nodeinfo.id + 1,  // pore b id
                0.0,                               // TODO: length pore 1
                0.0,                               // TODO: length pore 2
                throat_len,                        // TODO: length throat
                throat_vol,                        // throat volume
                throat_vol);                       // TODO: throat clay volume
    }
}

void create_statOil(Graph_Vector graph_vector, char *filename_path) {
    //printf("writing StatOil files...\n");
    for (int i = 0; i < graph_vector.size; ++i) {
        ComplexGraph *graph = Datavector_at(graph_vector, i);
        if (get_allVertices(graph).size > threshold) {
            setlocale(LC_NUMERIC, "en_US.UTF-8");

            char file[buffer];
            sprintf(file, "%sGraph_%d", filename_path, i);

            char output[buffer];
            sprintf(output, "%s%s", file, "_node1.dat");
            FILE *node1 = fopen(output, "w+");
            sprintf(output, "%s%s", file, "_node2.dat");
            FILE *node2 = fopen(output, "w+");
            sprintf(output, "%s%s", file, "_link1.dat");
            FILE *link1 = fopen(output, "w+");
            sprintf(output, "%s%s", file, "_link2.dat");
            FILE *link2 = fopen(output, "w+");

            write_node1(graph, node1);
            fclose(node1);

            write_node2(graph, node2);
            fclose(node2);

            write_link1(graph, link1);
            fclose(link1);

            write_link2(graph, link2);
            fclose(link2);
        }
    }
}

int main(int argc, char *argv[]) {
    PACE3DMAIN(argv[0]);
    getParams(argc, argv, tool, ARGUMENT(arguments));

    // IN
    int x_max, y_max, z_max;
    readFile(&x_max, &y_max, &z_max);

    Datavector_sort(pre_nodes, cmp_x); // Frage: warum werden die pre_nodes hier sortiert?
    init_stage(x_max, y_max, z_max);

    for (int i = 0; i < pre_nodes.size; i++) {
        Voxel node = Datavector_at(pre_nodes, i).voxel;
        stage[(node.x - 1) + x_size * ((node.y - 1) + y_size * (node.z - 1))] = &Datavector_at(pre_nodes, i);
    }

    init_distance();
    Graph_Vector graph_vector;
    Datavector_init(graph_vector, 1);
//    Pre_Node *preNode = &Datavector_at(pre_nodes, 0);
//    if (preNode->visited == false) {
//        ComplexGraph *graph = build_graph(preNode);
//        int id = getNodeID();
//        for (int i = 0; i < id; ++i) {
//            complexnode_t *node;
//            Hashmap_get(graph->nodes, i, (void **) &node);
//            Vertex *vertex = (Vertex *) node->nodeinfo.data;
//            if (vertex != NULL) {
//                Voxel voxel = calculate_center(*vertex);
//                printf("Center: %d,%d,%d\n", voxel.x, voxel.y, voxel.z);
//            } else {
//                printf("error\n");
//            }
//        }
//        get_junctions(graph);
//        printf("%lu\n", graph->nodes->allelements);
//        printf("%lu\n", graph->edges->allelements);
//    }

    for (int i = 0; i < pre_nodes.size; ++i) {

        Pre_Node *preNode = &Datavector_at(pre_nodes, i);
        Neighbors_Vector allNeighbors = find_all_neighbors(preNode->voxel);

        if (preNode->visited == false && allNeighbors.size != 2) {
            ComplexGraph *graph = build_graph(preNode);
//            for (int i = 0; i < graph.vertices.size; ++i) {
//                printf("Vertex Center: ");
//                print_voxel(calculate_center(Datavector_at(graph.vertices, i)));
//            }
            Datavector_pushBack(graph_vector, graph);
        }
        Datavector_deinit(allNeighbors);
    }

    Datavector_sort(graph_vector, cmp_graph);
    for (int i = 0; i < graph_vector.size; ++i) {
        printf("Graph %d: size = %zu\n", i, get_junctions(Datavector_at(graph_vector, i)).size);
    }
    //create_csv(graph_vector, fileout);

    create_statOil(graph_vector,fileout);
    return 0;
}


