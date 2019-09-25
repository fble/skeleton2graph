#include "skeleton2graph.h"


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

void generate_vertex(ComplexGraph *graph, Pre_Node *pre_node, int nodeID) {
    Vertex *vertex = malloc(sizeof(Vertex));
    Datavector_init(vertex->voxel, 7);
    Neighbors_Vector allNeighbors = find_all_neighbors(pre_node->voxel);
    if(allNeighbors.size != 2){
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
                    Datavector_pushBack(neighbors,Datavector_at(newNeighbors,i));
                }
            }
//        if (new_nodes.size > 2) {
//            Datavector_pushBack(vertex->voxel, node->voxel);
//            visit_voxel(node->voxel);
//            for (int i = 0; i < new_nodes.size; ++i) {
//                Voxel new_node = Datavector_at(new_nodes, i)->voxel;
//                Neighbors_Vector newNeighbors = find_all_neighbors(new_node);
//                if (newNeighbors.size > 2 && !contains_neighbor(neighbors, new_node) && !is_visited(new_node)) {
//                    Datavector_pushBack(neighbors, Datavector_at(new_nodes, i));
//                } else {
//                    Datavector_erase(new_nodes, i);
//                }
//            }
//        }

        }
        if(vertex->voxel.size == 1){
            vertex->type = Endpoint;
        }else{
            vertex->type = Junction;
        }
        ComplexGraph_addData2Node(graph, nodeID, vertex);
        for (int i = 0; i < vertex->voxel.size; ++i) {
            Voxel voxel = Datavector_at(vertex->voxel, i);
            Neighbors_Vector allNeighbors = find_all_neighbors(voxel);
            if(allNeighbors.size == 2){
                printf("Vertex Error\n");
            }
        }
    }else{
        printf("Edge Error");
    }

}

void visit_voxel(Voxel voxel) {
    int x = voxel.x;
    int y = voxel.y;
    int z = voxel.z;
    Pre_Node *stageNode = get_StageNode(x, y, z);
    stageNode->visited = true;
}

bool is_visited(Voxel node) {
    int x = node.x;
    int y = node.y;
    int z = node.z;
    Pre_Node *stageNode = get_StageNode(x, y, z);
    return stageNode->visited;
}

Voxel *get_next_edgeNode(Voxel node) {
    Neighbors_Vector newNeighbors = find_new_neighbors(node);
    if (newNeighbors.size == 1) {
        Voxel *voxel = &Datavector_at(newNeighbors, 0)->voxel;
        Neighbors_Vector allNeighbors = find_all_neighbors(*voxel);
        if (allNeighbors.size == 2) {
            return voxel;
        }
    }
    return NULL;
}


Pre_Node *follow_edge(ComplexGraph *graph, Edge *edge) {
    Voxel node = Datavector_at(edge->slabs, 1);
    Neighbors_Vector allNeighbors = find_all_neighbors(node);
    Voxel *previousNode;
    if (allNeighbors.size == 2) {
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
                return potentialVertex;
            }
        }
    } else{
        return get_StageNode(node.x, node.y, node.z);
    }
}
//Pre_Node *follow_edge(ComplexGraph *graph, Edge *edge) {
//    Voxel node = Datavector_at(edge->slabs, 1);
//    Neighbors_Vector edgeNodes;
//    Datavector_init(edgeNodes,10);
//    Neighbors_Vector neighbors = find_new_neighbors(node);
//    Neighbors_Vector allNeighbors = find_all_neighbors(node);
//    Pre_Node *neighbor = NULL;
//    if(allNeighbors.size == 2){
//        if (neighbors.size != 1) {
//            return NULL;
//        }
//    }else{
//        return get_StageNode(node.x, node.y, node.z);
//    }
//    visit_voxel(node);
//    while (neighbors.size == 1) {
//        neighbor = Datavector_at(neighbors, 0);
//        if (neighbor->visited == false) {
//            Datavector_pushBack(edge->slabs, neighbor->voxel);
//            Datavector_pushBack(edgeNodes,neighbor);
//            allNeighbors = find_all_neighbors(neighbor->voxel);
//            if (allNeighbors.size == 2) {
//                visit_voxel(neighbor->voxel);
//                neighbors = find_new_neighbors(neighbor->voxel);
//            }else{
//                return neighbor;
//            }
//        } else {
//            Datavector_erase(neighbors, 0);
//        }
//    }
//    if(edge->startNodeID == 581){
//        for (int i = 0; i < edge->slabs.size; ++i) {
//            Voxel voxel = Datavector_at(edge->slabs, i);
//            printf("Edge :%d,%d,%d\n",voxel.x,voxel.y,voxel.z);
//        }
//    }
//    if(allNeighbors.size == 2 && neighbors.size == 0){
//        for (int i = 0; i < allNeighbors.size; ++i) {
//            Pre_Node *potentialVertex = Datavector_at(allNeighbors, i);
//            if(!contains_neighbor(edgeNodes,potentialVertex->voxel)){
//                return potentialVertex;
//            }
//        }
//        return NULL;
//    }
//    return neighbor;
//}
//
bool contains_neighbor(Neighbors_Vector vector, Voxel voxel) {
    for (int i = 0; i < vector.size; ++i) {
        Pre_Node *tmp = Datavector_at(vector, i);
        if (tmp->voxel.x == voxel.x && tmp->voxel.y == voxel.y && tmp->voxel.z == voxel.z) {
            return true;
        }
    }
    return false;
}


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
    }
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

    return graph;
}

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


void print_voxel(Voxel voxel) {
    printf("%d,%d,%d \n", voxel.x, voxel.y, voxel.z);
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
    dat_skeleton = fopen(filename_skeleton, "r");
    if (dat_skeleton == NULL) {
        printf("File not found");
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

double vector_length(Vector3D vector) {
    return sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
}

double calculate_angle_x(Vector3D vector) {
    return fabs(asin(vector.x / vector_length(vector)) * 180 / M_PI);
}

double calculate_angle_y(Vector3D vector) {
    return fabs(asin(vector.y / vector_length(vector)) * 180 / M_PI);
}

double calculate_angle_z(Vector3D vector) {
    return fabs(asin(vector.z / vector_length(vector)) * 180 / M_PI);
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
        char *oldLocale = setlocale(LC_NUMERIC, NULL);
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
//        setlocale(LC_NUMERIC, oldLocale);
//        FILE *csv_length = fopen(strcat(file, "_length.csv"), "w+");
//        for (int i = 0; i < graph->edges.size; ++i) {
//            fprintf(csv_length, "%f\n", Datavector_at(graph->edges, i).length);
//        }

    }

}

int main(void) {
    int x_max;
    int y_max;
    int z_max;
    readFile(&x_max, &y_max, &z_max);
    Datavector_sort(pre_nodes, cmp_x);
    init_stage(x_max, y_max, z_max);
    for (int i = 0; i < pre_nodes.size; i++) {
        Voxel node = Datavector_at(pre_nodes, i).voxel;
        stage[node.x - 1][node.y - 1][node.z - 1] = &Datavector_at(pre_nodes, i);
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
//        printf("%lu", graph->edges->allelements);
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
    }
    Datavector_sort(graph_vector, cmp_graph);
    for (int i = 0; i < graph_vector.size; ++i) {
        printf("Graph %d: size = %zu\n", i, get_junctions(Datavector_at(graph_vector, i)).size);
    }
    create_csv(graph_vector, filename_output);
    return 0;
}


