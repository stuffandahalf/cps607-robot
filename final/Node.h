#ifndef NODE_H
#define NODE_H

#include "LinkedList.h"

#define START_NODE_1    (1)
#define START_NODE_2    (5)
#define START_NODE_3    (11)
#define END_NODE        (15)
#define NODE_COUNT      (15)

struct Node {
    uint8_t id;
    uint8_t edge_count;
    Node **edges;
};

extern Node *start_node_1;
extern Node *start_node_2;
extern Node *start_node_3;
extern Node *end_node;

void init_nodes();
bool reachable(Node *start, Node *end, LinkedList<Node> *path);
bool reachable(Node *start, uint8_t id, LinkedList<Node> *path);

#endif
