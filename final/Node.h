#ifndef NODE_H
#define NODE_H

#include "LinkedList.h"

#define START_NODE_1    (1)
#define START_NODE_2    (5)
#define START_NODE_3    (11)
#define END_NODE        (15)
#define NODE_COUNT      (15)

enum Direction {
    DIRECTION_INVALID,
    DIRECTION_NORTH,
    DIRECTION_EAST,
    DIRECTION_SOUTH,
    DIRECTION_WEST
};

struct Edge;

struct Node {
    uint8_t id;
    uint8_t edge_count;
    Edge *edges;
};

struct Edge {
    Node *node;
    Direction direction;
};

class Path {
public:
    Node *start;
    LinkedList<Edge *> edges;
    
    Path(Node *start) : edges()
    {
        this->start = start;
    }
    
    Path *clone()
    {
        Path *newPath = new Path(this->start);
        newPath->edges.append(this->edges);
        
        return newPath;
    }
    
    bool node_traversed(Node *node)
    {
        if (node == this->start) {
            return true;
        }
        
        for (LinkedList<Edge *>::ListNode *ln = this->edges.getFirst(); ln != NULL; ln = ln->next) {
            if (ln->value->node == node) {
                return true;
            }
        }
        
        return false;
    }
};

extern Node *start_node_1;
extern Node *start_node_2;
extern Node *start_node_3;
extern Node *end_node;
extern Node *left_intersection;
extern Node *right_intersection;

void init_nodes();
void delete_nodes();
bool reachable_node(Node *& start, Node *& end, Path *path);
bool reachable_id(Node *& start, uint8_t id, Path *path);

#endif
