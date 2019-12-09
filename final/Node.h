#ifndef NODE_H
#define NODE_H

#include "LinkedList.h"

#define START_NODE_1    (1)
#define START_NODE_2    (5)
#define START_NODE_3    (11)
#define END_NODE        (15)
#define NODE_COUNT      (15)

#define MAX_EDGE_COUNT  (4)

enum Direction {
    DIRECTION_INVALID,
    DIRECTION_NORTH,
    DIRECTION_EAST,
    DIRECTION_SOUTH,
    DIRECTION_WEST
};

struct Node;

struct Edge {
    Node *node;
    Direction direction;
};

struct Node {
    uint8_t id;
    uint8_t edge_count;
    Edge edges[MAX_EDGE_COUNT];
};

class Path {
public:
    Node *start;
    LinkedList<Edge *> edges;
    
    Path(Node *start) : edges()
    {
        this->start = start;
    }
    
    Path(Path& other) : edges(other.edges)
    {
        this->start = other.start;
    }
    
    bool node_traversed(const Node *node)
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
bool reachable_node(Node * start, Node * end, Path *path);
bool reachable_id(Node * start, uint8_t id, Path *path);


int direction_delta(Direction a, Direction b);

#endif
