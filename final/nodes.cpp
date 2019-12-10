#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#include <cstddef>
#include <iostream>
#endif
#include "Node.h"

static Node nodes[NODE_COUNT] = {
    { 1, 2, { { &nodes[1], DIRECTION_WEST }, { &nodes[5], DIRECTION_NORTH } } },    // start 1
    { 2, 2, { { &nodes[2], DIRECTION_WEST }, { &nodes[0], DIRECTION_EAST } } },
    { 3, 2, { { &nodes[3], DIRECTION_WEST }, { &nodes[1], DIRECTION_EAST } } },
    { 4, 2, { { &nodes[4], DIRECTION_WEST }, { &nodes[2], DIRECTION_EAST } } },
    { 5, 2, { { &nodes[3], DIRECTION_EAST }, { &nodes[9], DIRECTION_NORTH } } },    // start 3
    { 6, 3, { { &nodes[0], DIRECTION_SOUTH }, { &nodes[6], DIRECTION_WEST }, { &nodes[10], DIRECTION_NORTH } } },   // right intersection
    { 7, 2, { { &nodes[7], DIRECTION_WEST }, { &nodes[5], DIRECTION_EAST } } },
    { 8, 2, { { &nodes[8], DIRECTION_WEST }, { &nodes[6], DIRECTION_EAST } } },
    { 9, 2, { { &nodes[9], DIRECTION_WEST }, { &nodes[7], DIRECTION_EAST } } },
    { 10, 3, { { &nodes[4], DIRECTION_SOUTH }, { &nodes[8], DIRECTION_EAST }, { &nodes[14], DIRECTION_NORTH } } },  // left intersection
    { 11, 2, { { &nodes[5], DIRECTION_SOUTH }, { &nodes[11], DIRECTION_WEST } } },  // start 2
    { 12, 2, { { &nodes[12], DIRECTION_WEST }, { &nodes[10], DIRECTION_EAST } } },
    { 13, 2, { { &nodes[13], DIRECTION_WEST }, { &nodes[11], DIRECTION_EAST } } },
    { 14, 2, { { &nodes[14], DIRECTION_WEST }, { &nodes[12], DIRECTION_EAST } } },
    { 15, 2, { { &nodes[9], DIRECTION_SOUTH }, { &nodes[13], DIRECTION_EAST } } }   // end
};

Node *start_node_1 = &nodes[0];
Node *start_node_2 = &nodes[10];
Node *start_node_3 = &nodes[4];
Node *end_node = &nodes[14];
Node *left_intersection = &nodes[9];
Node *right_intersection = &nodes[5];

bool reachable_node(Node *start, Node *end, Path *path)
{
    return reachable_id(start, end->id, path);
}

bool reachable_id(Node *start, uint8_t end_node, Path *path)
{
    if (start->id == end_node) {
        return true;
    }
    
    LinkedList<Path *> branches;
    for (uint16_t i = 0; i < start->edge_count; i++) {
        Edge *next = &start->edges[i];
        if (path->node_traversed(next->node)) {
            continue;
        }
        
        Path *branch = new Path(*path);
        branch->edges.add(next);
        branch->edges.setRef(branch->edges.getLast());
        
        if (reachable_id(next->node, end_node, branch)) {
            branches.add(branch);
        }
        else {
            delete branch;
        }
    }
    
    if (branches.empty()) {
        return false;
    }
    
    Path *shortest_path = branches.getFirst()->value;
    size_t shortest_size = shortest_path->edges.size_from_ref();
    for (LinkedList<Path *>::ListNode *ln = branches.getFirst()->next; ln != NULL; ln = ln->next) {
        size_t current_size = ln->value->edges.size_from_ref();
        if (current_size < shortest_size) {
            delete shortest_path;
            
            shortest_size = current_size;
            shortest_path = ln->value;
        }
        else {
            delete ln->value;
        }
    }
    
    //delete branches;
    
    for (LinkedList<Edge *>::ListNode *ln = shortest_path->edges.getRef(); ln != NULL; ln = ln->next) {
        path->edges.add(ln->value);
    }
    delete shortest_path;
    return true;
}

int direction_delta(Direction a, Direction b)
{
    int delta = a - b;
    if (delta < 0) {
        delta += 4;
    }
    return delta;
}
