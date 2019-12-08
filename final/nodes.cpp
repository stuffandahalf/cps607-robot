#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#include <cstddef>
#include <iostream>
#endif
#include "Node.h"

Node *start_node_1;
Node *start_node_2;
Node *start_node_3;
Node *end_node;
Node *left_intersection;
Node *right_intersection;

static Node *nodes[NODE_COUNT];

void init_nodes()
{
    //Node *nodes[NODE_COUNT];
    for (uint8_t i = 0; i < NODE_COUNT; i++) {
        nodes[i] = new Node();
        nodes[i]->id = i + 1;
    }
    
    start_node_1 = nodes[0];    // 1st node is start
    start_node_2 = nodes[10];   // 11th node is start
    start_node_3 = nodes[4];    // 5th node is start
    end_node = nodes[14];       // last node is end
    
    right_intersection = nodes[5];
    left_intersection = nodes[9];
    
    
    uint8_t simple_nodes[] = { 1, 2, 3, 
                               6, 7, 8,
                               11, 12, 13 };
    uint8_t simple_node_count = sizeof(simple_nodes) / sizeof(uint8_t);
    for (uint8_t i = 0; i < simple_node_count; i++) {
        uint8_t node_index = simple_nodes[i];
        
        nodes[node_index]->edge_count = 2;
        nodes[node_index]->edges = new Edge[2] {
            { nodes[node_index - 1], DIRECTION_EAST },
            { nodes[node_index + 1], DIRECTION_WEST }
        };
    }
    
    
    start_node_1->edge_count = 2;
    start_node_1->edges = new Edge[2] {
        { nodes[1], DIRECTION_WEST },
        { nodes[5], DIRECTION_NORTH }
    };
    
    start_node_2->edge_count = 2;
    start_node_2->edges = new Edge[2] {
        { nodes[5], DIRECTION_SOUTH },
        { nodes[11], DIRECTION_WEST }
    };
    
    start_node_3->edge_count = 2;
    start_node_3->edges = new Edge[2] {
        { nodes[3], DIRECTION_EAST },
        { nodes[9], DIRECTION_NORTH }
    };
    
    end_node->edge_count = 2;
    end_node->edges = new Edge[2] {
        { nodes[9], DIRECTION_SOUTH },
        { nodes[13], DIRECTION_EAST }
    };
    
    right_intersection->edge_count = 3;
    right_intersection->edges = new Edge[3] {
        { nodes[0], DIRECTION_SOUTH },
        { nodes[6], DIRECTION_WEST },
        { nodes[10], DIRECTION_NORTH }
    };
    
    left_intersection->edge_count = 3;
    left_intersection->edges = new Edge[3] {
        { nodes[4], DIRECTION_SOUTH },
        { nodes[8], DIRECTION_EAST },
        { nodes[14], DIRECTION_NORTH }
    };
}

void delete_nodes()
{
    for (uint8_t i = 0; i < NODE_COUNT; i++) {
        delete[] nodes[i]->edges;
        delete nodes[i];
        nodes[i] = 0;
    }
    start_node_1 = 0;
    start_node_2 = 0;
    start_node_3 = 0;
    end_node = 0;
    left_intersection = 0;
    right_intersection = 0;
}

bool reachable_node(Node *& start, Node *& end, Path *path)
{
    return reachable_id(start, end->id, path);
}

bool reachable_id(Node *& start, uint8_t end_node, Path *path)
{
    if (start->id == end_node) {
        return true;
    }
    
    LinkedList<Path *> *branches = new LinkedList<Path *>();
    for (uint16_t i = 0; i < start->edge_count; i++) {
        Path *branch = path->clone();
        
        Edge *next = &start->edges[i];
        branch->edges.add(next);
        branch->edges.setRef(branch->edges.getLast());
        
        if (path->node_traversed(next->node)) {
            delete branch;
            continue;
        }
        
        if (reachable_id(next->node, end_node, branch)) {
            branches->add(branch);
        }
        else {
            delete branch;
        }
    }
    
    if (branches->empty()) {
        delete branches;
        return false;
    }
    
    Path *shortest_path = branches->getFirst()->value;
    size_t shortest_size = shortest_path->edges.size_from_ref();
    for (LinkedList<Path *>::ListNode *ln = branches->getFirst()->next; ln != NULL; ln = ln->next) {
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
    
    delete branches;
    
    for (LinkedList<Edge *>::ListNode *ln = shortest_path->edges.getRef(); ln != NULL; ln = ln->next) {
        path->edges.add(ln->value);
    }
    delete shortest_path;
    return true;
}
