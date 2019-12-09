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

void init_nodes()
{
    //Node nodes[NODE_COUNT];
    //for (uint8_t i = 0; i < NODE_COUNT; i++) {
        ////nodes[i] = new Node();
        //nodes[i].id = i + 1;
    //}
    
    //start_node_1 = &nodes[0];    // 1st node is start
    //start_node_2 = &nodes[10];   // 11th node is start
    //start_node_3 = &nodes[4];    // 5th node is start
    //end_node = &nodes[14];       // last node is end
    
    //right_intersection = &nodes[5];
    //left_intersection = &nodes[9];
    
    
    //uint8_t simple_nodes[] = { 1, 2, 3, 
                               //6, 7, 8,
                               //11, 12, 13 };
    //uint8_t simple_node_count = sizeof(simple_nodes) / sizeof(uint8_t);
    //for (uint8_t i = 0; i < simple_node_count; i++) {
        //uint8_t node_index = simple_nodes[i];
        
        //nodes[node_index].edge_count = 2;
        //nodes[node_index].edges[0] = { &nodes[node_index - 1], DIRECTION_EAST };
        //nodes[node_index].edges[1] = { &nodes[node_index + 1], DIRECTION_WEST };
        ////nodes[node_index]->edges = /*new Edge[2]*/ {
            ////{ nodes[node_index - 1], DIRECTION_EAST },
            ////{ nodes[node_index + 1], DIRECTION_WEST }
        ////};
    //}
    
    
    //start_node_1->edge_count = 2;
    //start_node_1->edges[0] = { &nodes[1], DIRECTION_WEST };
    //start_node_1->edges[1] = { &nodes[5], DIRECTION_NORTH };
    ///*start_node_1->edges = new Edge[2] {
        //{ nodes[1], DIRECTION_WEST },
        //{ nodes[5], DIRECTION_NORTH }
    //};*/
    
    
    //start_node_2->edge_count = 2;
    //start_node_2->edges[0] = { &nodes[5], DIRECTION_SOUTH };
    //start_node_2->edges[1] = { &nodes[11], DIRECTION_WEST };
    ///*start_node_2->edges = new Edge[2] {
        //{ nodes[5], DIRECTION_SOUTH },
        //{ nodes[11], DIRECTION_WEST }
    //};*/
    
    //start_node_3->edge_count = 2;
    //start_node_3->edges[0] = { &nodes[3], DIRECTION_EAST };
    //start_node_3->edges[1] = { &nodes[9], DIRECTION_NORTH };
    ///*start_node_3->edges = new Edge[2] {
        //{ nodes[3], DIRECTION_EAST },
        //{ nodes[9], DIRECTION_NORTH }
    //};*/
    
    //end_node->edge_count = 2;
    //end_node->edges[0] = { &nodes[9], DIRECTION_SOUTH };
    //end_node->edges[1] = { &nodes[13], DIRECTION_EAST };
    ///*end_node->edges = new Edge[2] {
        //{ nodes[9], DIRECTION_SOUTH },
        //{ nodes[13], DIRECTION_EAST }
    //};*/
    
    //right_intersection->edge_count = 3;
    //right_intersection->edges[0] = { &nodes[0], DIRECTION_SOUTH };
    //right_intersection->edges[1] = { &nodes[6], DIRECTION_WEST };
    //right_intersection->edges[2] = { &nodes[10], DIRECTION_NORTH };
    ///*right_intersection->edges = new Edge[3] {
        //{ nodes[0], DIRECTION_SOUTH },
        //{ nodes[6], DIRECTION_WEST },
        //{ nodes[10], DIRECTION_NORTH }
    //};*/
    
    //left_intersection->edge_count = 3;
    //left_intersection->edges[0] = { &nodes[4], DIRECTION_SOUTH };
    //left_intersection->edges[1] = { &nodes[8], DIRECTION_EAST };
    //left_intersection->edges[2] = { &nodes[14], DIRECTION_NORTH };
    ///*left_intersection->edges = new Edge[3] {
        //{ nodes[4], DIRECTION_SOUTH },
        //{ nodes[8], DIRECTION_EAST },
        //{ nodes[14], DIRECTION_NORTH }
    //};*/
}

void delete_nodes()
{
    /*for (uint8_t i = 0; i < NODE_COUNT; i++) {
        //delete[] nodes[i]->edges;
        delete nodes[i];
        nodes[i] = 0;
    }*/
    /*start_node_1 = 0;
    start_node_2 = 0;
    start_node_3 = 0;
    end_node = 0;
    left_intersection = 0;
    right_intersection = 0;*/
}

bool reachable_node(Node *start, Node *end, Path *path)
{
    Serial.println(start->id);
    Serial.println(start_node_1->id);
    
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
