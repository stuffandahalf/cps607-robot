#include <Arduino.h>
#include "Node.h"

Node *start_node_1;
Node *start_node_2;
Node *start_node_3;
Node *end_node;

void init_nodes()
{
    Node *nodes[NODE_COUNT];
    for (uint8_t i = 0; i < NODE_COUNT; i++) {
        nodes[i] = new Node();
        nodes[i]->id = i + 1;
    }
    
    start_node_1 = nodes[0];    // 1st node is start
    start_node_2 = nodes[10];   // 11th node is start
    start_node_3 = nodes[4];    // 5th node is start
    end_node = nodes[14];       // last node is end
    
    
    uint8_t simple_nodes[] = { 1, 2, 3, 
                               6, 7, 8,
                               11, 12, 13 };
    uint8_t simple_node_count = sizeof(simple_nodes) / sizeof(uint8_t);
    for (uint8_t i = 0; i < simple_node_count; i++) {
        uint8_t node_index = simple_nodes[i];
        
        nodes[node_index]->edge_count = 2;
        nodes[node_index]->edges = new Node *[2] { nodes[node_index - 1], nodes[node_index + 1] };
    }
    
    
    start_node_1->edge_count = 2;
    start_node_1->edges = new Node *[2] { nodes[1], nodes[5] };
    
    start_node_2->edge_count = 2;
    start_node_2->edges = new Node *[2] { nodes[5], nodes[11] };
    
    start_node_3->edge_count = 2;
    start_node_3->edges = new Node *[2] { nodes[3], nodes[9] };
    
    end_node->edge_count = 2;
    end_node->edges = new Node *[2] { nodes[9], nodes[13] };
    
    nodes[5]->edge_count = 3;
    nodes[5]->edges = new Node *[3] { nodes[0], nodes[6], nodes[10] };
    
    nodes[9]->edge_count = 3;
    nodes[9]->edges = new Node *[3] { nodes[5], nodes[8], nodes[14] };
}

//bool reachable(Node *start, 
