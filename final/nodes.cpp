#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdint>
#include <cstddef>
#endif
#include "Node.h"

Node *start_node_1;
Node *start_node_2;
Node *start_node_3;
Node *end_node;

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
    nodes[9]->edges = new Node *[3] { nodes[4], nodes[8], nodes[14] };
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
}

bool reachable_node(Node *& start, Node *& end, LinkedList<Node *> *path)
{
    return reachable_id(start, end->id, path);
}

bool reachable_id(Node *& start, uint8_t end_node, LinkedList<Node *> *path)
{
    path->add(start);
    if (start->id == end_node) {
        return true;
    }
    
    LinkedList<LinkedList<Node *> *> *branches = new LinkedList<LinkedList<Node *> *>();
    for (uint16_t i = 0; i < start->edge_count; i++) {
        LinkedList<Node *> *branch = path->clone();
        branch->setRef(branch->getLast());
        
        Node *next = start->edges[i];
        
        if (!path->contains(next) && reachable_id(next, end_node, branch)) {
            branches->add(branch);
        }
        else {
            delete branch;
        }
    }
    
    if (branches->empty()) {
        delete branches;
        path->removeLast();
        return false;
    }
    
    LinkedList<Node *> *shortest_branch = branches->getFirst()->value;
    size_t shortest_length = shortest_branch->size_from_ref();
    
    for (LinkedList<LinkedList<Node *> *>::ListNode *ln = branches->getFirst()->next; ln != NULL; ln = ln->next) {
        size_t current_length = ln->value->size_from_ref();
        if (current_length < shortest_length) {
            delete shortest_branch;
            
            shortest_branch = ln->value;
            shortest_length = current_length;
        }
        else {
            delete ln->value;
        }
    }
    
    delete branches;
    
    for (LinkedList<Node *>::ListNode *ln = shortest_branch->getRef()->next; ln != NULL; ln = ln->next) {
        path->add(ln->value);
    }
    delete shortest_branch;
    return true;
}
