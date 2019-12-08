#include <iostream>
#include "Node.h"
#include "LinkedList.h"

int main(int argc, char **argv)
{
    init_nodes();
    
    Path *path = new Path(start_node_1);
    
    reachable_node(start_node_1, end_node, path);
    std::cout << "\t" << (int)path->start->id << std::endl;
    for (LinkedList<Edge *>::ListNode *ln = path->edges.getFirst(); ln != 0; ln = ln->next) {
        std::cout << ln->value->direction << "\t" << (int)(ln->value->node->id) << std::endl;
    }
    
    delete path;
    delete_nodes();
    
	return 0;
}

