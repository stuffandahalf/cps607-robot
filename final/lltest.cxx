#include <iostream>
#include "Node.h"
#include "LinkedList.h"

int main(int argc, char **argv)
{
	/*LinkedList<int> *ll = new LinkedList<int>();
    for (int i = 0; i < 8; i++) {
        ll->add(i);
    }
    
    for (LinkedList<int>::ListNode *ln = ll->getFirst(); ln != 0; ln = ln->next) {
        std::cout << ln->value << std::endl;
    }*/
    
    /*for (LinkedList<int>::ListNode *ln = ll->getLast(); ln != 0; ln = ln->prev) {
        std::cout << *(ln->value) << std::endl;
        delete ln->value;
    }*/
    
    //int n = 3;
    //std::cout << ll->contains(&n) << std::endl;
    
    /*for (LinkedList<int>::ListNode *ln = ll->getLast(); ln != 0; ln = ln->prev) {
        delete ln->value;
    }*/
    
    //delete ll;
    
    init_nodes();
    
    LinkedList<Node *> *path = new LinkedList<Node *>();
    
    //reachable_node(start_node_1, end_node, path);
    reachable_node(start_node_1, start_node_1->edges[0]->edges[1], path);
    for (LinkedList<Node *>::ListNode *ln = path->getFirst(); ln != 0; ln = ln->next) {
        std::cout << (int)(ln->value->id) << std::endl;
    }
    
    delete path;
    delete_nodes();
    
	return 0;
}

