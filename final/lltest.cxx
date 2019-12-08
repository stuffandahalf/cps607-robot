#include <iostream>
#include "LinkedList.h"

int main(int argc, char **argv)
{
	LinkedList<int> *ll = new LinkedList<int>();
    for (int i = 0; i < 8; i++) {
        int *n = new int();
        *n = i;
        ll->add(n);
    }
    
    /*for (LinkedList<int>::ListNode *ln = ll->getFirst(); ln != 0; ln = ln->next) {
        std::cout << *(ln->value) << std::endl;
        delete ln->value;
    }*/
    
    /*for (LinkedList<int>::ListNode *ln = ll->getLast(); ln != 0; ln = ln->prev) {
        std::cout << *(ln->value) << std::endl;
        delete ln->value;
    }*/
    
    //int n = 3;
    //std::cout << ll->contains(&n) << std::endl;
    
    for (LinkedList<int>::ListNode *ln = ll->getLast(); ln != 0; ln = ln->prev) {
        delete ln->value;
    }
    
    delete ll;
    
	return 0;
}

