#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstddef>
using std::size_t;
#endif

template <typename T>
class LinkedList {
public:
    struct ListNode {
        T value;
        ListNode *prev;
        ListNode *next;
    };

private:
    ListNode *first;
    ListNode *last;
    
    ListNode *reference;
    
public:
    LinkedList()
    {
        this->first = NULL;
        this->last = NULL;
        
        this->reference = NULL;
    }
    
    ~LinkedList()
    {
        ListNode *ln = this->last;
        while (ln != NULL) {
            ListNode *tmp = ln;
            ln = ln->prev;
            delete tmp;
        }
    }
    
    LinkedList<T> *clone() {
        LinkedList<T> *c = new LinkedList<T>();
        c->append(*this);
        return c;
    }
    
    void add(T& value)
    {
        ListNode *newNode = new ListNode(/*value*/);
        newNode->value = value;
        newNode->next = NULL;
        
        if (this->first == NULL) {
            newNode->prev = NULL;
            this->first = newNode;
            this->last = newNode;
        }
        else {
            newNode->prev = this->last;
            this->last->next = newNode;
            this->last = newNode;
        }
    }
    
    void append(LinkedList<T>& list)
    {
        for (ListNode *ln = list.getFirst(); ln != NULL; ln = ln->next) {
            this->add(ln->value);
        }
    }
    
    T removeLast()
    {
        ListNode *last = this->last;
        this->last = last->prev;
        T value = last->value;
        delete last;
        return value;
    }
    
    bool contains(T& value)
    {
        for (ListNode *ln = this->first; ln != NULL; ln = ln->next) {
            if (ln->value == value) {
                return true;
            }
        }
        return false;
    }
    
    bool empty()
    {
        return this->first == NULL;
    }
    
    size_t size()
    {
        size_t count = 0;
        for (ListNode *ln = this->first; ln != NULL; ln = ln->next) {
            count++;
        }
        return count;
    }
    
    size_t size_from_ref()
    {
        size_t count = 0;
        for (ListNode *ln = this->reference; ln != NULL; ln = ln->next) {
            count++;
        }
        return count;
    }
    
    ListNode *getFirst() { return this->first; }
    ListNode *getLast() { return this->last; }
    ListNode *getRef() { return this->reference; }
    void setRef(ListNode *ref) { this->reference = ref; }
};

#endif
