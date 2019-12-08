#ifndef LINKEDLIST_H
#define LINKEDLIST_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstddef>
#endif

template <typename T>
class LinkedList {
public:
    struct ListNode {
        T *value;
        ListNode *prev;
        ListNode *next;
    };

private:
    ListNode *first;
    ListNode *last;
    
public:
    LinkedList()
    {
        this->first = NULL;
        this->last = NULL;
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
    
    void add(T *value)
    {
        ListNode *newNode = new ListNode();
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
    
    bool contains(T *value)
    {
        for (ListNode *ln = this->first; ln != NULL; ln = ln->next) {
            if (ln->value == value || *(ln->value) == *value) {
                return true;
            }
        }
        return false;
    }
    
    size_t size()
    {
        size_t count = 0;
        for (ListNode *ln = this->first; ln != NULL; ln = ln->next) {
            count++;
        }
        return count;
    }
    
    ListNode *getFirst() { return this->first; }
    ListNode *getLast() { return this->last; }
};

#endif
