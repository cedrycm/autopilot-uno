/*
SinlgyLinkedList.h - This is an implementation of 
a singly linked list to be used in the arduino zip sim solution
*/

#ifndef SinglyLinkedList_h
#define SinglyLinkedList_h

#include <stddef.h>

typedef void (*functiontype)();

template <class T>
struct Node
{
  T data;
  Node<T> *next;
};

template <class T>
class SinglyLinkedList
{
protected:
  int _size;
  functiontype _func;
  Node<T> *head;
  Node<T> *tail;

public:
  SinglyLinkedList();                 //initiate empty
  SinglyLinkedList(int size_n, T _t); //initiate with default
  ~SinglyLinkedList();

  virtual bool add(T);
  /*Add an item to the tail of the list. */

  virtual T pop();
  /*Remove an item from the head of the list and return it */

  virtual int size();
  /* Return the total number of elements in the list */

  virtual void attach_func();
};

// Initialize LinkedList with false values
template <typename T>
SinglyLinkedList<T>::SinglyLinkedList()
{
  head = NULL;
  tail = NULL;
  _size = 0;
}

template <typename T>
SinglyLinkedList<T>::~SinglyLinkedList()
{
  ListNode<T> *tmp;
  while (head != NULL)
  {
    tmp = head;
    root = root->next;
    delete tmp;
  }
}

template <typename T>
bool SinglyLinkedList<T>::add(T _t)
{
  Node<T> *newNode = new Node();
  newNode->data = _t;
  newNode->next = NULL;

  if (head)
  {
    //if elements already exist in list
    tail->next = newNode;
    tail = newNode;
  }
  else
  {
    head = newNode;
    tail = newNode;
  }

  _size++;
}

template <typename T>
T SinglyLinkedList<T>::pop()
{
  if (_size <= 0)
  {
    return T();
  }

  if (_size >= 2)
  {
    Node<T> *tmp = head->next;
    T return_data = head->data;

    delete (tmp->next);
    tmp->next = NULL;

    head = tmp;
    _size--;

    return return_data;
  }
  else
  {
    T return_data = head->data;
    delete (head);
    head = NULL;
    tail = NULL;
    _size = 0;
    return return_data;
  }
}

template <typename T>
int SinglyLinkedList<T>::size()
{
  return _size;
}

template <typename T>
void SinglyLinkedList<T>::attach_func()
#endif