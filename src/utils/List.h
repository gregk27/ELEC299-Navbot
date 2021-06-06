#ifndef LIST_H
#define LIST_H
#include <stdlib.h>

/**
 * Class which stores a add-only list of elements
 * Allows for easily building array
*/
template <typename T>
class List {
  private:
    T *elements = 0x0;
    int numElements = 0;

  public:
    List(){
      elements = 0x0;
      numElements = 0;
    }

    /**
     * Add an element to the list
    */
    void add(T elem){
      numElements ++;
      if(elements){
        elements = (T*) realloc(elements, sizeof(T)*numElements);
      } else {
        elements = (T*) malloc(sizeof(T));
      }
      elements[numElements-1] = elem;
    }
    
    /**
     * Remove the last element added
    */
    void pop(){
      if(numElements >= 0){
        numElements --;
      }
    }

    /**
     * Get the size of the list
    */
    int size() {
      return numElements;
    }

    /**
     * Get the element at index i
    */
    T operator [](int i){
      return elements[i];
    }

};

#endif