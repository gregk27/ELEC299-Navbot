#ifndef LIST_H
#define LIST_H
#include <stdlib.h>
#include <Arduino.h>

/**
 * Class which stores a fixed-size add-only list of elements
 * NOTE: If array is overrun, this function will be halted
*/
template <typename T>
class List {
  private:
    T *elements = 0x0;
    int numElements = 0;
    int capacity;

  public:
    List(int capacity){
      this->capacity = capacity;
      elements = (T*)malloc(sizeof(T)*capacity);
      numElements = 0;
    }

    /**
     * Add an element to the list
     * NOTE: If array is overrun, this function will hang
    */
    void add(T elem){
      numElements ++;
      if(numElements > capacity){
        Serial.print("Array capacity overflow: ");
        Serial.println(numElements);
        while(1){};
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
     * Reverse list contents
    */
    void reverse(){
      T temp;
      int len = size()-1;
      for(int i=0; i<size()/2; i++){
        if(i != len-i){
          temp = elements[i];
          elements[i] = elements[len-i];
          elements[len-i] = temp;
        }
      }
    }

    /**
     * Get the size of the list
    */
    int size() {
      return numElements;
    }

    /**
     * Get the list's capcity
    */
    int getCapacity(){
      return capacity;
    }

    /**
     * Get the element at index i
    */
    T operator [](int i){
      return elements[i];
    }

};

#endif