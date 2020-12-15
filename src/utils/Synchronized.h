#pragma once
#include "ofMain.h"

template <class T>
class Synchronized : public ofMutex {
private:
    T back, middle, front;
    bool newData;
public:
    Synchronized()
    :newData(false) {
    }
    void setup(const T& prototype) {
        back = prototype;
        middle = prototype;
        front = prototype;
    }
    T& getBack() {
        return back;
    }
    T& getFront() {
        return front;
    }
    void swapBack() {
        lock();
        swap(back, middle);
        newData = true;
        unlock();
    }
    bool swapFront() {
        lock();
        bool curNewData = newData;
        if(newData) {
            swap(front, middle);
            newData = false;
        }
        unlock();
        return curNewData;
    }
};