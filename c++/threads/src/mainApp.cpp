/**
*\file mainApp.cpp
*\author Mikel Sagardia
*\date 2018-2021
* This simple example tests concurrency functionalities with C++: Threads
*/

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>

// Complex data structure: Container
struct Container
{
    int a;
    float b;
    void print() {
        std::cout << "a = " << a << "; b = " << b << std::endl;
    };
};

std::mutex g_mutex;

void threadFunction(Container* c, int num, bool flag)
{
    for (int i = 0; i < num; ++i) {
        // We can either
        // - lock the mutex while using the shared data and then unlockit again, or
        // - use the loc_guard, which locks and unlocks automatically, so we don't forget (better practice)
        std::lock_guard<std::mutex> guard(g_mutex);
        //g_mutex.lock();
        if (flag)
        {
            std::cout << "Thread 1" << std::endl;
            c->a += 5;
            c->b *= 2.0;
        }
        else
        {
            std::cout << "Thread 2" << std::endl;
            c->a -= 1;
            c->b *= 0.9;
        }
        c->print();
        // We can put a thread to sleep this ways (although not necessary here)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //g_mutex.unlock();
    }
}

int main(int argc, char** argv) {

	std::cout << "Simple Threads Example" << std::endl;

    // We create a shared Container object on the heap
    // That is our shared data
    Container* c = new Container;
    c->a = 1;
    c->b = 1.0f;

    // We launch 2 threads that share the same data Container c
    // but both use the same function
    std::thread t1(threadFunction, c, 1000, true);
    std::thread t2(threadFunction, c, 1000, false);

    // Join all threads
    t1.join();
    t2.join();

    // Delete Container object from the heap
    delete c;
    c = nullptr;

    return (0);

}