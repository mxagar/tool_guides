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
#include <atomic>

// Complex data structure: Container
struct Container
{
    int a;
    float b;
    void print() {
        std::cout << "a = " << a << "; b = " << b << std::endl;
    };
};

// Atomic data can be safely accessed from different threads
// However, if complex operatiors and synchronization is required on them
// specific functions are defined for them: .load(), .fetch_add(), .store(), ...
std::atomic<int> g_number;
// Mutex for avoiding simultanoeus access of non-atomic data
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
            g_number += 5;
        }
        else
        {
            std::cout << "Thread 2" << std::endl;
            c->a -= 1;
            c->b *= 0.9;
            g_number -= 1;
        }
        c->print();
        std::cout << g_number << std::endl;
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
    if (t1.joinable())
        t1.join();
    if (t2.joinable())   
        t2.join();

    // Delete Container object from the heap
    delete c;
    c = nullptr;

    return (0);

}