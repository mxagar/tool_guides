/*
 * Producer-Consumer patterns used in sender()-receiver() scenarios
 * We have two threads which share resources
 * Mutexes are used to avoid racing issues
 * and conditions variables are used to wake up the reciprocal thread
 * Source: https://levelup.gitconnected.com/producer-consumer-problem-using-condition-variable-in-c-6c4d96efcbbc
 */

#include <condition_variable> // std::condition_variale
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>         // std::chrono::seconds
using namespace std;

std::mutex g_mutex;
std::condition_variable g_cv;

bool g_ready = false;
int g_data = 0;

int produceData() {
  int randomNumber = rand() % 1000;
  //std::this_thread::sleep_for (std::chrono::seconds(1));
  std::cout << "produce data: " << randomNumber << "\n";
  return randomNumber;
}

void consumeData(int data) { 
  //std::this_thread::sleep_for (std::chrono::milliseconds(100));
  std::cout << "receive data: " << data << "\n";
}

void consumer() {
  int data = 0;
  while (true) {
    std::unique_lock<std::mutex> ul(g_mutex);
    g_cv.wait(ul, []() { return g_ready; });
    consumeData(g_data);
    g_ready = false;
    ul.unlock();
    // WARNING: lock-unlock in a while loop might block the mutex...
    g_cv.notify_one();
  }
}

void producer() {
  while (true) {
    std::unique_lock<std::mutex> ul(g_mutex);
    g_data = produceData();
    g_ready = true;
    ul.unlock();
    g_cv.notify_one();
    ul.lock();
    g_cv.wait(ul, []() { return g_ready == false; });
  }
}

void consumerThread(int n) { consumer(); }

void producerThread(int n) { producer(); }

int main() {
  int times = 100;
  std::thread t1(consumerThread, times);
  std::thread t2(producerThread, times);
  t1.join();
  t2.join();
  return 0;
}