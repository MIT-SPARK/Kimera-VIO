/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testThreadsafeQueue.cpp
 * @brief  test ThreadsafeQueue
 * @author Antoni Rosinol
 */

#include <chrono>
#include <string>
#include <iostream>
#include <thread>

#include "utils/ThreadsafeQueue.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

void consumer(ThreadsafeQueue<std::string>& q,
              const std::atomic_bool& kill_switch) {
  while (!kill_switch) {
    std::string msg = "No msg!";
    if (q.popBlocking(msg)) {
      std::cout << "Got msg: " << msg << '\n';
    }
  }
  q.shutdown();
}

void producer(ThreadsafeQueue<std::string>& q,
              const std::atomic_bool& kill_switch) {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    q.push("Hello World!");
  }
  q.shutdown();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_reference) {
  ThreadsafeQueue<std::string> q ("1");
  std::thread p ([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::string s;
  q.popBlocking(s);
  EXPECT(s == "Hello World!");
  q.popBlocking(s);
  EXPECT(s == "Hello World 2!");
  q.shutdown();
  EXPECT(q.popBlocking(s) == false);
  EXPECT(s == "Hello World 2!");

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_shared_ptr) {
  ThreadsafeQueue<std::string> q("1");
  std::thread p ([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT(*s == "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT(*s2 == "Hello World 2!");
  q.shutdown();
  EXPECT(q.popBlocking() == nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, push) {
  ThreadsafeQueue<std::string> q("1");
  std::thread p ([&] {
    q.push(std::string("Hello World!"));
    std::string s = "Hello World 2!";
    q.push(s);
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT(*s == "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT(*s2 == "Hello World 2!");
  q.shutdown();
  EXPECT(q.popBlocking() == nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, producer_consumer) {
  ThreadsafeQueue<std::string> q("1");
  std::atomic_bool kill_switch (false);
  std::thread c (consumer, std::ref(q), std::ref(kill_switch));
  std::thread p (producer, std::ref(q), std::ref(kill_switch));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Joining threads.\n";
  kill_switch = true;
  c.join();
  p.join();
  std::cout << "Threads joined.\n";
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, stress_test) {
  ThreadsafeQueue<std::string> q("1");
  std::atomic_bool kill_switch (false);
  std::vector<std::thread> cs;
  for (size_t i = 0; i < 10; i++){
      // Create 10 consumers.
      cs.push_back(std::thread(consumer, std::ref(q), std::ref(kill_switch)));
  }
  std::vector<std::thread> ps;
  for (size_t i = 0; i < 10; i++){
      // Create 10 producers.
      ps.push_back(std::thread(producer, std::ref(q), std::ref(kill_switch)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Joining threads.\n";
  kill_switch = true;
  for (size_t i = 0; i < cs.size(); i++) {
    cs[i].join();
  }
  for (size_t i = 0; i < ps.size(); i++) {
    ps[i].join();
  }
  std::cout << "Threads joined.\n";
}


/* ************************************************************************* */
int main() {
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
