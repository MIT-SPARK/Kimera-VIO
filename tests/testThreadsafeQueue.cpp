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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/utils/ThreadsafeQueue.h"

namespace VIO {

void consumer(ThreadsafeQueue<std::string>& q,  // NOLINT
              const std::atomic_bool& kill_switch) {
  while (!kill_switch) {
    std::string msg = "No msg!";
    if (q.popBlocking(msg)) {
      VLOG(1) << "Got msg: " << msg << '\n';
    }
  }
  q.shutdown();
}

void producer(ThreadsafeQueue<std::string>& q,  // NOLINT
              const std::atomic_bool& kill_switch) {
  while (!kill_switch) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    q.push("Hello World!");
  }
  q.shutdown();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_reference) {
  ThreadsafeQueue<std::string> q("test_queue");
  std::thread p([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::string s;
  q.popBlocking(s);
  EXPECT_EQ(s, "Hello World!");
  q.popBlocking(s);
  EXPECT_EQ(s, "Hello World 2!");
  q.shutdown();
  EXPECT_FALSE(q.popBlocking(s));
  EXPECT_EQ(s, "Hello World 2!");

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, popBlocking_by_shared_ptr) {
  ThreadsafeQueue<std::string> q("test_queue");
  std::thread p([&] {
    q.push("Hello World!");
    q.push("Hello World 2!");
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT_EQ(*s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(*s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, push) {
  ThreadsafeQueue<std::string> q("test_queue");
  std::thread p([&] {
    q.push(std::string("Hello World!"));
    std::string s = "Hello World 2!";
    q.push(s);
  });
  std::shared_ptr<std::string> s = q.popBlocking();
  EXPECT_EQ(*s, "Hello World!");
  auto s2 = q.popBlocking();
  EXPECT_EQ(*s2, "Hello World 2!");
  q.shutdown();
  EXPECT_EQ(q.popBlocking(), nullptr);

  // Leave some time for p to finish its work.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(p.joinable());
  p.join();
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, producer_consumer) {
  ThreadsafeQueue<std::string> q("test_queue");
  std::atomic_bool kill_switch(false);
  std::thread c(consumer, std::ref(q), std::ref(kill_switch));
  std::thread p(producer, std::ref(q), std::ref(kill_switch));

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Joining threads.\n";
  kill_switch = true;
  c.join();
  p.join();
  VLOG(1) << "Threads joined.\n";
}

/* ************************************************************************* */
TEST(testThreadsafeQueue, stress_test) {
  ThreadsafeQueue<std::string> q("test_queue");
  std::atomic_bool kill_switch(false);
  std::vector<std::thread> cs;
  for (size_t i = 0; i < 10; i++) {
    // Create 10 consumers.
    cs.push_back(std::thread(consumer, std::ref(q), std::ref(kill_switch)));
  }
  std::vector<std::thread> ps;
  for (size_t i = 0; i < 10; i++) {
    // Create 10 producers.
    ps.push_back(std::thread(producer, std::ref(q), std::ref(kill_switch)));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Shutdown queue.\n";
  q.shutdown();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Resume queue.\n";
  q.resume();

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  VLOG(1) << "Joining threads.\n";
  kill_switch = true;
  for (size_t i = 0; i < cs.size(); i++) {
    cs[i].join();
  }
  for (size_t i = 0; i < ps.size(); i++) {
    ps[i].join();
  }
  VLOG(1) << "Threads joined.\n";
}

}  // namespace VIO
