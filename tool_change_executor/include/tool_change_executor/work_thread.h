// Copyright 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file tool_change_executor/work_thread.h
 * \brief Generic thread for concurrent execution of long-running tasks
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-05-07
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_EXECUTOR_WORK_THREAD_H_INCLUDED
#define TOOL_CHANGE_EXECUTOR_WORK_THREAD_H_INCLUDED

#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <rclcpp/logger.hpp>
#include <thread>

namespace tool_change_executor {

template <typename T>
class WorkThread
{
public:
  using Task = std::function<void(const T&, std::stop_token)>;

  WorkThread(Task task, rclcpp::Logger log);
  ~WorkThread();

  WorkThread(const WorkThread&)            = delete;
  WorkThread& operator=(const WorkThread&) = delete;

  [[nodiscard]] bool running() const;
  [[nodiscard]] bool work(const T& item);

private:
  void run(std::stop_token stop);

  rclcpp::Logger m_log;
  Task m_task;

  std::optional<T> m_item;
  mutable std::mutex m_item_mut;
  std::condition_variable m_item_cv;

  std::jthread m_thread;
};

} // namespace tool_change_executor


namespace tool_change_executor {

template <typename T>
WorkThread<T>::WorkThread(Task task, rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_task{std::move(task)}
  , m_thread{std::bind(&WorkThread<T>::run, this, std::placeholders::_1)}
{
}

template <typename T>
WorkThread<T>::~WorkThread()
{
  m_thread.request_stop();
  m_item_cv.notify_all();
}

template <typename T>
bool WorkThread<T>::running() const
{
  std::lock_guard lock{m_item_mut};
  return m_item;
}

template <typename T>
bool WorkThread<T>::work(const T& item)
{
  {
    std::lock_guard lock{m_item_mut};
    if (m_item)
    {
      return false;
    }

    m_item = item;
  }

  m_item_cv.notify_all();
  return true;
}

template <typename T>
void WorkThread<T>::run(std::stop_token stop)
{
  while (!stop.stop_requested())
  {
    const auto new_item = [&]() -> std::optional<T> {
      // Wait until we either got a new item or should stop
      std::unique_lock lock{m_item_mut};
      if (m_item)
      {
        return *m_item;
      }

      m_item_cv.wait(lock, [&] { return m_item.has_value() || stop.stop_requested(); });
      if (stop.stop_requested())
      {
        return std::nullopt;
      }

      return *m_item;
    }();

    if (new_item)
    {
      m_task(*new_item, stop);
    }

    {
      std::lock_guard lock{m_item_mut};
      m_item = std::nullopt;
    }
  }
}

} // namespace tool_change_executor

#endif // TOOL_CHANGE_EXECUTOR_WORK_THREAD_H_INCLUDED
