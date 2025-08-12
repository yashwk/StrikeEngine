#include "strikeengine/core/JobSystem.hpp"
#include <iostream>

namespace StrikeEngine {

    JobSystem::JobSystem(size_t num_threads) : _pending_jobs(0), _stop_processing(false) {
        size_t thread_count = (num_threads == 0) ? std::thread::hardware_concurrency() : num_threads;

        if (thread_count == 0) {
            thread_count = 1;
        }

        std::cout << "JobSystem: Initializing with " << thread_count << " worker threads." << std::endl;

        for (size_t i = 0; i < thread_count; ++i) {
            _worker_threads.emplace_back(&JobSystem::workerLoop, this);
        }
    }

    JobSystem::~JobSystem() {
        {
            std::unique_lock<std::mutex> lock(_queue_mutex);
            _stop_processing = true;
        }
        _condition.notify_all();
    }

    void JobSystem::submit(std::function<void()> job) {
        {
            std::unique_lock<std::mutex> lock(_queue_mutex);
            _job_queue.push(std::move(job));
            _pending_jobs++;
        }
        // Notify one waiting worker thread that a new job is available.
        _condition.notify_one();
    }

    void JobSystem::wait() {
        // This function blocks until the number of jobs in flight becomes zero.
        std::unique_lock<std::mutex> lock(_queue_mutex);
        // The condition variable waits until the lambda returns true.
        _condition.wait(lock, [this] { return _pending_jobs == 0; });
    }

    void JobSystem::workerLoop() {
        while (true) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(_queue_mutex);
                // Wait until the queue is not empty or the system is stopping.
                _condition.wait(lock, [this] {
                    return !_job_queue.empty() || _stop_processing;
                });

                // If we are stopping and the queue is empty, the thread can exit.
                if (_stop_processing && _job_queue.empty()) {
                    return;
                }

                // Pop a job from the queue.
                job = std::move(_job_queue.front());
                _job_queue.pop();
            }

            // Execute the job.
            if (job) {
                job();
            }

            // Decrement the job counter and notify the main thread if all jobs are done.
            {
                std::unique_lock<std::mutex> lock(_queue_mutex);
                _pending_jobs--;
            }
            _condition.notify_all(); // Use notify_all to be safe, especially for the wait() function.
        }
    }

} // namespace StrikeEngine
