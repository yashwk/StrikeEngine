#pragma once

#include <vector>
#include <functional>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace StrikeEngine {

    class JobSystem {
    public:
        /**
         * @brief Constructs the JobSystem and initializes the worker threads.
         * @param num_threads The number of worker threads to create. Defaults to the
         * hardware concurrency if not specified.
         */
        explicit JobSystem(size_t num_threads = 0);

        /**
         * @brief Destructor. Waits for all threads to finish and joins them.
         */
        ~JobSystem();

        /**
         * @brief Submits a new job to the queue.
         * @param job A function object (e.g., a lambda) to be executed.
         */
        void submit(std::function<void()> job);

        /**
         * @brief Blocks the calling thread until all submitted jobs are complete.
         */
        void wait();

    private:
        /**
         * @brief The main loop for each worker thread.
         */
        void workerLoop();

        std::vector<std::jthread> _worker_threads;
        std::queue<std::function<void()>> _job_queue;
        std::mutex _queue_mutex;
        std::condition_variable _condition;
        std::atomic<size_t> _pending_jobs;
        bool _stop_processing = false;
    };

} // namespace StrikeEngine
