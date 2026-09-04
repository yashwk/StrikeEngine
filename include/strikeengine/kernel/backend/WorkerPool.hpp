#pragma once

#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <cstddef>
#include <algorithm>

namespace StrikeEngine::Kernel {

    /**
     * @brief Lightweight persistent worker thread pool for parallel SoA entity execution.
     *
     * Features:
     *  - Zero-allocation steady state task execution.
     *  - Calling thread assists in executing work chunks (no idle worker starvation).
     *  - When threadCount <= 1, parallelFor executes directly on the caller with zero overhead.
     */
    class WorkerPool {
    public:
        WorkerPool() = default;

        explicit WorkerPool(std::size_t numThreads) {
            resize(numThreads);
        }

        ~WorkerPool() {
            stop();
        }

        WorkerPool(const WorkerPool&) = delete;
        WorkerPool& operator=(const WorkerPool&) = delete;

        void resize(std::size_t numThreads) {
            stop();
            if (numThreads <= 1) {
                activeWorkers = 1;
                return;
            }
            running = true;
            activeWorkers = numThreads;
            // The pool spawns (numThreads - 1) background workers; the calling
            // thread itself participates as worker 0 during parallelFor.
            const std::size_t backgroundThreads = numThreads - 1;
            workers.reserve(backgroundThreads);
            for (std::size_t i = 0; i < backgroundThreads; ++i) {
                workers.emplace_back([this]() {
                    workerLoop();
                });
            }
        }

        [[nodiscard]] std::size_t size() const {
            return activeWorkers > 0 ? activeWorkers : 1;
        }

        template <typename Func>
        void parallelFor(std::size_t totalCount, const Func& func, std::size_t minChunk = 4) {
            if (totalCount == 0) return;

            if (activeWorkers <= 1 || totalCount < minChunk * 2 || workers.empty()) {
                func(0, totalCount);
                return;
            }

            const std::size_t chunks = std::min(activeWorkers, (totalCount + minChunk - 1) / minChunk);
            if (chunks <= 1) {
                func(0, totalCount);
                return;
            }

            const std::size_t baseChunkSize = totalCount / chunks;
            const std::size_t remainder = totalCount % chunks;

            // Chunk 0 is executed on the calling thread.
            const std::size_t firstChunkSize = baseChunkSize + (remainder > 0 ? 1 : 0);
            std::size_t currentIdx = firstChunkSize;

            const std::size_t bgTasks = chunks - 1;
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                remainingTasks = bgTasks;
                for (std::size_t c = 1; c < chunks; ++c) {
                    const std::size_t count = baseChunkSize + (c < remainder ? 1 : 0);
                    const std::size_t start = currentIdx;
                    const std::size_t end = start + count;
                    currentIdx = end;

                    tasks.push([&func, start, end]() {
                        func(start, end);
                    });
                }
                cv.notify_all();
            }

            // Calling thread executes chunk 0 directly
            func(0, firstChunkSize);

            // Wait for background tasks to complete
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                doneCv.wait(lock, [this]() {
                    return remainingTasks == 0;
                });
            }
        }

        void stop() {
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                running = false;
                while (!tasks.empty()) tasks.pop();
                remainingTasks = 0;
                cv.notify_all();
            }
            for (auto& w : workers) {
                if (w.joinable()) {
                    w.join();
                }
            }
            workers.clear();
            activeWorkers = 1;
        }

    private:
        void workerLoop() {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    cv.wait(lock, [this]() {
                        return !running || !tasks.empty();
                    });
                    if (!running && tasks.empty()) {
                        return;
                    }
                    task = std::move(tasks.front());
                    tasks.pop();
                }

                task();

                {
                    std::unique_lock<std::mutex> lock(queueMutex);
                    --remainingTasks;
                    if (remainingTasks == 0) {
                        doneCv.notify_one();
                    }
                }
            }
        }

        std::vector<std::thread> workers;
        std::queue<std::function<void()>> tasks;
        std::mutex queueMutex;
        std::condition_variable cv;
        std::condition_variable doneCv;
        std::size_t remainingTasks = 0;
        std::size_t activeWorkers = 1;
        bool running = false;
    };

} // namespace StrikeEngine::Kernel
