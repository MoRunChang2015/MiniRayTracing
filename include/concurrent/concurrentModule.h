#pragma once
#include <functional>

/**
 * @brief Concurrent Module
 *
 */
class ConcurrentModule {
    ConcurrentModule() = default;
    virtual ~ConcurrentModule() = default;
    ConcurrentModule(const ConcurrentModule&) = default;
    ConcurrentModule& operator=(const ConcurrentModule&) = default;

   public:
    /**
     * @brief Get the singleton object
     *
     * @return ConcurrentModule reference
     */
    static ConcurrentModule& getInstance() {
        static ConcurrentModule instance;
        return instance;
    }

    /**
     * @brief Get the number of workers
     *
     * @return uint32, the number of workers
     */
    int getNumWorkerThreads() { return maxWorkerThreads_; }

    /**
     * @brief Set the number of workers
     *
     * @param num , the number of workers
     */
    void setNumWorkerThreads(int num) { maxWorkerThreads_ = num; }

    /**
     * @brief Commit parallerl task
     *
     * @param num, the number of tasks
     * @param body, task function
     */
    static void parallelFor(int num, std::function<void(int)> body);

   private:
    int maxWorkerThreads_{4};
};