#include "concurrent/concurrentModule.h"
#include <future>

void ConcurrentModule::parallelFor(int num, std::function<void(int)> body) {
    const int workerNum = std::min(num, getInstance().maxWorkerThreads_);
    if (workerNum == 0) {
        // no threads, just do it and return
        for (int index = 0; index < num; ++index) {
            body(index);
        }
        return;
    }

    std::vector<std::future<void>> tasksFuture;
    tasksFuture.reserve(num);
    int index = 0;
    while (index < num) {
        const int start = index;
        for (int i = 0; i < workerNum; ++i) {
            tasksFuture.emplace_back(std::async(std::launch::async, body, index));
            ++index;
            if (index >= num) break;
        }
        for (int i = start; i < index; ++i) tasksFuture[i].get();
    }
}