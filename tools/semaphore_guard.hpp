#include <semaphore>
namespace dts_vision::tools {

class SemaphoreGuard {
public:
    explicit SemaphoreGuard(std::counting_semaphore<>& sem, bool acquired):
        sem_(sem),
        acquired_(acquired) {}
    ~SemaphoreGuard() {
        if (acquired_) {
            sem_.release();
        }
    }

private:
    std::counting_semaphore<>& sem_;
    bool acquired_;
};
} // namespace dts_vision::tools
