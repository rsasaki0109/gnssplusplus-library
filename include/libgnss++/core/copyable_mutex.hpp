#pragma once

#include <mutex>

namespace libgnss {

/**
 * @brief A std::mutex whose owner can be copied: a copy gets a fresh,
 * unlocked mutex of its own (gnss ppp --combined copies a loaded processor
 * for its backward pass).
 */
class CopyableMutex : public std::mutex {
public:
    CopyableMutex() = default;
    CopyableMutex(const CopyableMutex&) : std::mutex() {}
    CopyableMutex& operator=(const CopyableMutex&) { return *this; }
};

}  // namespace libgnss
