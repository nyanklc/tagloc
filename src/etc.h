#ifndef __TAGLOC_ETC_H_
#define __TAGLOC_ETC_H_

#include <memory>
#include <utility>

namespace tagloc {
template <typename T, typename... Args>
inline std::unique_ptr<T> make_unique(Args&&... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}  // namespace tagloc

#endif