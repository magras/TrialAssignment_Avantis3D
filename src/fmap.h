#ifndef FMAP_H
#define FMAP_H

#include <optional>
#include <type_traits>

template <typename T, typename F>
std::optional<std::invoke_result_t<F, T const&>> fmap(std::optional<T> const& o, F f) {
    if (o)
        return f(*o);
    return std::nullopt;
}

#endif // FMAP_H
