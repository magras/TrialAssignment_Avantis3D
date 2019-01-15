#ifndef FMAP_H
#define FMAP_H

#include <optional>
#include <type_traits>

template <typename T> struct is_optional                   : std::false_type {};
template <typename T> struct is_optional<std::optional<T>> : std::true_type  {};
template <typename T> inline constexpr bool is_optional_v = is_optional<T>::value;

// TODO: remove when P0550R2 will be implemented
template <typename T> using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

template <
    typename O,
    typename F,
    typename = std::enable_if_t<is_optional_v<remove_cvref_t<O>>>
>
constexpr auto fmap(O && o, F f) noexcept -> std::optional<std::invoke_result_t<F, typename std::remove_reference_t<O>::value_type>>
{
    if (o.has_value())
        return f(*std::forward<O>(o));
    return std::nullopt;
}

#endif // FMAP_H
