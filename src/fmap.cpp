#include "fmap.h"

#include <gtest/gtest.h>

TEST(FmapOptional, Integer) {

    std::optional<int> a {1};
    auto b = fmap(a, [](auto x){ return x + 2; });

    static_assert(std::is_same_v<
        std::optional<int>,
        decltype(b)>
    );
    ASSERT_TRUE(b);
    ASSERT_EQ(3, *b);
}

TEST(FmapOptional, OptionalInteger) {

    std::optional<int> a {1};
    auto b = fmap(a, [](auto x){ return std::optional{x + 2}; });

    static_assert(std::is_same_v<
        std::optional<std::optional<int>>,
        decltype(b)>
    );
    ASSERT_TRUE(b);
    ASSERT_TRUE(*b);
    ASSERT_EQ(3, **b);
}
