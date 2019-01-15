#include "fmap.h"
#include <memory>
#include <gtest/gtest.h>

TEST(Fmap, OptionalInteger) {

    std::optional<int> a {1};
    auto b = fmap(a, [](auto x){ return x + 2; });

    static_assert(std::is_same_v<
        std::optional<int>,
        decltype(b)>
    );
    ASSERT_TRUE(b);
    ASSERT_EQ(3, *b);
}

TEST(Fmap, OptionalOptionalInteger) {

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

TEST(Fmap, OptionalMoveOnlyType) {

    auto a = std::make_optional(std::make_unique<int>(2));
    auto b = fmap(a, [](auto && p){ return std::make_unique<double>(*p * 0.25); });

    static_assert(std::is_same_v<
        std::optional<std::unique_ptr<double>>,
        decltype(b)>
    );
    ASSERT_TRUE(b);
    ASSERT_EQ(0.5, **b);
}
