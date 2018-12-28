// CRAP: Because of conflict between gtest and boost.test
// I have to implement `main` myself. I'll deal with it later.

#include <gtest/gtest.h>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
