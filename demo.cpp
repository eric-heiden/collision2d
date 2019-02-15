#include "sat.hpp"

#include <iostream>

int main(int argc, char **argv) {
  using namespace collision2d;

  const Polygon<double> a{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Polygon<double> b{{1, 1}, {3, 0}, {3, 5}, {1, 6}};
  const Polygon<double> c{{0, 3}, {2, 3}, {2, 6}, {0, 7}};

  std::cout << "intersect(a, b) ? " << std::boolalpha << intersect(a, b)
            << std::endl;
    std::cout << "intersect(b, c) ? " << std::boolalpha << intersect(b, c)
              << std::endl;
    std::cout << "intersect(a, c) ? " << std::boolalpha << intersect(a, c)
              << std::endl;

  return EXIT_SUCCESS;
}