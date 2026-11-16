#include <array>
#include <iostream>
#include <range/v3/view/span.hpp>

struct Identifier { int x; };
std::array<Identifier, 2> ids;

ranges::span<const Identifier> get_span() {
    return ids;
}

int main() {
    std::cout << get_span().size() << std::endl;
}
