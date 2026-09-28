#include <expected>
#include <string>

int main() {
  static_assert(__cplusplus >= 202302L);
  std::expected<int, std::string> value = 42;
  return value.value() == 42 ? 0 : 1;
}
