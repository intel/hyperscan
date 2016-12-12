# tests for compiler properties

# set -Werror so we can't ignore unused attribute warnings
set (CMAKE_REQUIRED_FLAGS "-Werror")

CHECK_C_SOURCE_COMPILES("
    int foo(int) __attribute__ ((ifunc(\"foo_i\")));
    int f1(int i) { return i; }
    void (*foo_i()) { return f1; }
    int main(void) { return 0; }
    " HAS_C_ATTR_IFUNC)

unset(CMAKE_REQUIRED_FLAGS)
