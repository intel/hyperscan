# determine the target arch

# really only interested in the preprocessor here
CHECK_C_SOURCE_COMPILES("#if !(defined(__x86_64__) || defined(_M_X64))\n#error not 64bit\n#endif\nint main(void) { return 0; }" ARCH_64_BIT)

CHECK_C_SOURCE_COMPILES("#if !(defined(__i386__) || defined(_M_IX86))\n#error not 64bit\n#endif\nint main(void) { return 0; }" ARCH_32_BIT)

set(ARCH_X86_64 ${ARCH_64_BIT})
set(ARCH_IA32 ${ARCH_32_BIT})
