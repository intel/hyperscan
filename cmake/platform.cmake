# determine the target arch

# really only interested in the preprocessor here
CHECK_C_SOURCE_COMPILES("#if !(defined(__x86_64__) || defined(_M_X64))\n#error not 64bit\n#endif\nint main(void) { return 0; }" ARCH_X86_64)

CHECK_C_SOURCE_COMPILES("#if !(defined(__i386__) || defined(_M_IX86))\n#error not 32bit\n#endif\nint main(void) { return 0; }" ARCH_IA32)

CHECK_C_SOURCE_COMPILES("#if !defined(__ARM_ARCH_ISA_A64)\n#error not 64bit\n#endif\nint main(void) { return 0; }" ARCH_AARCH64)
CHECK_C_SOURCE_COMPILES("#if !defined(__ARM_ARCH_ISA_ARM)\n#error not 32bit\n#endif\nint main(void) { return 0; }" ARCH_ARM32)

if (ARCH_X86_64 OR ARCH_AARCH64)
  set(ARCH_64_BIT TRUE)
else()
  set(ARCH_32_BIT TRUE)
endif()
