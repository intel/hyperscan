# detect architecture features
#
# must be called after determining where compiler intrinsics are defined

if (HAVE_C_X86INTRIN_H)
    set (INTRIN_INC_H "x86intrin.h")
elseif (HAVE_C_INTRIN_H)
    set (INTRIN_INC_H "intrin.h")
else ()
    message (FATAL_ERROR "No intrinsics header found")
endif ()


set (CMAKE_REQUIRED_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
# ensure we have the minimum of SSSE3 - call a SSSE3 intrinsic
CHECK_C_SOURCE_COMPILES("#include <${INTRIN_INC_H}>
int main() {
    __m128i a = _mm_set1_epi8(1);
    (void)_mm_shuffle_epi8(a, a);
}" HAVE_SSSE3)

if (NOT HAVE_SSSE3)
    message(FATAL_ERROR "A minimum of SSSE3 compiler support is required")
endif ()

# now look for AVX2
CHECK_C_SOURCE_COMPILES("#include <${INTRIN_INC_H}>
#if !defined(__AVX2__)
#error no avx2
#endif

int main(){
    __m256i z = _mm256_setzero_si256();
    (void)_mm256_xor_si256(z, z);
}" HAVE_AVX2)

if (NOT HAVE_AVX2)
    message(STATUS "Building without AVX2 support")
endif ()

unset (CMAKE_REQUIRED_FLAGS)
unset (INTRIN_INC_H)
