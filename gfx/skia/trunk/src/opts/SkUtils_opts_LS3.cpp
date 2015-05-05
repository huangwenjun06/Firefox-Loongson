/*
 * Copyright 2014 Lemote Co,.ltd.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "SkUtils.h"

#ifdef _MIPS_ARCH_LOONGSON3A
static void sk_memset16_LS3(uint16_t *dst, uint16_t value, int count)
{
    SkASSERT(dst != NULL && count >= 0);

    // dst must be 2-byte aligned.
    SkASSERT((((size_t) dst) & 0x01) == 0);

    if (count >= 32) {
        while (((size_t)dst) & 0x0F) {
            *dst++ = value;
            --count;
        }
        void *d = reinterpret_cast<void*>(dst);
        uint64_t v = ((uint64_t) value) << 32 | value;
        while (count >= 32) {
            __asm__ volatile (
                ".set push \n\t"
                ".set noreorder \n\t"
                ".set arch=loongson3a \n\t"
                "gssq %[v], %[v], 0(%[dst]) \n\t"
                "gssq %[v], %[v], 16(%[dst]) \n\t"
                "gssq %[v], %[v], 32(%[dst]) \n\t"
                "gssq %[v], %[v], 48(%[dst]) \n\t"
                ".set pop \n\t"
                ::[dst]"r"(d), [v]"r"(v)
                :"memory"
            );
            d += 64;
            count -= 32;
        }
        dst = reinterpret_cast<uint16_t*>(d);
    }
    while (count > 0) {
        *dst++ = value;
        --count;
    }
}
#endif

SkMemset16Proc SkMemset16GetPlatformProc() {
#ifdef _MIPS_ARCH_LOONGSON3A
    return sk_memset16_LS3;
#else
    return NULL;
#endif
}

#ifdef _MIPS_ARCH_LOONGSON3A
static void sk_memset32_LS3(uint32_t *dst, uint32_t value, int count)
{
    SkASSERT(dst != NULL && count >= 0);

    // dst must be 4-byte aligned.
    SkASSERT((((size_t) dst) & 0x03) == 0);

    if (count >= 16) {
        while (((size_t)dst) & 0x0F) {
            *dst++ = value;
            --count;
        }
        void *d = reinterpret_cast<void*>(dst);
        uint64_t v = ((uint64_t) value) << 32 | value;
        while (count >= 16) {
            __asm__ volatile (
                ".set push \n\t"
                ".set noreorder \n\t"
                ".set arch=loongson3a \n\t"
                "gssq %[v], %[v], 0(%[dst]) \n\t"
                "gssq %[v], %[v], 16(%[dst]) \n\t"
                "gssq %[v], %[v], 32(%[dst]) \n\t"
                "gssq %[v], %[v], 48(%[dst]) \n\t"
                ".set pop \n\t"
                ::[dst]"r"(d), [v]"r"(v)
                :"memory"
            );
            d += 64;
            count -= 16;
        }
        dst = reinterpret_cast<uint32_t*>(d);
    }
    while (count > 0) {
        *dst++ = value;
        --count;
    }
}
#endif

SkMemset32Proc SkMemset32GetPlatformProc() {
#ifdef _MIPS_ARCH_LOONGSON3A
    return sk_memset32_LS3;
#else
    return NULL;
#endif
}

#ifdef _MIPS_ARCH_LOONGSON3A
static void sk_memcpy32_LS3(uint32_t *dst, const uint32_t *src, int count)
{
    if (count >= 16) {
        while (((size_t)dst) & 0x0F) {
            *dst++ = *src++;
            --count;
        }
        void *dst128 = reinterpret_cast<void*>(dst);
        const void *src128 = reinterpret_cast<const void*>(src);
        while (count >= 16) {
            uint64_t t0, t1, t2, t3;
            uint64_t t4, t5, t6, t7;

            __asm__ volatile (
                ".set push \n\t"
                ".set noreorder \n\t"
                ".set arch=loongson3a \n\t"
                "ldl %[t0], 0xf(%[src]) \n\t"
                "ldr %[t0], 0x8(%[src]) \n\t"
                "ldl %[t1], 0x7(%[src]) \n\t"
                "ldr %[t1], 0x0(%[src]) \n\t"
                "ldl %[t2], 16+0xf(%[src]) \n\t"
                "ldr %[t2], 16+0x8(%[src]) \n\t"
                "ldl %[t3], 16+0x7(%[src]) \n\t"
                "ldr %[t3], 16+0x0(%[src]) \n\t"
                "ldl %[t4], 32+0xf(%[src]) \n\t"
                "ldr %[t4], 32+0x8(%[src]) \n\t"
                "ldl %[t5], 32+0x7(%[src]) \n\t"
                "ldr %[t5], 32+0x0(%[src]) \n\t"
                "ldl %[t6], 48+0xf(%[src]) \n\t"
                "ldr %[t6], 48+0x8(%[src]) \n\t"
                "ldl %[t7], 48+0x7(%[src]) \n\t"
                "ldr %[t7], 48+0x0(%[src]) \n\t"
                "gssq %[t0], %[t1], 0(%[dst]) \n\t"
                "gssq %[t2], %[t3], 16(%[dst]) \n\t"
                "gssq %[t4], %[t5], 32(%[dst]) \n\t"
                "gssq %[t6], %[t7], 48(%[dst]) \n\t"
                ".set pop \n\t"
                :[t0]"=&r"(t0), [t1]"=&r"(t1),
                 [t2]"=&r"(t2), [t3]"=&r"(t3),
                 [t4]"=&r"(t4), [t5]"=&r"(t5),
                 [t6]"=&r"(t6), [t7]"=&r"(t7)
                :[dst]"r"(dst128), [src]"r"(src128)
                :"memory"
            );

            dst128 += 64;
            src128 += 64;
            count -= 16;
        }
        dst = reinterpret_cast<uint32_t*>(dst128);
        src = reinterpret_cast<const uint32_t*>(src128);
    }
    while (count > 0) {
        *dst++ = *src++;
        --count;
    }
}
#endif

SkMemcpy32Proc SkMemcpy32GetPlatformProc() {
#ifdef _MIPS_ARCH_LOONGSON3A
    return sk_memcpy32_LS3;
#else
    return NULL;
#endif
}
