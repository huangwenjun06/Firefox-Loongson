/*
 * Copyright 2015 Loongson Co,.ltd.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "SkBlitMask.h"
#include "SkColorPriv.h"

#ifdef _MIPS_ARCH_LOONGSON3A

#include "MMIHelpers.h"

typedef struct { double l; double h; } __m128i;

static void
SkARGB32_A8_BlitMask_LS3(void* device, size_t dstRB, const void* maskPtr,
                               size_t maskRB, SkColor origColor,
                               int width, int height) {
    SkPMColor color = SkPreMultiplyColor(origColor);
    size_t dstOffset = dstRB - (width << 2);
    size_t maskOffset = maskRB - width;
    SkPMColor* dst = (SkPMColor *)device;
    const uint8_t* mask = (const uint8_t*)maskPtr;
    __m128i rb_mask, c_256, c_1;
    double zero, i8, if5;
    uint64_t tmp;

    asm volatile (
        ".set push \n\t"
        ".set arch=loongson3a \n\t"
        "xor %[zero], %[zero], %[zero] \n\t"
        "li %[tmp], 8 \n\t"
        "mtc1 %[tmp], %[i8] \n\t"
        "li %[tmp], 0xf5 \n\t"
        "mtc1 %[tmp], %[if5] \n\t"
        "li %[tmp], 0x00ff00ff \n\t"
        "mtc1 %[tmp], %[rbml] \n\t"
        "punpcklwd %[rbml], %[rbml], %[rbml] \n\t"
        "mov.d %[rbmh], %[rbml] \n\t"
        "li %[tmp], 256 \n\t"
        "mtc1 %[tmp], %[c256l] \n\t"
        "pshufh %[c256l], %[c256l], %[zero] \n\t"
        "mov.d %[c256h], %[c256l] \n\t"
        "li %[tmp], 1 \n\t"
        "mtc1 %[tmp], %[c1l] \n\t"
        "pshufh %[c1l], %[c1l], %[zero] \n\t"
        "mov.d %[c1h], %[c1l] \n\t"
        ".set pop \n\t"
        :[zero]"=f"(zero), [i8]"=f"(i8), [if5]"=f"(if5),
         [rbmh]"=f"(rb_mask.h), [rbml]"=f"(rb_mask.l),
         [c256h]"=f"(c_256.h), [c256l]"=f"(c_256.l),
         [c1h]"=f"(c_1.h), [c1l]"=f"(c_1.l),
         [tmp]"=&r"(tmp)
    );
    do {
        int count = width;
        if (count >= 4) {
            while (((size_t)dst & 0x0F) != 0 && (count > 0)) {
                *dst = SkBlendARGB32(color, *dst, *mask);
                mask++;
                dst++;
                count--;
            }

            __m128i src_pixel, *d = reinterpret_cast<__m128i*>(dst);
            asm volatile (
                ".set push \n\t"
                ".set arch=loongson3a \n\t"
                "mtc1 %[c], %[spl] \n\t"
                "punpcklwd %[spl], %[spl], %[spl] \n\t"
                "mov.d %[sph], %[spl] \n\t"
                ".set pop \n\t"
                :[sph]"=f"(src_pixel.h), [spl]"=f"(src_pixel.l)
                :[c]"r"(color)
            );
            while (count >= 4) {
                uint32_t m;
                __m128i dst_pixel, src_scale_wide;
                __m128i dst_rb, src_rb, dst_ag, src_ag, dst_alpha;

                asm volatile (
                    ".set push \n\t"
                    ".set arch=loongson3a \n\t"
                    // Load 4 pixels each of src and dest.
                    "gslqc1 %[dph], %[dpl], (%[d]) \n\t"
                    //set the aphla value
                    "ulw %[m], (%[mask]) \n\t"
                    "mtc1 %[m], %[sswh] \n\t"
                    "punpcklbh %[sswh], %[sswh], %[zero] \n\t"
                    "punpcklhw %[sswl], %[sswh], %[sswh] \n\t"
                    "punpckhhw %[sswh], %[sswh], %[sswh] \n\t"
                    //call SkAlpha255To256()
                    _mm_paddh(ssw, ssw, c1)
                    // Get red and blue pixels into lower byte of each word.
                    _mm_and(drb, rbm, dp)
                    _mm_and(srb, rbm, sp)
                    // Get alpha and green into lower byte of each word.
                    _mm_psrlh(dag, dp, i8)
                    _mm_psrlh(sag, sp, i8)
                    // Put per-pixel alpha in low byte of each word.
                    "pshufh %[dalh], %[sagh], %[if5]\n\t"
                    "mov.d %[dall], %[sagl] \n\t"
                    // "mov.d %[dalh], %[dalh] \n\t"
                    "pshufh %[dall], %[dall], %[if5] \n\t"
                    // dst_alpha = dst_alpha * src_scale
                    _mm_pmullh(dal, dal, ssw)
                    // Divide by 256.
                    _mm_psrlh(dal, dal, i8)
                    // Subtract alphas from 256, to get 1..256
                    _mm_psubh(dal, c256, dal)
                    // Multiply red and blue by dst pixel alpha.
                    _mm_pmullh(drb, drb, dal)
                    // Multiply alpha and green by dst pixel alpha.
                    _mm_pmullh(dag, dag, dal)
                    // Multiply red and blue by global alpha.
                    _mm_pmullh(srb, srb, ssw)
                    // Multiply alpha and green by global alpha.
                    _mm_pmullh(sag, sag, ssw)
                    // Divide by 256.
                    _mm_psrlh(drb, drb, i8)
                    _mm_psrlh(srb, srb, i8)
                    // Mask out low bits (goodies already in the right place; no need to divide)
                    _mm_pandn(dag, rbm, dag)
                    _mm_pandn(sag, rbm, sag)
                    // Combine back into RGBA.
                    _mm_or(dp, drb, dag)
                    _mm_or(drb, srb, sag)
                    // Add two pixels into result (overwrite drb).
                    _mm_paddb(drb, drb, dp)
                    "gssqc1 %[drbh], %[drbl], (%[d]) \n\t"
                    ".set pop \n\t"
                    :[dph]"=&f"(dst_pixel.h), [dpl]"=&f"(dst_pixel.l),
                     [sswh]"=&f"(src_scale_wide.h), [sswl]"=&f"(src_scale_wide.l),
                     [drbh]"=&f"(dst_rb.h), [drbl]"=&f"(dst_rb.l),
                     [srbh]"=&f"(src_rb.h), [srbl]"=&f"(src_rb.l),
                     [dagh]"=&f"(dst_ag.h), [dagl]"=&f"(dst_ag.l),
                     [sagh]"=&f"(src_ag.h), [sagl]"=&f"(src_ag.l),
                     [dalh]"=&f"(dst_alpha.h), [dall]"=&f"(dst_alpha.l),
                     [m]"=&r"(m)
                    :[d]"r"(d), [mask]"r"(mask), [zero]"f"(zero),
                     [rbmh]"f"(rb_mask.h), [rbml]"f"(rb_mask.l),
                     [sph]"f"(src_pixel.h), [spl]"f"(src_pixel.l),
                     [c256h]"f"(c_256.h), [c256l]"f"(c_256.l),
                     [c1h]"f"(c_1.h), [c1l]"f"(c_1.l),
                     [i8]"f"(i8), [if5]"f"(if5)
                );

                // load the next 4 pixel
                mask = mask + 4;
                d++;
                count -= 4;
            }
            dst = reinterpret_cast<SkPMColor *>(d);
        }
        while (count > 0) {
            *dst= SkBlendARGB32(color, *dst, *mask);
            dst += 1;
            mask++;
            count --;
        }
        dst = (SkPMColor *)((char*)dst + dstOffset);
        mask += maskOffset;
    } while (--height != 0);
}

#endif /* _MIPS_ARCH_LOONGSON3A */

SkBlitMask::ColorProc SkBlitMask::PlatformColorProcs(SkColorType dstCT,
                                                     SkMask::Format maskFormat,
                                                     SkColor color) {
    if (SkMask::kA8_Format != maskFormat) {
        return NULL;
    }

    ColorProc proc = NULL;
    switch (dstCT) {
#ifdef _MIPS_ARCH_LOONGSON3A
        case kN32_SkColorType:
            // The LS3 version is not (yet) faster for black, so we check
            // for that.
            if (SK_ColorBLACK != color) {
                proc = SkARGB32_A8_BlitMask_LS3;
            }
            break;
#endif
        default:
            break;
    }
    return proc;
}

SkBlitMask::BlitLCD16RowProc SkBlitMask::PlatformBlitRowProcs16(bool isOpaque) {
    return NULL;
}

SkBlitMask::RowProc SkBlitMask::PlatformRowProcs(SkColorType dstCT,
                                                 SkMask::Format maskFormat,
                                                 RowFlags flags) {
    return NULL;
}
