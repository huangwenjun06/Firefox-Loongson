/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/mips64/MacroAssembler-mips64.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/MathAlgorithms.h"

#include "jit/Bailouts.h"
#include "jit/BaselineFrame.h"
#include "jit/BaselineRegisters.h"
#include "jit/JitFrames.h"
#include "jit/MacroAssembler.h"
#include "jit/mips64/Simulator-mips64.h"
#include "jit/MoveEmitter.h"

using namespace js;
using namespace jit;

using mozilla::Abs;

static_assert(sizeof(intptr_t) == 8, "Not 32-bit clean.");

void
MacroAssemblerMIPS64::convertBoolToInt32(Register src, Register dest)
{
    // Note that C++ bool is only 1 byte, so zero extend it to clear the
    // higher-order bits.
    ma_and(dest, src, Imm32(0xff));
}

void
MacroAssemblerMIPS64::convertInt32ToDouble(Register src, FloatRegister dest)
{
    as_mtc1(src, dest);
    as_cvtdw(dest, dest);
}

void
MacroAssemblerMIPS64::convertInt32ToDouble(const Address& src, FloatRegister dest)
{
    ma_load(ScratchRegister, src, SizeWord);
    as_mtc1(ScratchRegister, dest);
    as_cvtdw(dest, dest);
}

void
MacroAssemblerMIPS64::convertUInt32ToDouble(Register src, FloatRegister dest)
{
    // We use SecondScratchDoubleReg because MacroAssembler::loadFromTypedArray
    // calls with ScratchDoubleReg as dest.
    MOZ_ASSERT(dest != SecondScratchDoubleReg);

    // Subtract INT32_MIN to get a positive number
    ma_subu(ScratchRegister, src, Imm32(INT32_MIN));

    // Convert value
    as_mtc1(ScratchRegister, dest);
    as_cvtdw(dest, dest);

    // Add unsigned value of INT32_MIN
    ma_lid(SecondScratchDoubleReg, 2147483648.0);
    as_addd(dest, dest, SecondScratchDoubleReg);
}

void
MacroAssemblerMIPS64::convertUInt32ToFloat32(Register src, FloatRegister dest)
{
    Label positive, done;
    ma_b(src, src, &positive, NotSigned, ShortJump);

    // We cannot do the same as convertUInt32ToDouble because float32 doesn't
    // have enough precision.
    convertUInt32ToDouble(src, dest);
    convertDoubleToFloat32(dest, dest);
    ma_b(&done);

    bind(&positive);
    convertInt32ToFloat32(src, dest);

    bind(&done);
}

void
MacroAssemblerMIPS64::convertDoubleToFloat32(FloatRegister src, FloatRegister dest)
{
    as_cvtsd(dest, src);
}

// Convert the floating point value to an integer, if it did not fit, then it
// was clamped to INT32_MIN/INT32_MAX, and we can test it.
// NOTE: if the value really was supposed to be INT32_MAX / INT32_MIN then it
// will be wrong.
void
MacroAssemblerMIPS64::branchTruncateDouble(FloatRegister src, Register dest,
                                         Label* fail)
{
    Label test, success;
    as_truncwd(ScratchDoubleReg, src);
    as_mfc1(dest, ScratchDoubleReg);

    ma_b(dest, Imm32(INT32_MAX), fail, Assembler::Equal);
    ma_b(dest, Imm32(INT32_MIN), fail, Assembler::Equal);
}

// Checks whether a double is representable as a 32-bit integer. If so, the
// integer is written to the output register. Otherwise, a bailout is taken to
// the given snapshot. This function overwrites the scratch float register.
void
MacroAssemblerMIPS64::convertDoubleToInt32(FloatRegister src, Register dest,
                                         Label* fail, bool negativeZeroCheck)
{
    // Convert double to int, then convert back and check if we have the
    // same number.
    as_cvtwd(ScratchDoubleReg, src);
    as_mfc1(dest, ScratchDoubleReg);
    as_cvtdw(ScratchDoubleReg, ScratchDoubleReg);
    ma_bc1d(src, ScratchDoubleReg, fail, Assembler::DoubleNotEqualOrUnordered);

    if (negativeZeroCheck) {
        Label notZero;
        ma_b(dest, Imm32(0), &notZero, Assembler::NotEqual, ShortJump);
        // Test and bail for -0.0, when integer result is 0
        // Move the top word of the double into the output reg, if it is
        // non-zero, then the original value was -0.0
        moveFromDoubleHi(src, dest);
        ma_b(dest, Imm32(INT32_MIN), fail, Assembler::Equal);
        bind(&notZero);
    }
}

// Checks whether a float32 is representable as a 32-bit integer. If so, the
// integer is written to the output register. Otherwise, a bailout is taken to
// the given snapshot. This function overwrites the scratch float register.
void
MacroAssemblerMIPS64::convertFloat32ToInt32(FloatRegister src, Register dest,
                                          Label* fail, bool negativeZeroCheck)
{
    // convert the floating point value to an integer, if it did not fit, then
    // when we convert it *back* to  a float, it will have a different value,
    // which we can test.
    as_cvtws(ScratchFloat32Reg, src);
    as_mfc1(dest, ScratchFloat32Reg);
    as_cvtsw(ScratchFloat32Reg, ScratchFloat32Reg);
    ma_bc1s(src, ScratchFloat32Reg, fail, Assembler::DoubleNotEqualOrUnordered);

    if (negativeZeroCheck) {
        Label notZero;
        ma_b(dest, Imm32(0), &notZero, Assembler::NotEqual, ShortJump);
        // Test and bail for -0.0, when integer result is 0
        // Move the top word of the double into the output reg,
        // if it is non-zero, then the original value was -0.0
        moveFromDoubleHi(src, dest);
        ma_b(dest, Imm32(INT32_MIN), fail, Assembler::Equal);
        bind(&notZero);
    }
}

void
MacroAssemblerMIPS64::convertFloat32ToDouble(FloatRegister src, FloatRegister dest)
{
    as_cvtds(dest, src);
}

void
MacroAssemblerMIPS64::branchTruncateFloat32(FloatRegister src, Register dest,
                                          Label* fail)
{
    Label test, success;
    as_truncws(ScratchFloat32Reg, src);
    as_mfc1(dest, ScratchFloat32Reg);

    ma_b(dest, Imm32(INT32_MAX), fail, Assembler::Equal);
}

void
MacroAssemblerMIPS64::convertInt32ToFloat32(Register src, FloatRegister dest)
{
    as_mtc1(src, dest);
    as_cvtsw(dest, dest);
}

void
MacroAssemblerMIPS64::convertInt32ToFloat32(const Address& src, FloatRegister dest)
{
    ma_load(ScratchRegister, src, SizeWord);
    as_mtc1(ScratchRegister, dest);
    as_cvtsw(dest, dest);
}

void
MacroAssemblerMIPS64::addDouble(FloatRegister src, FloatRegister dest)
{
    as_addd(dest, dest, src);
}

void
MacroAssemblerMIPS64::subDouble(FloatRegister src, FloatRegister dest)
{
    as_subd(dest, dest, src);
}

void
MacroAssemblerMIPS64::mulDouble(FloatRegister src, FloatRegister dest)
{
    as_muld(dest, dest, src);
}

void
MacroAssemblerMIPS64::divDouble(FloatRegister src, FloatRegister dest)
{
    as_divd(dest, dest, src);
}

void
MacroAssemblerMIPS64::negateDouble(FloatRegister reg)
{
    as_negd(reg, reg);
}

void
MacroAssemblerMIPS64::inc64(AbsoluteAddress dest)
{
    ma_li(ScratchRegister, ImmWord((uint64_t)dest.addr));
    as_ld(SecondScratchReg, ScratchRegister, 0);
    as_daddiu(SecondScratchReg, SecondScratchReg, 1);
    as_sd(SecondScratchReg, ScratchRegister, 0);
}

void
MacroAssemblerMIPS64::movq(Register rs, Register rd)
{
    ma_move(rd, rs);
}

void
MacroAssemblerMIPS64::ma_move(Register rd, Register rs)
{
    as_or(rd, rs, zero);
}

void
MacroAssemblerMIPS64::ma_li(Register dest, ImmGCPtr ptr)
{
    writeDataRelocation(ptr);
    ma_liPatchable(dest, ImmWord(uintptr_t(ptr.value)));
}

void
MacroAssemblerMIPS64::ma_li(Register dest, AbsoluteLabel* label)
{
    MOZ_ASSERT(!label->bound());
    // Thread the patch list through the unpatched address word in the
    // instruction stream.
    BufferOffset bo = m_buffer.nextOffset();
    ma_liPatchable(dest, ImmWord(label->prev()));
    label->setPrev(bo.getOffset());
}

void
MacroAssemblerMIPS64::ma_li(Register dest, Imm32 imm)
{
    if (Imm16::IsInSignedRange(imm.value)) {
        as_addiu(dest, zero, imm.value);
    } else if (Imm16::IsInUnsignedRange(imm.value)) {
        as_ori(dest, zero, Imm16::Lower(imm).encode());
    } else if (Imm16::Lower(imm).encode() == 0) {
        as_lui(dest, Imm16::Upper(imm).encode());
    } else {
        as_lui(dest, Imm16::Upper(imm).encode());
        as_ori(dest, dest, Imm16::Lower(imm).encode());
    }
}

void
MacroAssemblerMIPS64::ma_li(Register dest, ImmWord imm)
{
    if ((int64_t)imm.value >= INT16_MIN  && (int64_t)imm.value <= INT16_MAX) {
        as_addiu(dest, zero, imm.value);
    } else if (imm.value <= UINT16_MAX) {
        as_ori(dest, zero, Imm16::Lower(Imm32(imm.value)).encode());
    } else if (0 == (imm.value & 0xffff) && 0 == (imm.value >> 32)) {
        as_lui(dest, Imm16::Upper(Imm32(imm.value)).encode());
    } else if (imm.value <= UINT32_MAX) {
        as_lui(dest, Imm16::Upper(Imm32(imm.value)).encode());
        as_ori(dest, dest, Imm16::Lower(Imm32(imm.value)).encode());
    } else {
        as_lui(dest, Imm16::Upper(Imm32(imm.value >> 32)).encode());
        as_ori(dest, dest, Imm16::Lower(Imm32(imm.value >> 32)).encode());
        as_dsll(dest, dest, 16);
        as_ori(dest, dest, Imm16::Upper(Imm32(imm.value)).encode());
        as_dsll(dest, dest, 16);
        as_ori(dest, dest, Imm16::Lower(Imm32(imm.value)).encode());
    }
}

// This method generates lui, dsll and ori instruction block that can be modified
// by UpdateLoad64Value, either during compilation (eg. Assembler::bind), or
// during execution (eg. jit::PatchJump).
void
MacroAssemblerMIPS64::ma_liPatchable(Register dest, ImmPtr imm)
{
    return ma_liPatchable(dest, ImmWord(uintptr_t(imm.value)));
}

void
MacroAssemblerMIPS64::ma_liPatchable(Register dest, ImmWord imm)
{
    m_buffer.ensureSpace(6 * sizeof(uint32_t));
    as_lui(dest, Imm16::Upper(Imm32(imm.value >> 32)).encode());
    as_ori(dest, dest, Imm16::Lower(Imm32(imm.value >> 32)).encode());
    as_dsll(dest, dest, 16);
    as_ori(dest, dest, Imm16::Upper(Imm32(imm.value)).encode());
    as_dsll(dest, dest, 16);
    as_ori(dest, dest, Imm16::Lower(Imm32(imm.value)).encode());
}

// Shifts
void
MacroAssemblerMIPS64::ma_sll(Register rd, Register rt, Imm32 shift)
{
    as_sll(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_dsll(Register rd, Register rt, Imm32 shift)
{
    if (31 < shift.value)
      as_dsll32(rd, rt, shift.value % 64);
    else
      as_dsll(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_srl(Register rd, Register rt, Imm32 shift)
{
    as_srl(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_dsrl(Register rd, Register rt, Imm32 shift)
{
    if (31 < shift.value)
      as_dsrl32(rd, rt, shift.value % 64);
    else
      as_dsrl(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_sra(Register rd, Register rt, Imm32 shift)
{
    as_sra(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_dsra(Register rd, Register rt, Imm32 shift)
{
    if (31 < shift.value)
      as_dsra32(rd, rt, shift.value % 64);
    else
      as_dsra(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_dror(Register rd, Register rt, Imm32 shift)
{
    if (31 < shift.value)
      as_drotr32(rd, rt, shift.value % 64);
    else
      as_drotr(rd, rt, shift.value % 32);
}

void
MacroAssemblerMIPS64::ma_drol(Register rd, Register rt, Imm32 shift)
{
    uint32_t s =  64 - (shift.value % 64);

    if (31 < s)
      as_drotr32(rd, rt, s);
    else
      as_drotr(rd, rt, s);
}

void
MacroAssemblerMIPS64::ma_sll(Register rd, Register rt, Register shift)
{
    as_sllv(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_dsll(Register rd, Register rt, Register shift)
{
    as_dsllv(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_srl(Register rd, Register rt, Register shift)
{
    as_srlv(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_dsrl(Register rd, Register rt, Register shift)
{
    as_dsrlv(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_sra(Register rd, Register rt, Register shift)
{
    as_srav(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_dsra(Register rd, Register rt, Register shift)
{
    as_dsrav(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_dror(Register rd, Register rt, Register shift)
{
    as_drotrv(rd, rt, shift);
}

void
MacroAssemblerMIPS64::ma_drol(Register rd, Register rt, Register shift)
{
    ma_negu(ScratchRegister, shift);
    as_drotrv(rd, rt, ScratchRegister);
}

void
MacroAssemblerMIPS64::ma_dins(Register rt, Register rs, Imm32 pos, Imm32 size)
{
    if (pos.value >= 0 && pos.value < 32) {
        if (size.value >= 2)
          as_dinsm(rt, rs, pos.value, size.value);
        else
          as_dins(rt, rs, pos.value, size.value);
    } else {
        as_dinsu(rt, rs, pos.value, size.value);
    }
}

void
MacroAssemblerMIPS64::ma_dext(Register rt, Register rs, Imm32 pos, Imm32 size)
{
    if (pos.value >= 0 && pos.value < 32) {
        if (size.value > 32)
          as_dextm(rt, rs, pos.value, size.value);
        else
          as_dext(rt, rs, pos.value, size.value);
    } else {
        as_dextu(rt, rs, pos.value, size.value);
    }
}

void
MacroAssemblerMIPS64::ma_negu(Register rd, Register rs)
{
    as_subu(rd, zero, rs);
}

void
MacroAssemblerMIPS64::ma_not(Register rd, Register rs)
{
    as_nor(rd, rs, zero);
}

// And.
void
MacroAssemblerMIPS64::ma_and(Register rd, Register rs)
{
    as_and(rd, rd, rs);
}

void
MacroAssemblerMIPS64::ma_and(Register rd, Imm32 imm)
{
    ma_and(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_and(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        as_andi(rd, rs, imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_and(rd, rs, ScratchRegister);
    }
}

// Or.
void
MacroAssemblerMIPS64::ma_or(Register rd, Register rs)
{
    as_or(rd, rd, rs);
}

void
MacroAssemblerMIPS64::ma_or(Register rd, Imm32 imm)
{
    ma_or(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_or(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        as_ori(rd, rs, imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_or(rd, rs, ScratchRegister);
    }
}

// xor
void
MacroAssemblerMIPS64::ma_xor(Register rd, Register rs)
{
    as_xor(rd, rd, rs);
}

void
MacroAssemblerMIPS64::ma_xor(Register rd, Imm32 imm)
{
    ma_xor(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_xor(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInUnsignedRange(imm.value)) {
        as_xori(rd, rs, imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_xor(rd, rs, ScratchRegister);
    }
}

// Arithmetic-based ops.

// Add.
void
MacroAssemblerMIPS64::ma_addu(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInSignedRange(imm.value)) {
        as_addiu(rd, rs, imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_addu(rd, rs, ScratchRegister);
    }
}

void
MacroAssemblerMIPS64::ma_addu(Register rd, Imm32 imm)
{
    ma_addu(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_daddu(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInSignedRange(imm.value)) {
        as_daddiu(rd, rs, imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_daddu(rd, rs, ScratchRegister);
    }
}

void
MacroAssemblerMIPS64::ma_daddu(Register rd, Register rs)
{
    as_daddu(rd, rd, rs);
}

void
MacroAssemblerMIPS64::ma_daddu(Register rd, Imm32 imm)
{
    ma_daddu(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_addTestOverflow(Register rd, Register rs, Register rt, Label* overflow)
{
    Label goodAddition;
    as_addu(rd, rs, rt);

    as_xor(ScratchRegister, rs, rt); // If different sign, no overflow
    ma_b(ScratchRegister, Imm32(0), &goodAddition, Assembler::LessThan, ShortJump);

    // If different sign, then overflow
    as_xor(ScratchRegister, rs, rd);
    ma_b(ScratchRegister, Imm32(0), overflow, Assembler::LessThan);

    bind(&goodAddition);
}

void
MacroAssemblerMIPS64::ma_addTestOverflow(Register rd, Register rs, Imm32 imm, Label* overflow)
{
    // Check for signed range because of as_addiu
    // Check for unsigned range because of as_xori
    if (Imm16::IsInSignedRange(imm.value) && Imm16::IsInUnsignedRange(imm.value)) {
        Label goodAddition;
        as_addiu(rd, rs, imm.value);

        // If different sign, no overflow
        as_xori(ScratchRegister, rs, imm.value);
        ma_b(ScratchRegister, Imm32(0), &goodAddition, Assembler::LessThan, ShortJump);

        // If different sign, then overflow
        as_xor(ScratchRegister, rs, rd);
        ma_b(ScratchRegister, Imm32(0), overflow, Assembler::LessThan);

        bind(&goodAddition);
    } else {
        ma_li(ScratchRegister, imm);
        ma_addTestOverflow(rd, rs, ScratchRegister, overflow);
    }
}

// Subtract.
void
MacroAssemblerMIPS64::ma_subu(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInSignedRange(-imm.value)) {
        as_addiu(rd, rs, -imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_subu(rd, rs, ScratchRegister);
    }
}

void
MacroAssemblerMIPS64::ma_subu(Register rd, Imm32 imm)
{
    ma_subu(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_dsubu(Register rd, Register rs, Imm32 imm)
{
    if (Imm16::IsInSignedRange(-imm.value)) {
        as_daddiu(rd, rs, -imm.value);
    } else {
        ma_li(ScratchRegister, imm);
        as_dsubu(rd, rs, ScratchRegister);
    }
}

void
MacroAssemblerMIPS64::ma_dsubu(Register rd, Imm32 imm)
{
    ma_dsubu(rd, rd, imm);
}

void
MacroAssemblerMIPS64::ma_subTestOverflow(Register rd, Register rs, Register rt, Label* overflow)
{
    Label goodSubtraction;
    // Use second scratch. The instructions generated by ma_b don't use the
    // second scratch register.
    as_subu(rd, rs, rt);

    as_xor(ScratchRegister, rs, rt); // If same sign, no overflow
    ma_b(ScratchRegister, Imm32(0), &goodSubtraction, Assembler::GreaterThanOrEqual, ShortJump);

    // If different sign, then overflow
    as_xor(ScratchRegister, rs, rd);
    ma_b(ScratchRegister, Imm32(0), overflow, Assembler::LessThan);

    bind(&goodSubtraction);
}

void
MacroAssemblerMIPS64::ma_subTestOverflow(Register rd, Register rs, Imm32 imm, Label* overflow)
{
    if (imm.value != INT32_MIN) {
        ma_addTestOverflow(rd, rs, Imm32(-imm.value), overflow);
    } else {
        ma_li(ScratchRegister, Imm32(imm.value));
        ma_subTestOverflow(rd, rs, ScratchRegister, overflow);
    }
}

void
MacroAssemblerMIPS64::ma_mult(Register rs, Imm32 imm)
{
    ma_li(ScratchRegister, imm);
    as_mult(rs, ScratchRegister);
}

void
MacroAssemblerMIPS64::ma_dmult(Register rs, Imm32 imm)
{
    ma_li(ScratchRegister, imm);
    as_dmult(rs, ScratchRegister);
}

void
MacroAssemblerMIPS64::ma_mul_branch_overflow(Register rd, Register rs, Register rt, Label* overflow)
{
    as_mult(rs, rt);
    as_mflo(rd);
    as_sra(ScratchRegister, rd, 31);
    as_mfhi(SecondScratchReg);
    ma_b(ScratchRegister, SecondScratchReg, overflow, Assembler::NotEqual);
}

void
MacroAssemblerMIPS64::ma_mul_branch_overflow(Register rd, Register rs, Imm32 imm, Label* overflow)
{
    ma_li(ScratchRegister, imm);
    ma_mul_branch_overflow(rd, rs, ScratchRegister, overflow);
}

void
MacroAssemblerMIPS64::ma_div_branch_overflow(Register rd, Register rs, Register rt, Label* overflow)
{
    as_div(rs, rt);
    as_mflo(rd);
    as_mfhi(ScratchRegister);
    ma_b(ScratchRegister, ScratchRegister, overflow, Assembler::NonZero);
}

void
MacroAssemblerMIPS64::ma_div_branch_overflow(Register rd, Register rs, Imm32 imm, Label* overflow)
{
    ma_li(ScratchRegister, imm);
    ma_div_branch_overflow(rd, rs, ScratchRegister, overflow);
}

void
MacroAssemblerMIPS64::ma_mod_mask(Register src, Register dest, Register hold, Register remain,
                                int32_t shift, Label* negZero)
{
    // MATH:
    // We wish to compute x % (1<<y) - 1 for a known constant, y.
    // First, let b = (1<<y) and C = (1<<y)-1, then think of the 32 bit
    // dividend as a number in base b, namely
    // c_0*1 + c_1*b + c_2*b^2 ... c_n*b^n
    // now, since both addition and multiplication commute with modulus,
    // x % C == (c_0 + c_1*b + ... + c_n*b^n) % C ==
    // (c_0 % C) + (c_1%C) * (b % C) + (c_2 % C) * (b^2 % C)...
    // now, since b == C + 1, b % C == 1, and b^n % C == 1
    // this means that the whole thing simplifies to:
    // c_0 + c_1 + c_2 ... c_n % C
    // each c_n can easily be computed by a shift/bitextract, and the modulus
    // can be maintained by simply subtracting by C whenever the number gets
    // over C.
    int32_t mask = (1 << shift) - 1;
    Label head, negative, sumSigned, done;

    // hold holds -1 if the value was negative, 1 otherwise.
    // remain holds the remaining bits that have not been processed
    // SecondScratchReg serves as a temporary location to store extracted bits
    // into as well as holding the trial subtraction as a temp value dest is
    // the accumulator (and holds the final result)

    // move the whole value into the remain.
    ma_move(remain, src);
    // Zero out the dest.
    ma_li(dest, Imm32(0));
    // Set the hold appropriately.
    ma_b(remain, remain, &negative, Signed, ShortJump);
    ma_li(hold, Imm32(1));
    ma_b(&head, ShortJump);

    bind(&negative);
    ma_li(hold, Imm32(-1));
    ma_negu(remain, remain);

    // Begin the main loop.
    bind(&head);

    // Extract the bottom bits into SecondScratchReg.
    ma_and(SecondScratchReg, remain, Imm32(mask));
    // Add those bits to the accumulator.
    as_addu(dest, dest, SecondScratchReg);
    // Do a trial subtraction
    ma_subu(SecondScratchReg, dest, Imm32(mask));
    // If (sum - C) > 0, store sum - C back into sum, thus performing a
    // modulus.
    ma_b(SecondScratchReg, SecondScratchReg, &sumSigned, Signed, ShortJump);
    ma_move(dest, SecondScratchReg);
    bind(&sumSigned);
    // Get rid of the bits that we extracted before.
    as_srl(remain, remain, shift);
    // If the shift produced zero, finish, otherwise, continue in the loop.
    ma_b(remain, remain, &head, NonZero, ShortJump);
    // Check the hold to see if we need to negate the result.
    ma_b(hold, hold, &done, NotSigned, ShortJump);

    // If the hold was non-zero, negate the result to be in line with
    // what JS wants
    if (negZero != nullptr) {
        // Jump out in case of negative zero.
        ma_b(hold, hold, negZero, Zero);
        ma_negu(dest, dest);
    } else {
        ma_negu(dest, dest);
    }

    bind(&done);
}

// Memory.

void
MacroAssemblerMIPS64::ma_load(Register dest, Address address,
                            LoadStoreSize size, LoadStoreExtension extension)
{
    int16_t encodedOffset;
    Register base;
    if (!Imm16::IsInSignedRange(address.offset)) {
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        base = ScratchRegister;
        encodedOffset = Imm16(0).encode();
    } else {
        encodedOffset = Imm16(address.offset).encode();
        base = address.base;
    }

    switch (size) {
      case SizeByte:
        if (ZeroExtend == extension)
            as_lbu(dest, base, encodedOffset);
        else
            as_lb(dest, base, encodedOffset);
        break;
      case SizeHalfWord:
        if (ZeroExtend == extension)
            as_lhu(dest, base, encodedOffset);
        else
            as_lh(dest, base, encodedOffset);
        break;
      case SizeWord:
        if (ZeroExtend == extension)
            as_lwu(dest, base, encodedOffset);
        else
            as_lw(dest, base, encodedOffset);
        break;
      case SizeDouble:
        as_ld(dest, base, encodedOffset);
        break;
      default:
        MOZ_CRASH("Invalid argument for ma_load");
    }
}

void
MacroAssemblerMIPS64::ma_load(Register dest, const BaseIndex& src,
                            LoadStoreSize size, LoadStoreExtension extension)
{
    computeScaledAddress(src, SecondScratchReg);
    ma_load(dest, Address(SecondScratchReg, src.offset), size, extension);
}

void
MacroAssemblerMIPS64::ma_store(Register data, Address address, LoadStoreSize size,
                             LoadStoreExtension extension)
{
    int16_t encodedOffset;
    Register base;
    if (!Imm16::IsInSignedRange(address.offset)) {
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        base = ScratchRegister;
        encodedOffset = Imm16(0).encode();
    } else {
        encodedOffset = Imm16(address.offset).encode();
        base = address.base;
    }

    switch (size) {
      case SizeByte:
        as_sb(data, base, encodedOffset);
        break;
      case SizeHalfWord:
        as_sh(data, base, encodedOffset);
        break;
      case SizeWord:
        as_sw(data, base, encodedOffset);
        break;
      case SizeDouble:
        as_sd(data, base, encodedOffset);
        break;
      default:
        MOZ_CRASH("Invalid argument for ma_store");
    }
}

void
MacroAssemblerMIPS64::ma_store(Register data, const BaseIndex& dest,
                             LoadStoreSize size, LoadStoreExtension extension)
{
    computeScaledAddress(dest, SecondScratchReg);
    ma_store(data, Address(SecondScratchReg, dest.offset), size, extension);
}

void
MacroAssemblerMIPS64::ma_store(Imm32 imm, const BaseIndex& dest,
                             LoadStoreSize size, LoadStoreExtension extension)
{
    // Make sure that SecondScratchReg contains absolute address so that
    // offset is 0.
    computeEffectiveAddress(dest, SecondScratchReg);

    // Scrach register is free now, use it for loading imm value
    ma_li(ScratchRegister, imm);

    // with offset=0 ScratchRegister will not be used in ma_store()
    // so we can use it as a parameter here
    ma_store(ScratchRegister, Address(SecondScratchReg, 0), size, extension);
}

void
MacroAssemblerMIPS64::computeScaledAddress(const BaseIndex& address, Register dest)
{
    int32_t shift = Imm32::ShiftOf(address.scale).value;
    if (shift) {
        ma_dsll(ScratchRegister, address.index, Imm32(shift));
        as_daddu(dest, address.base, ScratchRegister);
    } else {
        as_daddu(dest, address.base, address.index);
    }
}

// Shortcut for when we know we're transferring 32 bits of data.
void
MacroAssemblerMIPS64::ma_pop(Register r)
{
    as_ld(r, StackPointer, 0);
    as_daddiu(StackPointer, StackPointer, sizeof(intptr_t));
}

void
MacroAssemblerMIPS64::ma_push(Register r)
{
    if (r == sp) {
        // Pushing sp requires one more instruction.
        ma_move(ScratchRegister, sp);
        r = ScratchRegister;
    }

    as_daddiu(StackPointer, StackPointer, (int32_t)-sizeof(intptr_t));
    as_sd(r, StackPointer, 0);
}

// Branches when done from within mips-specific code.
void
MacroAssemblerMIPS64::ma_b(Register lhs, Register rhs, Label* label, Condition c, JumpKind jumpKind)
{
    switch (c) {
      case Equal :
      case NotEqual:
        branchWithCode(getBranchCode(lhs, rhs, c), label, jumpKind);
        break;
      case Always:
        ma_b(label, jumpKind);
        break;
      case Zero:
      case NonZero:
      case Signed:
      case NotSigned:
        MOZ_ASSERT(lhs == rhs);
        branchWithCode(getBranchCode(lhs, c), label, jumpKind);
        break;
      default:
        Condition cond = ma_cmp(ScratchRegister, lhs, rhs, c);
        branchWithCode(getBranchCode(ScratchRegister, cond), label, jumpKind);
        break;
    }
}

void
MacroAssemblerMIPS64::ma_b(Register lhs, Imm32 imm, Label* label, Condition c, JumpKind jumpKind)
{
    MOZ_ASSERT(c != Overflow);
    if (imm.value == 0) {
        if (c == Always || c == AboveOrEqual)
            ma_b(label, jumpKind);
        else if (c == Below)
            ; // This condition is always false. No branch required.
        else
            branchWithCode(getBranchCode(lhs, c), label, jumpKind);
    } else {
        MOZ_ASSERT(lhs != ScratchRegister);
        ma_li(ScratchRegister, imm);
        ma_b(lhs, ScratchRegister, label, c, jumpKind);
    }
}

void
MacroAssemblerMIPS64::ma_b(Register lhs, ImmWord imm, Label* label, Condition c, JumpKind jumpKind)
{
    MOZ_ASSERT(c != Overflow);
    if (imm.value == 0) {
        if (c == Always || c == AboveOrEqual)
            ma_b(label, jumpKind);
        else if (c == Below)
            ; // This condition is always false. No branch required.
        else
            branchWithCode(getBranchCode(lhs, c), label, jumpKind);
    } else {
        MOZ_ASSERT(lhs != ScratchRegister);
        ma_li(ScratchRegister, imm);
        ma_b(lhs, ScratchRegister, label, c, jumpKind);
    }
}

void
MacroAssemblerMIPS64::ma_b(Register lhs, Address addr, Label* label, Condition c, JumpKind jumpKind)
{
    MOZ_ASSERT(lhs != ScratchRegister);
    ma_load(ScratchRegister, addr, SizeDouble);
    ma_b(lhs, ScratchRegister, label, c, jumpKind);
}

void
MacroAssemblerMIPS64::ma_b(Address addr, Imm32 imm, Label* label, Condition c, JumpKind jumpKind)
{
    ma_load(SecondScratchReg, addr, SizeDouble);
    ma_b(SecondScratchReg, imm, label, c, jumpKind);
}

void
MacroAssemblerMIPS64::ma_b(Address addr, ImmGCPtr imm, Label* label, Condition c, JumpKind jumpKind)
{
    ma_load(SecondScratchReg, addr, SizeDouble);
    ma_b(SecondScratchReg, imm, label, c, jumpKind);
}

void
MacroAssemblerMIPS64::ma_b(Label* label, JumpKind jumpKind)
{
    branchWithCode(getBranchCode(BranchIsJump), label, jumpKind);
}

void
MacroAssemblerMIPS64::ma_bal(Label* label, DelaySlotFill delaySlotFill)
{
    if (label->bound()) {
        // Generate the long jump for calls because return address has to be
        // the address after the reserved block.
        addLongJump(nextOffset());
        ma_liPatchable(ScratchRegister, ImmWord(label->offset()));
        as_jalr(ScratchRegister);
        if (delaySlotFill == FillDelaySlot)
            as_nop();
        return;
    }

    // Second word holds a pointer to the next branch in label's chain.
    uint32_t nextInChain = label->used() ? label->offset() : LabelBase::INVALID_OFFSET;

    // Make the whole branch continous in the buffer.
    m_buffer.ensureSpace(8 * sizeof(uint32_t));

    BufferOffset bo = writeInst(getBranchCode(BranchIsCall).encode());
    writeInst(nextInChain);
    label->use(bo.getOffset());
    // Leave space for long jump.
    as_nop();
    as_nop();
    as_nop();
    as_nop();
    as_nop();
    if (delaySlotFill == FillDelaySlot)
        as_nop();
}

void
MacroAssemblerMIPS64::branchWithCode(InstImm code, Label* label, JumpKind jumpKind)
{
    MOZ_ASSERT(code.encode() != InstImm(op_regimm, zero, rt_bgezal, BOffImm16(0)).encode());
    InstImm inst_beq = InstImm(op_beq, zero, zero, BOffImm16(0));

    if (label->bound()) {
        int32_t offset = label->offset() - m_buffer.nextOffset().getOffset();

        if (BOffImm16::IsInRange(offset))
            jumpKind = ShortJump;

        if (jumpKind == ShortJump) {
            MOZ_ASSERT(BOffImm16::IsInRange(offset));
            code.setBOffImm16(BOffImm16(offset));
            writeInst(code.encode());
            as_nop();
            return;
        }

        if (code.encode() == inst_beq.encode()) {
            // Handle long jump
            addLongJump(nextOffset());
            ma_liPatchable(ScratchRegister, ImmWord(label->offset()));
            as_jr(ScratchRegister);
            as_nop();
            return;
        }

        // Handle long conditional branch
        writeInst(invertBranch(code, BOffImm16(9 * sizeof(uint32_t))).encode());
        // No need for a "nop" here because we can clobber scratch.
        addLongJump(nextOffset());
        ma_liPatchable(ScratchRegister, ImmWord(label->offset()));
        as_jr(ScratchRegister);
        as_nop();
        return;
    }

    // Generate open jump and link it to a label.

    // Second word holds a pointer to the next branch in label's chain.
    uint32_t nextInChain = label->used() ? label->offset() : LabelBase::INVALID_OFFSET;

    if (jumpKind == ShortJump) {
        // Make the whole branch continous in the buffer.
        m_buffer.ensureSpace(2 * sizeof(uint32_t));

        // Indicate that this is short jump with offset 4.
        code.setBOffImm16(BOffImm16(4));
        BufferOffset bo = writeInst(code.encode());
        writeInst(nextInChain);
        label->use(bo.getOffset());
        return;
    }

    bool conditional = code.encode() != inst_beq.encode();

    // Make the whole branch continous in the buffer.
    m_buffer.ensureSpace((conditional ? 9 : 8) * sizeof(uint32_t));

    BufferOffset bo = writeInst(code.encode());
    writeInst(nextInChain);
    label->use(bo.getOffset());
    // Leave space for potential long jump.
    as_nop();
    as_nop();
    as_nop();
    as_nop();
    as_nop();
    as_nop();
    if (conditional)
        as_nop();
}

Assembler::Condition
MacroAssemblerMIPS64::ma_cmp(Register scratch, Register lhs, Register rhs, Condition c)
{
    switch (c) {
      case Above:
        // bgtu s,t,label =>
        //   sltu at,t,s
        //   bne at,$zero,offs
        as_sltu(scratch, rhs, lhs);
        return NotEqual;
      case AboveOrEqual:
        // bgeu s,t,label =>
        //   sltu at,s,t
        //   beq at,$zero,offs
        as_sltu(scratch, lhs, rhs);
        return Equal;
      case Below:
        // bltu s,t,label =>
        //   sltu at,s,t
        //   bne at,$zero,offs
        as_sltu(scratch, lhs, rhs);
        return NotEqual;
      case BelowOrEqual:
        // bleu s,t,label =>
        //   sltu at,t,s
        //   beq at,$zero,offs
        as_sltu(scratch, rhs, lhs);
        return Equal;
      case GreaterThan:
        // bgt s,t,label =>
        //   slt at,t,s
        //   bne at,$zero,offs
        as_slt(scratch, rhs, lhs);
        return NotEqual;
      case GreaterThanOrEqual:
        // bge s,t,label =>
        //   slt at,s,t
        //   beq at,$zero,offs
        as_slt(scratch, lhs, rhs);
        return Equal;
      case LessThan:
        // blt s,t,label =>
        //   slt at,s,t
        //   bne at,$zero,offs
        as_slt(scratch, lhs, rhs);
        return NotEqual;
      case LessThanOrEqual:
        // ble s,t,label =>
        //   slt at,t,s
        //   beq at,$zero,offs
        as_slt(scratch, rhs, lhs);
        return Equal;
      case Equal :
      case NotEqual:
      case Zero:
      case NonZero:
      case Always:
      case Signed:
      case NotSigned:
        MOZ_CRASH("There is a better way to compare for equality.");
        break;
      case Overflow:
        MOZ_CRASH("Overflow condition not supported for MIPS.");
        break;
      default:
        MOZ_CRASH("Invalid condition for branch.");
    }
    return Always;
}

void
MacroAssemblerMIPS64::ma_cmp_set(Register rd, Register rs, Register rt, Condition c)
{
    switch (c) {
      case Equal :
        // seq d,s,t =>
        //   xor d,s,t
        //   sltiu d,d,1
        as_xor(rd, rs, rt);
        as_sltiu(rd, rd, 1);
        break;
      case NotEqual:
        // sne d,s,t =>
        //   xor d,s,t
        //   sltu d,$zero,d
        as_xor(rd, rs, rt);
        as_sltu(rd, zero, rd);
        break;
      case Above:
        // sgtu d,s,t =>
        //   sltu d,t,s
        as_sltu(rd, rt, rs);
        break;
      case AboveOrEqual:
        // sgeu d,s,t =>
        //   sltu d,s,t
        //   xori d,d,1
        as_sltu(rd, rs, rt);
        as_xori(rd, rd, 1);
        break;
      case Below:
        // sltu d,s,t
        as_sltu(rd, rs, rt);
        break;
      case BelowOrEqual:
        // sleu d,s,t =>
        //   sltu d,t,s
        //   xori d,d,1
        as_sltu(rd, rt, rs);
        as_xori(rd, rd, 1);
        break;
      case GreaterThan:
        // sgt d,s,t =>
        //   slt d,t,s
        as_slt(rd, rt, rs);
        break;
      case GreaterThanOrEqual:
        // sge d,s,t =>
        //   slt d,s,t
        //   xori d,d,1
        as_slt(rd, rs, rt);
        as_xori(rd, rd, 1);
        break;
      case LessThan:
        // slt d,s,t
        as_slt(rd, rs, rt);
        break;
      case LessThanOrEqual:
        // sle d,s,t =>
        //   slt d,t,s
        //   xori d,d,1
        as_slt(rd, rt, rs);
        as_xori(rd, rd, 1);
        break;
      case Zero:
        MOZ_ASSERT(rs == rt);
        // seq d,s,$zero =>
        //   xor d,s,$zero
        //   sltiu d,d,1
        as_xor(rd, rs, zero);
        as_sltiu(rd, rd, 1);
        break;
      case NonZero:
        // sne d,s,$zero =>
        //   xor d,s,$zero
        //   sltu d,$zero,d
        as_xor(rd, rs, zero);
        as_sltu(rd, zero, rd);
        break;
      case Signed:
        as_slt(rd, rs, zero);
        break;
      case NotSigned:
        // sge d,s,$zero =>
        //   slt d,s,$zero
        //   xori d,d,1
        as_slt(rd, rs, zero);
        as_xori(rd, rd, 1);
        break;
      default:
        MOZ_CRASH("Invalid condition for ma_cmp_set.");
    }
}

void
MacroAssemblerMIPS64::ma_cmp_set(Register rd, Register rs, Imm32 imm, Condition c)
{
    ma_li(ScratchRegister, imm);
    ma_cmp_set(rd, rs, ScratchRegister, c);
}

void
MacroAssemblerMIPS64::ma_cmp_set(Register rd, Register rs, ImmWord imm, Condition c)
{
    ma_li(ScratchRegister, imm);
    ma_cmp_set(rd, rs, ScratchRegister, c);
}

void
MacroAssemblerMIPS64::ma_cmp_set(Register rd, Register rs, ImmPtr imm, Condition c)
{
    ma_li(ScratchRegister, ImmWord(uint64_t(imm.value)));
    ma_cmp_set(rd, rs, ScratchRegister, c);
}

void
MacroAssemblerMIPS64::compareFloatingPoint(FloatFormat fmt, FloatRegister lhs, FloatRegister rhs,
                                         DoubleCondition c, FloatTestKind* testKind,
                                         FPConditionBit fcc)
{
    switch (c) {
      case DoubleOrdered:
        as_cun(fmt, lhs, rhs, fcc);
        *testKind = TestForFalse;
        break;
      case DoubleEqual:
        as_ceq(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleNotEqual:
        as_cueq(fmt, lhs, rhs, fcc);
        *testKind = TestForFalse;
        break;
      case DoubleGreaterThan:
        as_colt(fmt, rhs, lhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleGreaterThanOrEqual:
        as_cole(fmt, rhs, lhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleLessThan:
        as_colt(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleLessThanOrEqual:
        as_cole(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleUnordered:
        as_cun(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleEqualOrUnordered:
        as_cueq(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleNotEqualOrUnordered:
        as_ceq(fmt, lhs, rhs, fcc);
        *testKind = TestForFalse;
        break;
      case DoubleGreaterThanOrUnordered:
        as_cult(fmt, rhs, lhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleGreaterThanOrEqualOrUnordered:
        as_cule(fmt, rhs, lhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleLessThanOrUnordered:
        as_cult(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      case DoubleLessThanOrEqualOrUnordered:
        as_cule(fmt, lhs, rhs, fcc);
        *testKind = TestForTrue;
        break;
      default:
        MOZ_CRASH("Invalid DoubleCondition.");
    }
}

void
MacroAssemblerMIPS64::ma_cmp_set_double(Register dest, FloatRegister lhs, FloatRegister rhs,
                                      DoubleCondition c)
{
    ma_li(dest, Imm32(0));
    ma_li(ScratchRegister, Imm32(1));

    FloatTestKind moveCondition;
    compareFloatingPoint(DoubleFloat, lhs, rhs, c, &moveCondition);

    if (moveCondition == TestForTrue)
        as_movt(dest, ScratchRegister);
    else
        as_movf(dest, ScratchRegister);
}

void
MacroAssemblerMIPS64::ma_cmp_set_float32(Register dest, FloatRegister lhs, FloatRegister rhs,
                                       DoubleCondition c)
{
    ma_li(dest, Imm32(0));
    ma_li(ScratchRegister, Imm32(1));

    FloatTestKind moveCondition;
    compareFloatingPoint(SingleFloat, lhs, rhs, c, &moveCondition);

    if (moveCondition == TestForTrue)
        as_movt(dest, ScratchRegister);
    else
        as_movf(dest, ScratchRegister);
}

// fp instructions
void
MacroAssemblerMIPS64::ma_lis(FloatRegister dest, float value)
{
    Imm32 imm(mozilla::BitwiseCast<uint32_t>(value));

    ma_li(ScratchRegister, imm);
    moveToFloat32(ScratchRegister, dest);
}

void
MacroAssemblerMIPS64::ma_lid(FloatRegister dest, double value)
{
    struct DoubleStruct {
        uint32_t lo;
        uint32_t hi;
    } ;
    DoubleStruct intStruct = mozilla::BitwiseCast<DoubleStruct>(value);

    // put low part of 64 bit value into the even register
    if (intStruct.lo == 0) {
        moveToDoubleLo(zero, dest);
    } else {
        ma_li(ScratchRegister, Imm32(intStruct.lo));
        moveToDoubleLo(ScratchRegister, dest);
    }

    // put hi part of 64 bit value into the odd register
    if (intStruct.hi == 0) {
        moveToDoubleHi(zero, dest);
    } else {
        ma_li(ScratchRegister, Imm32(intStruct.hi));
        moveToDoubleHi(ScratchRegister, dest);
    }
}

void
MacroAssemblerMIPS64::ma_liNegZero(FloatRegister dest)
{
    moveToDoubleLo(zero, dest);
    ma_li(ScratchRegister, Imm32(INT_MIN));
    moveToDoubleHi(ScratchRegister, dest);
}

void
MacroAssemblerMIPS64::ma_mv(FloatRegister src, ValueOperand dest)
{
    as_dmfc1(dest.valueReg(), src);
}

void
MacroAssemblerMIPS64::ma_mv(ValueOperand src, FloatRegister dest)
{
    as_dmtc1(src.valueReg(), dest);
}

void
MacroAssemblerMIPS64::ma_ls(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        as_ls(ft, address.base, address.offset);
    } else {
        MOZ_ASSERT(address.base != ScratchRegister);
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        as_ls(ft, ScratchRegister, 0);
    }
}

void
MacroAssemblerMIPS64::ma_ld(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        as_ld(ft, address.base, address.offset);
    } else {
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        as_ld(ft, ScratchRegister, 0);
    }
}

void
MacroAssemblerMIPS64::ma_sd(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        as_sd(ft, address.base, address.offset);
    } else {
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        as_sd(ft, ScratchRegister, 0);
    }
}

void
MacroAssemblerMIPS64::ma_sd(FloatRegister ft, BaseIndex address)
{
    computeScaledAddress(address, SecondScratchReg);
    ma_sd(ft, Address(SecondScratchReg, address.offset));
}

void
MacroAssemblerMIPS64::ma_ss(FloatRegister ft, Address address)
{
    if (Imm16::IsInSignedRange(address.offset)) {
        as_ss(ft, address.base, address.offset);
    } else {
        ma_li(ScratchRegister, Imm32(address.offset));
        as_daddu(ScratchRegister, address.base, ScratchRegister);
        as_ss(ft, ScratchRegister, 0);
    }
}

void
MacroAssemblerMIPS64::ma_ss(FloatRegister ft, BaseIndex address)
{
    computeScaledAddress(address, SecondScratchReg);
    ma_ss(ft, Address(SecondScratchReg, address.offset));
}

void
MacroAssemblerMIPS64::ma_pop(FloatRegister fs)
{
    ma_ld(fs, Address(StackPointer, 0));
    as_daddiu(StackPointer, StackPointer, sizeof(double));
}

void
MacroAssemblerMIPS64::ma_push(FloatRegister fs)
{
    as_daddiu(StackPointer, StackPointer, (int32_t)-sizeof(double));
    ma_sd(fs, Address(StackPointer, 0));
}

void
MacroAssemblerMIPS64::ma_bc1s(FloatRegister lhs, FloatRegister rhs, Label* label,
                            DoubleCondition c, JumpKind jumpKind, FPConditionBit fcc)
{
    FloatTestKind testKind;
    compareFloatingPoint(SingleFloat, lhs, rhs, c, &testKind, fcc);
    branchWithCode(getBranchCode(testKind, fcc), label, jumpKind);
}

void
MacroAssemblerMIPS64::ma_bc1d(FloatRegister lhs, FloatRegister rhs, Label* label,
                            DoubleCondition c, JumpKind jumpKind, FPConditionBit fcc)
{
    FloatTestKind testKind;
    compareFloatingPoint(DoubleFloat, lhs, rhs, c, &testKind, fcc);
    branchWithCode(getBranchCode(testKind, fcc), label, jumpKind);
}

void
MacroAssemblerMIPS64Compat::buildFakeExitFrame(Register scratch, uint32_t* offset)
{
    mozilla::DebugOnly<uint32_t> initialDepth = framePushed();

    CodeLabel cl;
    ma_li(scratch, cl.dest());

    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    asMasm().Push(Imm32(descriptor));
    asMasm().Push(scratch);

    bind(cl.src());
    *offset = currentOffset();

    MOZ_ASSERT(framePushed() == initialDepth + ExitFrameLayout::Size());
    addCodeLabel(cl);
}

bool
MacroAssemblerMIPS64Compat::buildOOLFakeExitFrame(void* fakeReturnAddr)
{
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);

    asMasm().Push(Imm32(descriptor)); // descriptor_
    asMasm().Push(ImmPtr(fakeReturnAddr));

    return true;
}

void
MacroAssemblerMIPS64Compat::callWithExitFrame(Label* target)
{
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    asMasm().Push(Imm32(descriptor)); // descriptor

    ma_callJit(target);
}

void
MacroAssemblerMIPS64Compat::callWithExitFrame(JitCode* target)
{
    uint32_t descriptor = MakeFrameDescriptor(framePushed(), JitFrame_IonJS);
    asMasm().Push(Imm32(descriptor)); // descriptor

    addPendingJump(m_buffer.nextOffset(), ImmPtr(target->raw()), Relocation::JITCODE);
    ma_liPatchable(ScratchRegister, ImmPtr(target->raw()));
    ma_callJit(ScratchRegister);
}

void
MacroAssemblerMIPS64Compat::callWithExitFrame(JitCode* target, Register dynStack)
{
    ma_daddu(dynStack, dynStack, Imm32(framePushed()));
    makeFrameDescriptor(dynStack, JitFrame_IonJS);
    asMasm().Push(dynStack); // descriptor

    addPendingJump(m_buffer.nextOffset(), ImmPtr(target->raw()), Relocation::JITCODE);
    ma_liPatchable(ScratchRegister, ImmPtr(target->raw()));
    ma_callJit(ScratchRegister);
}

void
MacroAssemblerMIPS64Compat::callJit(Register callee)
{
    MOZ_ASSERT((framePushed() & 7) == 0);
    ma_callJit(callee);
}

void
MacroAssemblerMIPS64Compat::reserveStack(uint32_t amount)
{
    if (amount)
        ma_dsubu(StackPointer, StackPointer, Imm32(amount));
    adjustFrame(amount);
}

void
MacroAssemblerMIPS64Compat::freeStack(uint32_t amount)
{
    MOZ_ASSERT(amount <= framePushed_);
    if (amount)
        ma_daddu(StackPointer, StackPointer, Imm32(amount));
    adjustFrame(-amount);
}

void
MacroAssemblerMIPS64Compat::freeStack(Register amount)
{
    as_daddu(StackPointer, StackPointer, amount);
}

void
MacroAssemblerMIPS64Compat::add32(Register src, Register dest)
{
    as_addu(dest, dest, src);
}

void
MacroAssemblerMIPS64Compat::add32(Imm32 imm, Register dest)
{
    ma_addu(dest, dest, imm);
}

void

MacroAssemblerMIPS64Compat::add32(Imm32 imm, const Address& dest)
{
    load32(dest, SecondScratchReg);
    ma_addu(SecondScratchReg, imm);
    store32(SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::sub32(Imm32 imm, Register dest)
{
    ma_subu(dest, dest, imm);
}

void
MacroAssemblerMIPS64Compat::sub32(Register src, Register dest)
{
    as_subu(dest, dest, src);
}

void
MacroAssemblerMIPS64Compat::addPtr(Register src, Register dest)
{
    ma_daddu(dest, src);
}

void
MacroAssemblerMIPS64Compat::addPtr(const Address& src, Register dest)
{
    loadPtr(src, ScratchRegister);
    ma_daddu(dest, ScratchRegister);
}

void
MacroAssemblerMIPS64Compat::subPtr(Register src, Register dest)
{
    as_dsubu(dest, dest, src);
}

void
MacroAssemblerMIPS64Compat::not32(Register reg)
{
    ma_not(reg, reg);
}

// Logical operations
void
MacroAssemblerMIPS64Compat::and32(Register src, Register dest)
{
    as_and(dest, dest, src);
}

void
MacroAssemblerMIPS64Compat::and32(Imm32 imm, Register dest)
{
    ma_and(dest, imm);
}

void
MacroAssemblerMIPS64Compat::and32(Imm32 imm, const Address& dest)
{
    load32(dest, SecondScratchReg);
    ma_and(SecondScratchReg, imm);
    store32(SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::and32(const Address& src, Register dest)
{
    load32(src, SecondScratchReg);
    ma_and(dest, SecondScratchReg);
}

void
MacroAssemblerMIPS64Compat::or32(Imm32 imm, Register dest)
{
    ma_or(dest, imm);
}


void
MacroAssemblerMIPS64Compat::or32(Imm32 imm, const Address& dest)
{
    load32(dest, SecondScratchReg);
    ma_or(SecondScratchReg, imm);
    store32(SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::xor32(Imm32 imm, Register dest)
{
    ma_xor(dest, imm);
}

void
MacroAssemblerMIPS64Compat::xorPtr(Imm32 imm, Register dest)
{
    ma_xor(dest, imm);
}

void
MacroAssemblerMIPS64Compat::xorPtr(Register src, Register dest)
{
    ma_xor(dest, src);
}

void
MacroAssemblerMIPS64Compat::orPtr(Imm32 imm, Register dest)
{
    ma_or(dest, imm);
}

void
MacroAssemblerMIPS64Compat::orPtr(Register src, Register dest)
{
    ma_or(dest, src);
}

void
MacroAssemblerMIPS64Compat::andPtr(Imm32 imm, Register dest)
{
    ma_and(dest, imm);
}

void
MacroAssemblerMIPS64Compat::andPtr(ImmWord imm, Register dest)
{
    ma_li(ScratchRegister, imm);
    ma_and(dest, ScratchRegister);
}

void
MacroAssemblerMIPS64Compat::andPtr(Register src, Register dest)
{
    ma_and(dest, src);
}

void
MacroAssemblerMIPS64Compat::move32(Imm32 imm, Register dest)
{
    ma_li(dest, imm);
}

void
MacroAssemblerMIPS64Compat::move32(Register src, Register dest)
{
    ma_move(dest, src);
}

void
MacroAssemblerMIPS64Compat::movePtr(Register src, Register dest)
{
    ma_move(dest, src);
}
void
MacroAssemblerMIPS64Compat::movePtr(ImmWord imm, Register dest)
{
    ma_li(dest, ImmWord(imm.value));
}

void
MacroAssemblerMIPS64Compat::movePtr(ImmGCPtr imm, Register dest)
{
    ma_li(dest, imm);
}

void
MacroAssemblerMIPS64Compat::movePtr(ImmMaybeNurseryPtr imm, Register dest)
{
    movePtr(noteMaybeNurseryPtr(imm), dest);
}
void
MacroAssemblerMIPS64Compat::movePtr(ImmPtr imm, Register dest)
{
    movePtr(ImmWord(uintptr_t(imm.value)), dest);
}
void
MacroAssemblerMIPS64Compat::movePtr(AsmJSImmPtr imm, Register dest)
{
    append(AsmJSAbsoluteLink(CodeOffsetLabel(nextOffset().getOffset()), imm.kind()));
    ma_liPatchable(dest, ImmWord(-1));
}

void
MacroAssemblerMIPS64Compat::load8ZeroExtend(const Address& address, Register dest)
{
    ma_load(dest, address, SizeByte, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::load8ZeroExtend(const BaseIndex& src, Register dest)
{
    ma_load(dest, src, SizeByte, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::load8SignExtend(const Address& address, Register dest)
{
    ma_load(dest, address, SizeByte, SignExtend);
}

void
MacroAssemblerMIPS64Compat::load8SignExtend(const BaseIndex& src, Register dest)
{
    ma_load(dest, src, SizeByte, SignExtend);
}

void
MacroAssemblerMIPS64Compat::load16ZeroExtend(const Address& address, Register dest)
{
    ma_load(dest, address, SizeHalfWord, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::load16ZeroExtend(const BaseIndex& src, Register dest)
{
    ma_load(dest, src, SizeHalfWord, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::load16SignExtend(const Address& address, Register dest)
{
    ma_load(dest, address, SizeHalfWord, SignExtend);
}

void
MacroAssemblerMIPS64Compat::load16SignExtend(const BaseIndex& src, Register dest)
{
    ma_load(dest, src, SizeHalfWord, SignExtend);
}

void
MacroAssemblerMIPS64Compat::load32(const Address& address, Register dest)
{
    ma_load(dest, address, SizeWord);
}

void
MacroAssemblerMIPS64Compat::load32(const BaseIndex& address, Register dest)
{
    ma_load(dest, address, SizeWord);
}

void
MacroAssemblerMIPS64Compat::load32(AbsoluteAddress address, Register dest)
{
    movePtr(ImmPtr(address.addr), ScratchRegister);
    load32(Address(ScratchRegister, 0), dest);
}

void
MacroAssemblerMIPS64Compat::load32(AsmJSAbsoluteAddress address, Register dest)
{
    movePtr(AsmJSImmPtr(address.kind()), ScratchRegister);
    load32(Address(ScratchRegister, 0), dest);
}

void
MacroAssemblerMIPS64Compat::loadPtr(const Address& address, Register dest)
{
    ma_load(dest, address, SizeDouble);
}

void
MacroAssemblerMIPS64Compat::loadPtr(const BaseIndex& src, Register dest)
{
    ma_load(dest, src, SizeDouble);
}

void
MacroAssemblerMIPS64Compat::loadPtr(AbsoluteAddress address, Register dest)
{
    movePtr(ImmPtr(address.addr), ScratchRegister);
    loadPtr(Address(ScratchRegister, 0), dest);
}

void
MacroAssemblerMIPS64Compat::loadPtr(AsmJSAbsoluteAddress address, Register dest)
{
    movePtr(AsmJSImmPtr(address.kind()), ScratchRegister);
    loadPtr(Address(ScratchRegister, 0), dest);
}

void
MacroAssemblerMIPS64Compat::loadPrivate(const Address& address, Register dest)
{
    loadPtr(address, dest);
    ma_dsll(dest, dest, Imm32(1));
}

void
MacroAssemblerMIPS64Compat::loadDouble(const Address& address, FloatRegister dest)
{
    ma_ld(dest, address);
}

void
MacroAssemblerMIPS64Compat::loadDouble(const BaseIndex& src, FloatRegister dest)
{
    computeScaledAddress(src, SecondScratchReg);
    ma_ld(dest, Address(SecondScratchReg, src.offset));
}

void
MacroAssemblerMIPS64Compat::loadFloatAsDouble(const Address& address, FloatRegister dest)
{
    ma_ls(dest, address);
    as_cvtds(dest, dest);
}

void
MacroAssemblerMIPS64Compat::loadFloatAsDouble(const BaseIndex& src, FloatRegister dest)
{
    loadFloat32(src, dest);
    as_cvtds(dest, dest);
}

void
MacroAssemblerMIPS64Compat::loadFloat32(const Address& address, FloatRegister dest)
{
    ma_ls(dest, address);
}

void
MacroAssemblerMIPS64Compat::loadFloat32(const BaseIndex& src, FloatRegister dest)
{
    computeScaledAddress(src, SecondScratchReg);
    ma_ls(dest, Address(SecondScratchReg, src.offset));
}

void
MacroAssemblerMIPS64Compat::store8(Imm32 imm, const Address& address)
{
    ma_li(SecondScratchReg, imm);
    ma_store(SecondScratchReg, address, SizeByte);
}

void
MacroAssemblerMIPS64Compat::store8(Register src, const Address& address)
{
    ma_store(src, address, SizeByte);
}

void
MacroAssemblerMIPS64Compat::store8(Imm32 imm, const BaseIndex& dest)
{
    ma_store(imm, dest, SizeByte);
}

void
MacroAssemblerMIPS64Compat::store8(Register src, const BaseIndex& dest)
{
    ma_store(src, dest, SizeByte);
}

void
MacroAssemblerMIPS64Compat::store16(Imm32 imm, const Address& address)
{
    ma_li(SecondScratchReg, imm);
    ma_store(SecondScratchReg, address, SizeHalfWord);
}

void
MacroAssemblerMIPS64Compat::store16(Register src, const Address& address)
{
    ma_store(src, address, SizeHalfWord);
}

void
MacroAssemblerMIPS64Compat::store16(Imm32 imm, const BaseIndex& dest)
{
    ma_store(imm, dest, SizeHalfWord);
}

void
MacroAssemblerMIPS64Compat::store16(Register src, const BaseIndex& address)
{
    ma_store(src, address, SizeHalfWord);
}

void
MacroAssemblerMIPS64Compat::store32(Register src, AbsoluteAddress address)
{
    movePtr(ImmPtr(address.addr), ScratchRegister);
    store32(src, Address(ScratchRegister, 0));
}

void
MacroAssemblerMIPS64Compat::store32(Register src, const Address& address)
{
    ma_store(src, address, SizeWord);
}

void
MacroAssemblerMIPS64Compat::store32(Imm32 src, const Address& address)
{
    move32(src, SecondScratchReg);
    ma_store(SecondScratchReg, address, SizeWord);
}

void
MacroAssemblerMIPS64Compat::store32(Imm32 imm, const BaseIndex& dest)
{
    ma_store(imm, dest, SizeWord);
}

void
MacroAssemblerMIPS64Compat::store32(Register src, const BaseIndex& dest)
{
    ma_store(src, dest, SizeWord);
}

template <typename T>
void
MacroAssemblerMIPS64Compat::storePtr(ImmWord imm, T address)
{
    ma_li(SecondScratchReg, imm);
    ma_store(SecondScratchReg, address, SizeDouble);
}

template void MacroAssemblerMIPS64Compat::storePtr<Address>(ImmWord imm, Address address);
template void MacroAssemblerMIPS64Compat::storePtr<BaseIndex>(ImmWord imm, BaseIndex address);

template <typename T>
void
MacroAssemblerMIPS64Compat::storePtr(ImmPtr imm, T address)
{
    storePtr(ImmWord(uintptr_t(imm.value)), address);
}

template void MacroAssemblerMIPS64Compat::storePtr<Address>(ImmPtr imm, Address address);
template void MacroAssemblerMIPS64Compat::storePtr<BaseIndex>(ImmPtr imm, BaseIndex address);

template <typename T>
void
MacroAssemblerMIPS64Compat::storePtr(ImmGCPtr imm, T address)
{
    storePtr(ImmWord(uintptr_t(imm.value)), address);
}

template void MacroAssemblerMIPS64Compat::storePtr<Address>(ImmGCPtr imm, Address address);
template void MacroAssemblerMIPS64Compat::storePtr<BaseIndex>(ImmGCPtr imm, BaseIndex address);

void
MacroAssemblerMIPS64Compat::storePtr(Register src, const Address& address)
{
    ma_store(src, address, SizeDouble);
}

void
MacroAssemblerMIPS64Compat::storePtr(Register src, const BaseIndex& address)
{
    ma_store(src, address, SizeDouble);
}

void
MacroAssemblerMIPS64Compat::storePtr(Register src, AbsoluteAddress dest)
{
    movePtr(ImmPtr(dest.addr), ScratchRegister);
    storePtr(src, Address(ScratchRegister, 0));
}

// Note: this function clobbers the input register.
void
MacroAssembler::clampDoubleToUint8(FloatRegister input, Register output)
{
    MOZ_ASSERT(input != ScratchDoubleReg);
    Label positive, done;

    // <= 0 or NaN --> 0
    zeroDouble(ScratchDoubleReg);
    branchDouble(DoubleGreaterThan, input, ScratchDoubleReg, &positive);
    {
        move32(Imm32(0), output);
        jump(&done);
    }

    bind(&positive);

    // Add 0.5 and truncate.
    loadConstantDouble(0.5, ScratchDoubleReg);
    addDouble(ScratchDoubleReg, input);

    Label outOfRange;

    branchTruncateDouble(input, output, &outOfRange);
    branch32(Assembler::Above, output, Imm32(255), &outOfRange);
    {
        // Check if we had a tie.
        convertInt32ToDouble(output, ScratchDoubleReg);
        branchDouble(DoubleNotEqual, input, ScratchDoubleReg, &done);

        // It was a tie. Mask out the ones bit to get an even value.
        // See also js_TypedArray_uint8_clamp_double.
        and32(Imm32(~1), output);
        jump(&done);
    }

    // > 255 --> 255
    bind(&outOfRange);
    {
        move32(Imm32(255), output);
    }

    bind(&done);
}

void
MacroAssemblerMIPS64Compat::subPtr(Imm32 imm, const Register dest)
{
    ma_dsubu(dest, dest, imm);
}

void
MacroAssemblerMIPS64Compat::subPtr(const Address& addr, const Register dest)
{
    loadPtr(addr, SecondScratchReg);
    subPtr(SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::subPtr(Register src, const Address& dest)
{
    loadPtr(dest, SecondScratchReg);
    subPtr(src, SecondScratchReg);
    storePtr(SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::addPtr(Imm32 imm, const Register dest)
{
    ma_daddu(dest, imm);
}

void
MacroAssemblerMIPS64Compat::addPtr(Imm32 imm, const Address& dest)
{
    loadPtr(dest, ScratchRegister);
    addPtr(imm, ScratchRegister);
    storePtr(ScratchRegister, dest);
}

void
MacroAssemblerMIPS64Compat::branchDouble(DoubleCondition cond, FloatRegister lhs,
                                       FloatRegister rhs, Label* label)
{
    ma_bc1d(lhs, rhs, label, cond);
}

void
MacroAssemblerMIPS64Compat::branchFloat(DoubleCondition cond, FloatRegister lhs,
                                      FloatRegister rhs, Label* label)
{
    ma_bc1s(lhs, rhs, label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestGCThing(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET), label,
         (cond == Equal) ? AboveOrEqual : Below);
}
void
MacroAssemblerMIPS64Compat::branchTestGCThing(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_LOWER_INCL_TAG_OF_GCTHING_SET), label,
         (cond == Equal) ? AboveOrEqual : Below);
}

void
MacroAssemblerMIPS64Compat::branchTestPrimitive(Condition cond, const ValueOperand& value,
                                              Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestPrimitive(cond, SecondScratchReg, label);
}
void
MacroAssemblerMIPS64Compat::branchTestPrimitive(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_UPPER_EXCL_TAG_OF_PRIMITIVE_SET), label,
         (cond == Equal) ? Below : AboveOrEqual);
}

void
MacroAssemblerMIPS64Compat::branchTestInt32(Condition cond, const ValueOperand& value, Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    splitTag(value, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_INT32), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestInt32(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_INT32), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestInt32(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_INT32), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestInt32(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_INT32), label, cond);
}

void
MacroAssemblerMIPS64Compat:: branchTestBoolean(Condition cond, const ValueOperand& value,
                                             Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    splitTag(value, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_BOOLEAN), label, cond);
}

void
MacroAssemblerMIPS64Compat:: branchTestBoolean(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_BOOLEAN), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestBoolean(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_BOOLEAN), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestDouble(Condition cond, const ValueOperand& value, Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    branchTestDouble(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestDouble(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == NotEqual);
    Condition actual = (cond == Equal) ? BelowOrEqual : Above;
    ma_b(tag, ImmTag(JSVAL_TAG_MAX_DOUBLE), label, actual);
}

void
MacroAssemblerMIPS64Compat::branchTestDouble(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    branchTestDouble(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestDouble(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    branchTestDouble(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestNull(Condition cond, const ValueOperand& value, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_NULL), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestNull(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_NULL), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestNull(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_NULL), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestNull(Condition cond, const Address& address, Label* label) {
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_NULL), label, cond);
}

void
MacroAssemblerMIPS64Compat::testNullSet(Condition cond, const ValueOperand& value, Register dest)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    ma_cmp_set(dest, SecondScratchReg, ImmTag(JSVAL_TAG_NULL), cond);
}

void
MacroAssemblerMIPS64Compat::branchTestObject(Condition cond, const ValueOperand& value, Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestObject(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestObject(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_OBJECT), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestObject(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_OBJECT), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestObject(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_OBJECT), label, cond);
}

void
MacroAssemblerMIPS64Compat::testObjectSet(Condition cond, const ValueOperand& value, Register dest)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    ma_cmp_set(dest, SecondScratchReg, ImmTag(JSVAL_TAG_OBJECT), cond);
}

void
MacroAssemblerMIPS64Compat::branchTestString(Condition cond, const ValueOperand& value, Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestString(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestString(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_STRING), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestString(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_STRING), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestSymbol(Condition cond, const ValueOperand& value, Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestSymbol(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestSymbol(Condition cond, const Register& tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_SYMBOL), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestSymbol(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_SYMBOL), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestUndefined(Condition cond, const ValueOperand& value,
                                              Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_UNDEFINED), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestUndefined(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_UNDEFINED), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestUndefined(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_UNDEFINED), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestUndefined(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_UNDEFINED), label, cond);
}

void
MacroAssemblerMIPS64Compat::testUndefinedSet(Condition cond, const ValueOperand& value, Register dest)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    splitTag(value, SecondScratchReg);
    ma_cmp_set(dest, SecondScratchReg, ImmTag(JSVAL_TAG_UNDEFINED), cond);
}

void
MacroAssemblerMIPS64Compat::branchTestNumber(Condition cond, const ValueOperand& value, Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestNumber(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestNumber(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_UPPER_INCL_TAG_OF_NUMBER_SET), label,
         cond == Equal ? BelowOrEqual : Above);
}

void
MacroAssemblerMIPS64Compat::branchTestMagic(Condition cond, const ValueOperand& value, Label* label)
{
    splitTag(value, SecondScratchReg);
    branchTestMagic(cond, SecondScratchReg, label);
}

void
MacroAssemblerMIPS64Compat::branchTestMagic(Condition cond, Register tag, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    ma_b(tag, ImmTag(JSVAL_TAG_MAGIC), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestMagic(Condition cond, const Address& address, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(address, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_MAGIC), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestMagic(Condition cond, const BaseIndex& src, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    extractTag(src, SecondScratchReg);
    ma_b(SecondScratchReg, ImmTag(JSVAL_TAG_MAGIC), label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestValue(Condition cond, const ValueOperand& value,
                                          const Value& v, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    moveValue(v, ScratchRegister);
    ma_b(value.valueReg(), ScratchRegister, label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestValue(Condition cond, const Address& valaddr,
                                          const ValueOperand& value, Label* label)
{
    MOZ_ASSERT(cond == Equal || cond == NotEqual);
    loadPtr(Address(valaddr.base, valaddr.offset), ScratchRegister);
    ma_b(value.valueReg(), ScratchRegister, label, cond);
}

// unboxing code
void
MacroAssemblerMIPS64Compat::unboxNonDouble(const ValueOperand& operand, Register dest)
{
    ma_dext(dest, operand.valueReg(), Imm32(0), Imm32(JSVAL_TAG_SHIFT));
}

void
MacroAssemblerMIPS64Compat::unboxNonDouble(const Address& src, Register dest)
{
    loadPtr(Address(src.base, src.offset), dest);
    ma_dext(dest, dest, Imm32(0), Imm32(JSVAL_TAG_SHIFT));
}

void
MacroAssemblerMIPS64Compat::unboxNonDouble(const BaseIndex& src, Register dest)
{
    computeScaledAddress(src, SecondScratchReg);
    loadPtr(Address(SecondScratchReg, src.offset), dest);
    ma_dext(dest, dest, Imm32(0), Imm32(JSVAL_TAG_SHIFT));
}

void
MacroAssemblerMIPS64Compat::unboxInt32(const ValueOperand& operand, Register dest)
{
    ma_dsll(dest, operand.valueReg(), Imm32(32));
    ma_dsra(dest, dest, Imm32(32));
}

void
MacroAssemblerMIPS64Compat::unboxInt32(const Operand& operand, Register dest)
{
    switch(operand.getTag()) {
    case Operand::REG:
        ma_dsll(dest, operand.toReg(), Imm32(32));
        ma_dsra(dest, dest, Imm32(32));
        break;
    case Operand::MEM:
        unboxInt32(operand.toAddress(), dest);
        break;
    case Operand::FREG:
    default:
        MOZ_CRASH("unexpected operand kind");
        break;
    }
}

void
MacroAssemblerMIPS64Compat::unboxInt32(const Address& src, Register dest)
{
    load32(Address(src.base, src.offset), dest);
}

void
MacroAssemblerMIPS64Compat::unboxInt32(const BaseIndex& src, Register dest)
{
    computeScaledAddress(src, SecondScratchReg);
    load32(Address(SecondScratchReg, src.offset), dest);
}

void
MacroAssemblerMIPS64Compat::unboxBoolean(const ValueOperand& operand, Register dest)
{
    ma_dext(dest, operand.valueReg(), Imm32(0), Imm32(32));
}

void
MacroAssemblerMIPS64Compat::unboxBoolean(const Operand& operand, Register dest)
{
    switch(operand.getTag()) {
    case Operand::REG:
        ma_dext(dest, operand.toReg(), Imm32(0), Imm32(32));
        break;
    case Operand::MEM:
        unboxBoolean(operand.toAddress(), dest);
        break;
    case Operand::FREG:
    default:
        MOZ_CRASH("unexpected operand kind");
        break;
    }
}

void
MacroAssemblerMIPS64Compat::unboxBoolean(const Address& src, Register dest)
{
    ma_load(dest, Address(src.base, src.offset), SizeWord, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::unboxBoolean(const BaseIndex& src, Register dest)
{
    computeScaledAddress(src, SecondScratchReg);
    ma_load(dest, Address(SecondScratchReg, src.offset), SizeWord, ZeroExtend);
}

void
MacroAssemblerMIPS64Compat::unboxDouble(const ValueOperand& operand, FloatRegister dest)
{
    as_dmtc1(operand.valueReg(), dest);
}

void
MacroAssemblerMIPS64Compat::unboxDouble(const Address& src, FloatRegister dest)
{
    ma_ld(dest, Address(src.base, src.offset));
}

void
MacroAssemblerMIPS64Compat::unboxString(const ValueOperand& operand, Register dest)
{
    unboxNonDouble(operand, dest);
}

void
MacroAssemblerMIPS64Compat::unboxString(const Operand& operand, Register dest)
{
    switch(operand.getTag()) {
    case Operand::REG:
        ma_dext(dest, operand.toReg(), Imm32(0), Imm32(JSVAL_TAG_SHIFT));
        break;
    case Operand::MEM:
        unboxNonDouble(operand.toAddress(), dest);
        break;
    case Operand::FREG:
    default:
        MOZ_CRASH("unexpected operand kind");
        break;
    }
}

void
MacroAssemblerMIPS64Compat::unboxString(const Address& src, Register dest)
{
    unboxNonDouble(src, dest);
}

void
MacroAssemblerMIPS64Compat::unboxSymbol(const Operand& operand, Register dest)
{
    switch(operand.getTag()) {
    case Operand::REG:
        ma_dext(dest, operand.toReg(), Imm32(0), Imm32(JSVAL_TAG_SHIFT));
        break;
    case Operand::MEM:
        unboxNonDouble(operand.toAddress(), dest);
        break;
    case Operand::FREG:
    default:
        MOZ_CRASH("unexpected operand kind");
        break;
    }
}

void
MacroAssemblerMIPS64Compat::unboxSymbol(const Address& src, Register dest)
{
    unboxNonDouble(src, dest);
}

void
MacroAssemblerMIPS64Compat::unboxObject(const ValueOperand& src, Register dest)
{
    unboxNonDouble(src, dest);
}

void
MacroAssemblerMIPS64Compat::unboxObject(const Operand& src, Register dest)
{
    switch(src.getTag()) {
    case Operand::REG:
        ma_dext(dest, src.toReg(), Imm32(0), Imm32(JSVAL_TAG_SHIFT));
        break;
    case Operand::MEM:
        unboxNonDouble(src.toAddress(), dest);
        break;
    case Operand::FREG:
    default:
        MOZ_CRASH("unexpected operand kind");
        break;
    }
}

void
MacroAssemblerMIPS64Compat::unboxObject(const Address& src, Register dest)
{
    unboxNonDouble(src, dest);
}

void
MacroAssemblerMIPS64Compat::unboxValue(const ValueOperand& src, AnyRegister dest)
{
    if (dest.isFloat()) {
        Label notInt32, end;
        branchTestInt32(Assembler::NotEqual, src, &notInt32);
        convertInt32ToDouble(src.valueReg(), dest.fpu());
        ma_b(&end, ShortJump);
        bind(&notInt32);
        unboxDouble(src, dest.fpu());
        bind(&end);
    } else {
        unboxNonDouble(src, dest.gpr());
    }
}

void
MacroAssemblerMIPS64Compat::unboxPrivate(const ValueOperand& src, Register dest)
{
    ma_dsrl(dest, src.valueReg(), Imm32(1));
}

void
MacroAssemblerMIPS64Compat::boxDouble(FloatRegister src, const ValueOperand& dest)
{
    as_dmfc1(dest.valueReg(), src);
}

void
MacroAssemblerMIPS64Compat::boxNonDouble(JSValueType type, Register src,
                                       const ValueOperand& dest)
{
    MOZ_ASSERT(src != dest.valueReg());
    boxValue(type, src, dest.valueReg());
}

void
MacroAssemblerMIPS64Compat::boolValueToDouble(const ValueOperand& operand, FloatRegister dest)
{
    convertBoolToInt32(ScratchRegister, operand.valueReg());
    convertInt32ToDouble(ScratchRegister, dest);
}

void
MacroAssemblerMIPS64Compat::int32ValueToDouble(const ValueOperand& operand,
                                             FloatRegister dest)
{
    convertInt32ToDouble(operand.valueReg(), dest);
}

void
MacroAssemblerMIPS64Compat::boolValueToFloat32(const ValueOperand& operand,
                                             FloatRegister dest)
{

    convertBoolToInt32(ScratchRegister, operand.valueReg());
    convertInt32ToFloat32(ScratchRegister, dest);
}

void
MacroAssemblerMIPS64Compat::int32ValueToFloat32(const ValueOperand& operand,
                                              FloatRegister dest)
{
    convertInt32ToFloat32(operand.valueReg(), dest);
}

void
MacroAssemblerMIPS64Compat::loadConstantFloat32(float f, FloatRegister dest)
{
    ma_lis(dest, f);
}

void
MacroAssemblerMIPS64Compat::loadInt32OrDouble(const Address& src, FloatRegister dest)
{
    Label notInt32, end;
    // If it's an int, convert it to double.
    loadPtr(Address(src.base, src.offset), ScratchRegister);
    ma_dsrl(SecondScratchReg, ScratchRegister, Imm32(JSVAL_TAG_SHIFT));
    branchTestInt32(Assembler::NotEqual, SecondScratchReg, &notInt32);
    loadPtr(Address(src.base, src.offset), SecondScratchReg);
    convertInt32ToDouble(SecondScratchReg, dest);
    ma_b(&end, ShortJump);

    // Not an int, just load as double.
    bind(&notInt32);
    ma_ld(dest, src);
    bind(&end);
}

void
MacroAssemblerMIPS64Compat::loadInt32OrDouble(const BaseIndex& addr, FloatRegister dest)
{
    Label notInt32, end;

    // If it's an int, convert it to double.
    computeScaledAddress(addr, SecondScratchReg);
    // Since we only have one scratch, we need to stomp over it with the tag.
    loadPtr(Address(SecondScratchReg, 0), ScratchRegister);
    ma_dsrl(SecondScratchReg, ScratchRegister, Imm32(JSVAL_TAG_SHIFT));
    branchTestInt32(Assembler::NotEqual, SecondScratchReg, &notInt32);

    computeScaledAddress(addr, SecondScratchReg);
    loadPtr(Address(SecondScratchReg, 0), SecondScratchReg);
    convertInt32ToDouble(SecondScratchReg, dest);
    ma_b(&end, ShortJump);

    // Not an int, just load as double.
    bind(&notInt32);
    // First, recompute the offset that had been stored in the scratch register
    // since the scratch register was overwritten loading in the type.
    computeScaledAddress(addr, SecondScratchReg);
    loadDouble(Address(SecondScratchReg, 0), dest);
    bind(&end);
}

void
MacroAssemblerMIPS64Compat::loadConstantDouble(double dp, FloatRegister dest)
{
    ma_lid(dest, dp);
}

void
MacroAssemblerMIPS64Compat::branchTestInt32Truthy(bool b, const ValueOperand& value, Label* label)
{
    ma_dext(ScratchRegister, value.valueReg(), Imm32(0), Imm32(32));
    ma_b(ScratchRegister, ScratchRegister, label, b ? NonZero : Zero);
}

void
MacroAssemblerMIPS64Compat::branchTestStringTruthy(bool b, const ValueOperand& value, Label* label)
{
    unboxString(value, SecondScratchReg);
    load32(Address(SecondScratchReg, JSString::offsetOfLength()), SecondScratchReg);
    ma_b(SecondScratchReg, Imm32(0), label, b ? NotEqual : Equal);
}

void
MacroAssemblerMIPS64Compat::branchTestDoubleTruthy(bool b, FloatRegister value, Label* label)
{
    ma_lid(ScratchDoubleReg, 0.0);
    DoubleCondition cond = b ? DoubleNotEqual : DoubleEqualOrUnordered;
    ma_bc1d(value, ScratchDoubleReg, label, cond);
}

void
MacroAssemblerMIPS64Compat::branchTestBooleanTruthy(bool b, const ValueOperand& operand,
                                                  Label* label)
{
    unboxBoolean(operand, SecondScratchReg);
    ma_b(SecondScratchReg, SecondScratchReg, label, b ? NonZero : Zero);
}

Register
MacroAssemblerMIPS64Compat::extractObject(const Address& address, Register scratch)
{
    loadPtr(Address(address.base, address.offset), scratch);
    ma_dext(scratch, scratch, Imm32(0), Imm32(JSVAL_TAG_SHIFT));
    return scratch;
}

Register
MacroAssemblerMIPS64Compat::extractTag(const Address& address, Register scratch)
{
    loadPtr(Address(address.base, address.offset), scratch);
    ma_dext(scratch, scratch, Imm32(JSVAL_TAG_SHIFT), Imm32(64 - JSVAL_TAG_SHIFT));
    return scratch;
}

Register
MacroAssemblerMIPS64Compat::extractTag(const BaseIndex& address, Register scratch)
{
    computeScaledAddress(address, scratch);
    return extractTag(Address(scratch, address.offset), scratch);
}

template <typename T>
void
MacroAssemblerMIPS64Compat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const T& dest,
                                            MIRType slotType)
{
    if (valueType == MIRType_Double) {
        storeDouble(value.reg().typedReg().fpu(), dest);
        return;
    }

    // For known integers and booleans, we can just store the unboxed value if
    // the slot has the same type.
    if ((valueType == MIRType_Int32 || valueType == MIRType_Boolean) && slotType == valueType) {
        if (value.constant()) {
            Value val = value.value();
            if (valueType == MIRType_Int32)
                store32(Imm32(val.toInt32()), dest);
            else
                store32(Imm32(val.toBoolean() ? 1 : 0), dest);
        } else {
            store32(value.reg().typedReg().gpr(), dest);
        }
        return;
    }

    if (value.constant())
        storeValue(value.value(), dest);
    else
        storeValue(ValueTypeFromMIRType(valueType), value.reg().typedReg().gpr(), dest);
}

template void
MacroAssemblerMIPS64Compat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const Address& dest,
                                            MIRType slotType);

template void
MacroAssemblerMIPS64Compat::storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const BaseIndex& dest,
                                            MIRType slotType);

void
MacroAssemblerMIPS64Compat::moveValue(const Value& val, Register dest)
{
    jsval_layout jv = JSVAL_TO_IMPL(val);
    writeDataRelocation(val);
    movWithPatch(ImmWord(jv.asBits), dest);
}

void
MacroAssemblerMIPS64Compat::moveValue(const Value& val, const ValueOperand& dest)
{
    moveValue(val, dest.valueReg());
}

/* There are 3 paths trough backedge jump. They are listed here in the order
 * in which instructions are executed.
 *  - The short jump is simple:
 *     b offset            # Jumps directly to target.
 *     lui at, addr1_hh    # In delay slot. Don't care about 'at' here.
 *
 *  - The long jump to loop header:
 *      b label1
 *      lui at, addr1_hh   # In delay slot. We use the value in 'at' later.
 *    label1:
 *      ori at, addr1_hl
 *      dsll at, at, 16
 *      ori at, addr1_lh
 *      dsll at, at, 16
 *      ori at, addr1_ll
 *      jr at
 *      lui at, addr2_hh   # In delay slot. Don't care about 'at' here.
 *
 *  - The long jump to interrupt loop:
 *      b label2
 *      ...
 *      jr at
 *    label2:
 *      lui at, addr2_hh   # In delay slot. Don't care about 'at' here.
 *      ori at, addr2_hl
 *      dsll at, at, 16
 *      ori at, addr2_lh
 *      dsll at, at, 16
 *      ori at, addr2_ll
 *      jr at
 *      nop                # In delay slot.
 *
 * The backedge is done this way to avoid patching lui+ori pair while it is
 * being executed. Look also at jit::PatchBackedge().
 */
CodeOffsetJump
MacroAssemblerMIPS64Compat::backedgeJump(RepatchLabel* label)
{
    // Only one branch per label.
    MOZ_ASSERT(!label->used());
    uint32_t dest = label->bound() ? label->offset() : LabelBase::INVALID_OFFSET;
    BufferOffset bo = nextOffset();
    label->use(bo.getOffset());

    // Backedges are short jumps when bound, but can become long when patched.
    m_buffer.ensureSpace(16 * sizeof(uint32_t));
    if (label->bound()) {
        int32_t offset = label->offset() - bo.getOffset();
        MOZ_ASSERT(BOffImm16::IsInRange(offset));
        as_b(BOffImm16(offset));
    } else {
        // Jump to "label1" by default to jump to the loop header.
        as_b(BOffImm16(2 * sizeof(uint32_t)));
    }
    // No need for nop here. We can safely put next instruction in delay slot.
    ma_liPatchable(ScratchRegister, ImmWord(dest));
    MOZ_ASSERT(nextOffset().getOffset() - bo.getOffset() == 7 * sizeof(uint32_t));
    as_jr(ScratchRegister);
    // No need for nop here. We can safely put next instruction in delay slot.
    ma_liPatchable(ScratchRegister, ImmWord(dest));
    as_jr(ScratchRegister);
    as_nop();
    MOZ_ASSERT(nextOffset().getOffset() - bo.getOffset() == 16 * sizeof(uint32_t));
    return CodeOffsetJump(bo.getOffset());
}

CodeOffsetJump
MacroAssemblerMIPS64Compat::jumpWithPatch(RepatchLabel* label)
{
    // Only one branch per label.
    MOZ_ASSERT(!label->used());
    uint32_t dest = label->bound() ? label->offset() : LabelBase::INVALID_OFFSET;

    BufferOffset bo = nextOffset();
    label->use(bo.getOffset());
    addLongJump(bo);
    ma_liPatchable(ScratchRegister, ImmWord(dest));
    as_jr(ScratchRegister);
    as_nop();
    return CodeOffsetJump(bo.getOffset());
}

/////////////////////////////////////////////////////////////////
// X86/X64-common/ARM/MIPS interface.
/////////////////////////////////////////////////////////////////
void
MacroAssemblerMIPS64Compat::storeValue(ValueOperand val, Operand dst)
{
    storeValue(val, Address(Register::FromCode(dst.base()), dst.disp()));
}

void
MacroAssemblerMIPS64Compat::storeValue(ValueOperand val, const BaseIndex& dest)
{
    computeScaledAddress(dest, SecondScratchReg);
    storeValue(val, Address(SecondScratchReg, dest.offset));
}

void
MacroAssemblerMIPS64Compat::storeValue(JSValueType type, Register reg, BaseIndex dest)
{
    computeScaledAddress(dest, ScratchRegister);

    int32_t offset = dest.offset;
    if (!Imm16::IsInSignedRange(offset)) {
        ma_li(SecondScratchReg, Imm32(offset));
        as_daddu(ScratchRegister, ScratchRegister, SecondScratchReg);
        offset = 0;
    }

    storeValue(type, reg, Address(ScratchRegister, offset));
}

void
MacroAssemblerMIPS64Compat::storeValue(ValueOperand val, const Address& dest)
{
    storePtr(val.valueReg(), Address(dest.base, dest.offset));
}

void
MacroAssemblerMIPS64Compat::storeValue(JSValueType type, Register reg, Address dest)
{
    MOZ_ASSERT(dest.base != SecondScratchReg);

    ma_li(SecondScratchReg, ImmTag(JSVAL_TYPE_TO_TAG(type)));
    ma_dsll(SecondScratchReg, SecondScratchReg, Imm32(JSVAL_TAG_SHIFT));
    ma_dins(SecondScratchReg, reg, Imm32(0), Imm32(JSVAL_TAG_SHIFT));
    storePtr(SecondScratchReg, Address(dest.base, dest.offset));
}

void
MacroAssemblerMIPS64Compat::storeValue(const Value& val, Address dest)
{
    jsval_layout jv = JSVAL_TO_IMPL(val);
    if (val.isMarkable()) {
        writeDataRelocation(val);
        movWithPatch(ImmWord(jv.asBits), SecondScratchReg);
    } else {
        ma_li(SecondScratchReg, ImmWord(jv.asBits));
    }
    storePtr(SecondScratchReg, Address(dest.base, dest.offset));
}

void
MacroAssemblerMIPS64Compat::storeValue(const Value& val, BaseIndex dest)
{
    computeScaledAddress(dest, ScratchRegister);

    int32_t offset = dest.offset;
    if (!Imm16::IsInSignedRange(offset)) {
        ma_li(SecondScratchReg, Imm32(offset));
        as_daddu(ScratchRegister, ScratchRegister, SecondScratchReg);
        offset = 0;
    }
    storeValue(val, Address(ScratchRegister, offset));
}

void
MacroAssemblerMIPS64Compat::loadValue(const BaseIndex& addr, ValueOperand val)
{
    computeScaledAddress(addr, SecondScratchReg);
    loadValue(Address(SecondScratchReg, addr.offset), val);
}

void
MacroAssemblerMIPS64Compat::loadValue(Address src, ValueOperand val)
{
    loadPtr(Address(src.base, src.offset), val.valueReg());
}

void
MacroAssemblerMIPS64Compat::tagValue(JSValueType type, Register payload, ValueOperand dest)
{
    MOZ_ASSERT(dest.valueReg() != ScratchRegister);
    if (payload != dest.valueReg())
      ma_move(dest.valueReg(), payload);
    ma_li(ScratchRegister, ImmTag(JSVAL_TYPE_TO_TAG(type)));
    ma_dins(dest.valueReg(), ScratchRegister, Imm32(JSVAL_TAG_SHIFT), Imm32(64 - JSVAL_TAG_SHIFT));
}

void
MacroAssemblerMIPS64Compat::pushValue(ValueOperand val)
{
    ma_dsubu(StackPointer, StackPointer, Imm32(sizeof(Value)));
    storeValue(val, Address(StackPointer, 0));
}

void
MacroAssemblerMIPS64Compat::pushValue(const Address& addr)
{
    // Load value before allocate stack, addr.base may be is sp.
    loadPtr(Address(addr.base, addr.offset), ScratchRegister);
    ma_dsubu(StackPointer, StackPointer, Imm32(sizeof(Value)));
    storePtr(ScratchRegister, Address(StackPointer, 0));
}

void
MacroAssemblerMIPS64Compat::popValue(ValueOperand val)
{
    as_ld(val.valueReg(), StackPointer, 0);
    as_daddiu(StackPointer, StackPointer, sizeof(Value));
}

void
MacroAssemblerMIPS64::ma_callJit(const Register r)
{
    // This is a MIPS hack to push return address during jalr delay slot.
    as_daddiu(StackPointer, StackPointer, (int32_t)-sizeof(intptr_t));
    as_jalr(r);
    as_sd(ra, StackPointer, 0);
}

void
MacroAssemblerMIPS64::ma_callJit(Label* label)
{
    // This is a MIPS hack to push return address during jalr delay slot.
    as_daddiu(StackPointer, StackPointer, (int32_t)-sizeof(intptr_t));
    ma_bal(label, DontFillDelaySlot);
    as_sd(ra, StackPointer, 0);
}

void
MacroAssemblerMIPS64::ma_call(ImmPtr dest)
{
    ma_liPatchable(CallReg, dest);
    as_jalr(CallReg);
    as_nop();
}

void
MacroAssemblerMIPS64::ma_jump(ImmPtr dest)
{
    ma_liPatchable(ScratchRegister, dest);
    as_jr(ScratchRegister);
    as_nop();
}

void
MacroAssemblerMIPS64Compat::breakpoint()
{
    as_break(0);
}

void
MacroAssemblerMIPS64Compat::ensureDouble(const ValueOperand& source, FloatRegister dest,
                                       Label* failure)
{
    Label isDouble, done;
    Register tag = splitTagForTest(source);
    branchTestDouble(Assembler::Equal, tag, &isDouble);
    branchTestInt32(Assembler::NotEqual, tag, failure);

    unboxInt32(source, ScratchRegister);
    convertInt32ToDouble(ScratchRegister, dest);
    jump(&done);

    bind(&isDouble);
    unboxDouble(source, dest);

    bind(&done);
}

void
MacroAssemblerMIPS64Compat::cmpPtrSet(Assembler::Condition cond, Address lhs, ImmPtr rhs,
                                        Register dest)
{
    loadPtr(lhs, ScratchRegister);
    movePtr(rhs, SecondScratchReg);
    cmpPtrSet(cond, ScratchRegister, SecondScratchReg, dest);
}

void
MacroAssemblerMIPS64Compat::cmpPtrSet(Assembler::Condition cond, Register lhs, Address rhs,
                                        Register dest)
{
    loadPtr(rhs, ScratchRegister);
    cmpPtrSet(cond, lhs, ScratchRegister, dest);
}

void
MacroAssemblerMIPS64Compat::cmp32Set(Assembler::Condition cond, Register lhs, Address rhs,
                                        Register dest)
{
    load32(rhs, ScratchRegister);
    cmp32Set(cond, lhs, ScratchRegister, dest);
}

void
MacroAssemblerMIPS64Compat::setupABICall(uint32_t args)
{
    MOZ_ASSERT(!inCall_);
    inCall_ = true;
    args_ = args;
    passedArgs_ = 0;
    passedArgTypes_ = 0;

    usedArgSlots_ = 0;
    firstArgType = MoveOp::GENERAL;
}

void
MacroAssemblerMIPS64Compat::setupAlignedABICall(uint32_t args)
{
    setupABICall(args);

    dynamicAlignment_ = false;
}

void
MacroAssemblerMIPS64Compat::setupUnalignedABICall(uint32_t args, Register scratch)
{
    setupABICall(args);
    dynamicAlignment_ = true;

    ma_move(scratch, StackPointer);

    // Force sp to be aligned
    ma_dsubu(StackPointer, StackPointer, Imm32(sizeof(uintptr_t)));
    ma_and(StackPointer, StackPointer, Imm32(~(ABIStackAlignment - 1)));
    as_sd(scratch, StackPointer, 0);
}

void
MacroAssemblerMIPS64Compat::passABIArg(const MoveOperand& from, MoveOp::Type type)
{
    ++passedArgs_;
    if (!enoughMemory_)
        return;
    switch (type) {
      case MoveOp::FLOAT32:
      case MoveOp::DOUBLE: {
        FloatRegister destFReg;
        if (!usedArgSlots_)
          firstArgType = type;
        if (GetFloatArgReg(usedArgSlots_, &destFReg)) {
            if (from.isFloatReg() && from.floatReg() == destFReg) {
                // Nothing to do; the value is in the right register already
            } else {
                enoughMemory_ = moveResolver_.addMove(from, MoveOperand(destFReg), type);
            }
        } else {
            uint32_t disp = GetArgStackDisp(usedArgSlots_);
            enoughMemory_ = moveResolver_.addMove(from, MoveOperand(sp, disp), type);
        }
        usedArgSlots_++;
        passedArgTypes_ = (passedArgTypes_ << ArgType_Shift) |
                                ((MoveOp::FLOAT32 == type) ? ArgType_Float32 : ArgType_Double);
        break;
      }
      case MoveOp::GENERAL: {
        Register destReg;
        if (GetIntArgReg(usedArgSlots_, &destReg)) {
            if (from.isGeneralReg() && from.reg() == destReg) {
                // Nothing to do. Value is in the right register already
            } else {
                enoughMemory_ = moveResolver_.addMove(from, MoveOperand(destReg), type);
            }
        } else {
            uint32_t disp = GetArgStackDisp(usedArgSlots_);
            enoughMemory_ = moveResolver_.addMove(from, MoveOperand(sp, disp), type);
        }
        usedArgSlots_++;
        passedArgTypes_ = (passedArgTypes_ << ArgType_Shift) | ArgType_General;
        break;
      }
      default:
        MOZ_CRASH("Unexpected argument type");
    }
}

void
MacroAssemblerMIPS64Compat::passABIArg(Register reg)
{
    passABIArg(MoveOperand(reg), MoveOp::GENERAL);
}

void
MacroAssemblerMIPS64Compat::passABIArg(FloatRegister freg, MoveOp::Type type)
{
    passABIArg(MoveOperand(freg), type);
}

void
MacroAssemblerMIPS64Compat::checkStackAlignment()
{
#ifdef DEBUG
    Label aligned;
    as_andi(ScratchRegister, sp, ABIStackAlignment - 1);
    ma_b(ScratchRegister, zero, &aligned, Equal, ShortJump);
    as_break(BREAK_STACK_UNALIGNED);
    bind(&aligned);
#endif
}

void
MacroAssemblerMIPS64Compat::alignStackPointer()
{
    movePtr(StackPointer, SecondScratchReg);
    subPtr(Imm32(sizeof(uintptr_t)), StackPointer);
    andPtr(Imm32(~(ABIStackAlignment - 1)), StackPointer);
    storePtr(SecondScratchReg, Address(StackPointer, 0));
}

void
MacroAssemblerMIPS64Compat::restoreStackPointer()
{
    loadPtr(Address(StackPointer, 0), StackPointer);
}

void
MacroAssembler::alignFrameForICArguments(AfterICSaveLive& aic)
{
    if (framePushed() % ABIStackAlignment != 0) {
        aic.alignmentPadding = ABIStackAlignment - (framePushed() % ABIStackAlignment);
        reserveStack(aic.alignmentPadding);
    } else {
        aic.alignmentPadding = 0;
    }
    MOZ_ASSERT(framePushed() % ABIStackAlignment == 0);
    checkStackAlignment();
}

void
MacroAssembler::restoreFrameAlignmentForICArguments(AfterICSaveLive& aic)
{
    if (aic.alignmentPadding != 0)
        freeStack(aic.alignmentPadding);
}

void
MacroAssemblerMIPS64Compat::callWithABIPre(uint32_t* stackAdjust, bool callFromAsmJS)
{
    MOZ_ASSERT(inCall_);

    // Reserve place for $ra.
    *stackAdjust = sizeof(intptr_t);

    *stackAdjust += usedArgSlots_ > NumIntArgRegs ?
                    (usedArgSlots_ - NumIntArgRegs) * sizeof(intptr_t) : 0;

    uint32_t alignmentAtPrologue = callFromAsmJS ? sizeof(AsmJSFrame) : 0;

    if (dynamicAlignment_) {
        *stackAdjust += ComputeByteAlignment(*stackAdjust, ABIStackAlignment);
    } else {
        *stackAdjust += ComputeByteAlignment(framePushed_ + alignmentAtPrologue + *stackAdjust,
                                             ABIStackAlignment);
    }

    reserveStack(*stackAdjust);

    // Save $ra because call is going to clobber it. Restore it in
    // callWithABIPost. NOTE: This is needed for calls from BaselineIC.
    // Maybe we can do this differently.
    storePtr(ra, Address(StackPointer, *stackAdjust - sizeof(intptr_t)));

    // Position all arguments.
    {
        enoughMemory_ = enoughMemory_ && moveResolver_.resolve();
        if (!enoughMemory_)
            return;

        MoveEmitter emitter(asMasm());
        emitter.emit(moveResolver_);
        emitter.finish();
    }

    checkStackAlignment();
}

void
MacroAssemblerMIPS64Compat::callWithABIPost(uint32_t stackAdjust, MoveOp::Type result)
{
    // Restore ra value (as stored in callWithABIPre()).
    loadPtr(Address(StackPointer, stackAdjust - sizeof(intptr_t)), ra);

    if (dynamicAlignment_) {
        // Restore sp value from stack (as stored in setupUnalignedABICall()).
        loadPtr(Address(StackPointer, stackAdjust), StackPointer);
        // Use adjustFrame instead of freeStack because we already restored sp.
        adjustFrame(-stackAdjust);
    } else {
        freeStack(stackAdjust);
    }

    MOZ_ASSERT(inCall_);
    inCall_ = false;
}

#if defined(DEBUG) && defined(JS_MIPS64_SIMULATOR)
static void
AssertValidABIFunctionType(uint32_t passedArgTypes)
{
    switch (passedArgTypes) {
      case Args_General0:
      case Args_General1:
      case Args_General2:
      case Args_General3:
      case Args_General4:
      case Args_General5:
      case Args_General6:
      case Args_General7:
      case Args_General8:
      case Args_Double_None:
      case Args_Int_Double:
      case Args_Float32_Float32:
      case Args_Double_Double:
      case Args_Double_Int:
      case Args_Double_DoubleInt:
      case Args_Double_DoubleDouble:
      case Args_Double_IntDouble:
      case Args_Int_IntDouble:
      case Args_Double_DoubleDoubleDouble:
      case Args_Double_DoubleDoubleDoubleDouble:
        break;
      default:
        MOZ_CRASH("Unexpected type");
    }
}
#endif

void
MacroAssemblerMIPS64Compat::callWithABI(void* fun, MoveOp::Type result)
{
#ifdef JS_MIPS64_SIMULATOR
    MOZ_ASSERT(passedArgs_ <= 15);
    passedArgTypes_ <<= ArgType_Shift;
    switch (result) {
      case MoveOp::GENERAL: passedArgTypes_ |= ArgType_General; break;
      case MoveOp::DOUBLE:  passedArgTypes_ |= ArgType_Double;  break;
      case MoveOp::FLOAT32: passedArgTypes_ |= ArgType_Float32; break;
      default: MOZ_CRASH("Invalid return type");
    }
#ifdef DEBUG
    AssertValidABIFunctionType(passedArgTypes_);
#endif
    ABIFunctionType type = ABIFunctionType(passedArgTypes_);
    fun = Simulator::RedirectNativeFunction(fun, type);
#endif

    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    ma_call(ImmPtr(fun));
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerMIPS64Compat::callWithABI(AsmJSImmPtr imm, MoveOp::Type result)
{
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust, /* callFromAsmJS = */ true);
    call(imm);
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerMIPS64Compat::callWithABI(const Address& fun, MoveOp::Type result)
{
    // Load the callee in t9, no instruction between the lw and call
    // should clobber it. Note that we can't use fun.base because it may
    // be one of the IntArg registers clobbered before the call.
    loadPtr(Address(fun.base, fun.offset), t9);
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(t9);
    callWithABIPost(stackAdjust, result);

}

void
MacroAssemblerMIPS64Compat::callWithABI(Register fun, MoveOp::Type result)
{
    // Load the callee in t9, as above.
    ma_move(t9, fun);
    uint32_t stackAdjust;
    callWithABIPre(&stackAdjust);
    call(t9);
    callWithABIPost(stackAdjust, result);
}

void
MacroAssemblerMIPS64Compat::handleFailureWithHandlerTail(void* handler)
{
    // Reserve space for exception information.
    int size = (sizeof(ResumeFromException) + ABIStackAlignment) & ~(ABIStackAlignment - 1);
    ma_dsubu(StackPointer, StackPointer, Imm32(size));
    ma_move(a0, StackPointer); // Use a0 since it is a first function argument

    // Call the handler.
    setupUnalignedABICall(1, a1);
    passABIArg(a0);
    callWithABI(handler);

    Label entryFrame;
    Label catch_;
    Label finally;
    Label return_;
    Label bailout;

    // Already clobbered a0, so use it...
    load32(Address(StackPointer, offsetof(ResumeFromException, kind)), a0);
    branch32(Assembler::Equal, a0, Imm32(ResumeFromException::RESUME_ENTRY_FRAME), &entryFrame);
    branch32(Assembler::Equal, a0, Imm32(ResumeFromException::RESUME_CATCH), &catch_);
    branch32(Assembler::Equal, a0, Imm32(ResumeFromException::RESUME_FINALLY), &finally);
    branch32(Assembler::Equal, a0, Imm32(ResumeFromException::RESUME_FORCED_RETURN), &return_);
    branch32(Assembler::Equal, a0, Imm32(ResumeFromException::RESUME_BAILOUT), &bailout);

    breakpoint(); // Invalid kind.

    // No exception handler. Load the error value, load the new stack pointer
    // and return from the entry frame.
    bind(&entryFrame);
    moveValue(MagicValue(JS_ION_ERROR), JSReturnOperand);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, stackPointer)), StackPointer);

    // We're going to be returning by the ion calling convention
    ma_pop(ra);
    as_jr(ra);
    as_nop();

    // If we found a catch handler, this must be a baseline frame. Restore
    // state and jump to the catch block.
    bind(&catch_);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, target)), a0);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, framePointer)), BaselineFrameReg);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, stackPointer)), StackPointer);
    jump(a0);

    // If we found a finally block, this must be a baseline frame. Push
    // two values expected by JSOP_RETSUB: BooleanValue(true) and the
    // exception.
    bind(&finally);
    ValueOperand exception = ValueOperand(a1);
    loadValue(Address(sp, offsetof(ResumeFromException, exception)), exception);

    loadPtr(Address(sp, offsetof(ResumeFromException, target)), a0);
    loadPtr(Address(sp, offsetof(ResumeFromException, framePointer)), BaselineFrameReg);
    loadPtr(Address(sp, offsetof(ResumeFromException, stackPointer)), sp);

    pushValue(BooleanValue(true));
    pushValue(exception);
    jump(a0);

    // Only used in debug mode. Return BaselineFrame->returnValue() to the
    // caller.
    bind(&return_);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, framePointer)), BaselineFrameReg);
    loadPtr(Address(StackPointer, offsetof(ResumeFromException, stackPointer)), StackPointer);
    loadValue(Address(BaselineFrameReg, BaselineFrame::reverseOffsetOfReturnValue()),
              JSReturnOperand);
    ma_move(StackPointer, BaselineFrameReg);
    pop(BaselineFrameReg);

    // If profiling is enabled, then update the lastProfilingFrame to refer to caller
    // frame before returning.
    {
        Label skipProfilingInstrumentation;
        // Test if profiler enabled.
        AbsoluteAddress addressOfEnabled(GetJitContext()->runtime->spsProfiler().addressOfEnabled());
        branch32(Assembler::Equal, addressOfEnabled, Imm32(0), &skipProfilingInstrumentation);
        profilerExitFrame();
        bind(&skipProfilingInstrumentation);
    }

    ret();

    // If we are bailing out to baseline to handle an exception, jump to
    // the bailout tail stub.
    bind(&bailout);
    loadPtr(Address(sp, offsetof(ResumeFromException, bailoutInfo)), a2);
    ma_li(ReturnReg, Imm32(BAILOUT_RETURN_OK));
    loadPtr(Address(sp, offsetof(ResumeFromException, target)), a1);
    jump(a1);
}

void
MacroAssemblerMIPS64Compat::atomicEffectOpMIPSr2(int nbytes, AtomicOp op,
            const Register& value, const Register& addr, AllocatableGeneralRegisterSet& regs)
{
    atomicFetchOpMIPSr2(nbytes, false, op, value, addr, InvalidReg, InvalidReg, regs);
}

void
MacroAssemblerMIPS64Compat::atomicFetchOpMIPSr2(int nbytes, bool signExtend, AtomicOp op, const Register& value,
            const Register& addr, const Register& temp, const Register& output, AllocatableGeneralRegisterSet& regs)
{
    Label again;
    Register t0, t1, t2, t3, t4, t5;

    t0 = regs.takeAny();
    t1 = regs.takeAny();
    t2 = regs.takeAny();
    t3 = regs.takeAny();
    t4 = regs.takeAny();
    t5 = temp;

    // The addr maybe is ScratchRegister, so don't use macro
    // instructions depends it here.
    as_sd(t0, StackPointer, (int16_t)-(1 * sizeof(uintptr_t)));
    as_sd(t1, StackPointer, (int16_t)-(2 * sizeof(uintptr_t)));
    as_sd(t2, StackPointer, (int16_t)-(3 * sizeof(uintptr_t)));
    as_sd(t3, StackPointer, (int16_t)-(4 * sizeof(uintptr_t)));
    as_sd(t4, StackPointer, (int16_t)-(5 * sizeof(uintptr_t)));
    if (temp == InvalidReg) {
        t5 = regs.takeAny();
        as_sd(t5, StackPointer, (int16_t)-(6 * sizeof(uintptr_t)));
    }

    as_andi(t0, addr, 3);
    as_dsubu(addr, addr, t0);
    as_sll(t2, t0, 3);
    as_sllv(t1, value, t2);
    ma_li(t3, Imm32(0xffffffffu >> ((4 - nbytes) * 8)));
    as_sllv(t3, t3, t2);
    ma_not(t4, t3);

    bind(&again);

    as_sync(0);

    as_ll(t0, addr, 0);

    if (output != InvalidReg) {
        as_srlv(output, t0, t2);
        if (signExtend) {
            switch (nbytes) {
            case 1:
                as_seb(output, output);
                break;
            case 2:
                as_seh(output, output);
                break;
            case 4:
                as_sll(output, output, 0);
                break;
            default:
                MOZ_CRASH("NYI");
            }
        }
    }

    switch (op) {
    case AtomicFetchAddOp:
        as_addu(t5, t0, t1);
        break;
    case AtomicFetchSubOp:
        as_subu(t5, t0, t1);
        break;
    case AtomicFetchAndOp:
        as_and(t5, t0, t1);
        break;
    case AtomicFetchOrOp:
        as_or(t5, t0, t1);
        break;
    case AtomicFetchXorOp:
        as_xor(t5, t0, t1);
        break;
    default:
        MOZ_CRASH("NYI");
    }

    as_and(t5, t5, t3);
    as_and(t0, t0, t4);
    as_or(t0, t0, t5);

    as_sc(t0, addr, 0);

    ma_b(t0, t0, &again, Zero, ShortJump);

    as_sync(0);

    as_ld(t0, StackPointer, (int16_t)-(1 * sizeof(uintptr_t)));
    as_ld(t1, StackPointer, (int16_t)-(2 * sizeof(uintptr_t)));
    as_ld(t2, StackPointer, (int16_t)-(3 * sizeof(uintptr_t)));
    as_ld(t3, StackPointer, (int16_t)-(4 * sizeof(uintptr_t)));
    as_ld(t4, StackPointer, (int16_t)-(5 * sizeof(uintptr_t)));
    if (temp == InvalidReg)
      as_ld(t5, StackPointer, (int16_t)-(6 * sizeof(uintptr_t)));
}

void
MacroAssemblerMIPS64Compat::atomicEffectOp(int nbytes, AtomicOp op, const Imm32& value, const Address& address)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(address.base);
    ma_li(SecondScratchReg, value);
    computeEffectiveAddress(address, ScratchRegister);
    atomicEffectOpMIPSr2(nbytes, op, SecondScratchReg, ScratchRegister, regs);
}

void
MacroAssemblerMIPS64Compat::atomicEffectOp(int nbytes, AtomicOp op, const Imm32& value, const BaseIndex& address)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(address.base);
    regs.take(address.index);
    ma_li(SecondScratchReg, value);
    computeEffectiveAddress(address, ScratchRegister);
    atomicEffectOpMIPSr2(nbytes, op, SecondScratchReg, ScratchRegister, regs);
}

void
MacroAssemblerMIPS64Compat::atomicEffectOp(int nbytes, AtomicOp op, const Register& value, const Address& address)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(value);
    regs.take(address.base);
    computeEffectiveAddress(address, ScratchRegister);
    atomicEffectOpMIPSr2(nbytes, op, value, ScratchRegister, regs);
}

void
MacroAssemblerMIPS64Compat::atomicEffectOp(int nbytes, AtomicOp op, const Register& value, const BaseIndex& address)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(value);
    regs.take(address.base);
    regs.take(address.index);
    computeEffectiveAddress(address, ScratchRegister);
    atomicEffectOpMIPSr2(nbytes, op, value, ScratchRegister, regs);
}

void
MacroAssemblerMIPS64Compat::atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Imm32& value,
                   const Address& address, Register temp, Register output)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(temp);
    regs.take(output);
    regs.take(address.base);
    ma_li(SecondScratchReg, value);
    computeEffectiveAddress(address, ScratchRegister);
    atomicFetchOpMIPSr2(nbytes, signExtend, op, SecondScratchReg, ScratchRegister, temp, output, regs);
}

void
MacroAssemblerMIPS64Compat::atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Imm32& value,
                   const BaseIndex& address, Register temp, Register output)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(temp);
    regs.take(output);
    regs.take(address.base);
    regs.take(address.index);
    ma_li(SecondScratchReg, value);
    computeEffectiveAddress(address, ScratchRegister);
    atomicFetchOpMIPSr2(nbytes, signExtend, op, SecondScratchReg, ScratchRegister, temp, output, regs);
}

void
MacroAssemblerMIPS64Compat::atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Register& value,
                   const Address& address, Register temp, Register output)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(value);
    regs.take(temp);
    regs.take(output);
    regs.take(address.base);
    computeEffectiveAddress(address, ScratchRegister);
    atomicFetchOpMIPSr2(nbytes, signExtend, op, value, ScratchRegister, temp, output, regs);
}

void
MacroAssemblerMIPS64Compat::atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Register& value,
                   const BaseIndex& address, Register temp, Register output)
{
    AllocatableGeneralRegisterSet regs(GeneralRegisterSet(Registers::VolatileMask));

    regs.take(value);
    regs.take(temp);
    regs.take(output);
    regs.take(address.base);
    regs.take(address.index);
    computeEffectiveAddress(address, ScratchRegister);
    atomicFetchOpMIPSr2(nbytes, signExtend, op, value, ScratchRegister, temp, output, regs);
}

CodeOffsetLabel
MacroAssemblerMIPS64Compat::toggledJump(Label* label)
{
    CodeOffsetLabel ret(nextOffset().getOffset());
    ma_b(label);
    return ret;
}

CodeOffsetLabel
MacroAssemblerMIPS64Compat::toggledCall(JitCode* target, bool enabled)
{
    BufferOffset bo = nextOffset();
    CodeOffsetLabel offset(bo.getOffset());
    addPendingJump(bo, ImmPtr(target->raw()), Relocation::JITCODE);
    ma_liPatchable(ScratchRegister, ImmPtr(target->raw()));
    if (enabled) {
        as_jalr(ScratchRegister);
        as_nop();
    } else {
        as_nop();
        as_nop();
    }
    MOZ_ASSERT(nextOffset().getOffset() - offset.offset() == ToggledCallSize(nullptr));
    return offset;
}

void
MacroAssemblerMIPS64Compat::branchPtrInNurseryRange(Condition cond, Register ptr, Register temp,
                                                  Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);
    MOZ_ASSERT(ptr != temp);
    MOZ_ASSERT(ptr != SecondScratchReg);

    const Nursery& nursery = GetJitContext()->runtime->gcNursery();
    movePtr(ImmWord(-ptrdiff_t(nursery.start())), SecondScratchReg);
    addPtr(ptr, SecondScratchReg);
    branchPtr(cond == Assembler::Equal ? Assembler::Below : Assembler::AboveOrEqual,
              SecondScratchReg, Imm32(nursery.nurserySize()), label);
}

void
MacroAssemblerMIPS64Compat::branchValueIsNurseryObject(Condition cond, ValueOperand value,
                                                     Register temp, Label* label)
{
    MOZ_ASSERT(cond == Assembler::Equal || cond == Assembler::NotEqual);

    // 'Value' representing the start of the nursery tagged as a JSObject
    const Nursery& nursery = GetJitContext()->runtime->gcNursery();
    Value start = ObjectValue(*reinterpret_cast<JSObject *>(nursery.start()));

    movePtr(ImmWord(-ptrdiff_t(start.asRawBits())), SecondScratchReg);
    addPtr(value.valueReg(), SecondScratchReg);
    branchPtr(cond == Assembler::Equal ? Assembler::Below : Assembler::AboveOrEqual,
              SecondScratchReg, Imm32(nursery.nurserySize()), label);
}

void
MacroAssemblerMIPS64Compat::profilerEnterFrame(Register framePtr, Register scratch)
{
    AbsoluteAddress activation(GetJitContext()->runtime->addressOfProfilingActivation());
    loadPtr(activation, scratch);
    storePtr(framePtr, Address(scratch, JitActivation::offsetOfLastProfilingFrame()));
    storePtr(ImmPtr(nullptr), Address(scratch, JitActivation::offsetOfLastProfilingCallSite()));
}

void
MacroAssemblerMIPS64Compat::profilerExitFrame()
{
    branch(GetJitContext()->runtime->jitRuntime()->getProfilerExitFrameTail());
}

MacroAssembler&
MacroAssemblerMIPS64Compat::asMasm()
{
    return *static_cast<MacroAssembler*>(this);
}

const MacroAssembler&
MacroAssemblerMIPS64Compat::asMasm() const
{
    return *static_cast<const MacroAssembler*>(this);
}

// ===============================================================
// Stack manipulation functions.

void
MacroAssembler::PushRegsInMask(LiveRegisterSet set)
{
    int32_t diffF = set.fpus().getPushSizeInBytes();
    int32_t diffG = set.gprs().size() * sizeof(intptr_t);

    reserveStack(diffG);
    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
        diffG -= sizeof(intptr_t);
        storePtr(*iter, Address(StackPointer, diffG));
    }
    MOZ_ASSERT(diffG == 0);

    // Double values have to be aligned. We reserve extra space so that we can
    // start writing from the first aligned location.
    // We reserve a whole extra double so that the buffer has even size.
    ma_and(SecondScratchReg, sp, Imm32(~(ABIStackAlignment - 1)));
    reserveStack(diffF + sizeof(double));

    for (FloatRegisterForwardIterator iter(set.fpus().reduceSetForPush()); iter.more(); iter++) {
        as_sd(*iter, SecondScratchReg, -diffF);
        diffF -= sizeof(double);
    }
    MOZ_ASSERT(diffF == 0);
}

void
MacroAssembler::PopRegsInMaskIgnore(LiveRegisterSet set, LiveRegisterSet ignore)
{
    int32_t diffG = set.gprs().size() * sizeof(intptr_t);
    int32_t diffF = set.fpus().getPushSizeInBytes();
    const int32_t reservedG = diffG;
    const int32_t reservedF = diffF;

    // Read the buffer form the first aligned location.
    ma_daddu(SecondScratchReg, sp, Imm32(reservedF + sizeof(double)));
    ma_and(SecondScratchReg, SecondScratchReg, Imm32(~(ABIStackAlignment - 1)));

    for (FloatRegisterForwardIterator iter(set.fpus().reduceSetForPush()); iter.more(); iter++) {
        if (!ignore.has(*iter))
            // Use assembly l.d because we have alligned the stack.
            as_ld(*iter, SecondScratchReg, -diffF);
        diffF -= sizeof(double);
    }
    freeStack(reservedF + sizeof(double));
    MOZ_ASSERT(diffF == 0);

    for (GeneralRegisterBackwardIterator iter(set.gprs()); iter.more(); iter++) {
        diffG -= sizeof(intptr_t);
        if (!ignore.has(*iter))
            loadPtr(Address(StackPointer, diffG), *iter);
    }
    freeStack(reservedG);
    MOZ_ASSERT(diffG == 0);
}

void
MacroAssembler::Push(Register reg)
{
    ma_push(reg);
    adjustFrame(sizeof(intptr_t));
}

void
MacroAssembler::Push(const Imm32 imm)
{
    ma_li(ScratchRegister, imm);
    ma_push(ScratchRegister);
    adjustFrame(sizeof(intptr_t));
}

void
MacroAssembler::Push(const ImmWord imm)
{
    ma_li(ScratchRegister, imm);
    ma_push(ScratchRegister);
    adjustFrame(sizeof(intptr_t));
}

void
MacroAssembler::Push(const ImmPtr imm)
{
    Push(ImmWord(uintptr_t(imm.value)));
}

void
MacroAssembler::Push(const ImmGCPtr ptr)
{
    ma_li(ScratchRegister, ptr);
    ma_push(ScratchRegister);
    adjustFrame(sizeof(intptr_t));
}

void
MacroAssembler::Push(FloatRegister f)
{
    ma_push(f);
    adjustFrame(sizeof(double));
}

void
MacroAssembler::Pop(Register reg)
{
    ma_pop(reg);
    adjustFrame((int32_t)-sizeof(intptr_t));
}

void
MacroAssembler::Pop(const ValueOperand& val)
{
    popValue(val);
    framePushed_ -= sizeof(Value);
}
