/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef jit_mips64_MacroAssembler_mips64_h
#define jit_mips64_MacroAssembler_mips64_h

#include "jsopcode.h"

#include "jit/AtomicOp.h"
#include "jit/IonCaches.h"
#include "jit/JitFrames.h"
#include "jit/mips64/Assembler-mips64.h"
#include "jit/MoveResolver.h"

namespace js {
namespace jit {

enum LoadStoreSize
{
    SizeByte = 8,
    SizeHalfWord = 16,
    SizeWord = 32,
    SizeDouble = 64
};

enum LoadStoreExtension
{
    ZeroExtend = 0,
    SignExtend = 1
};

enum JumpKind
{
    LongJump = 0,
    ShortJump = 1
};

enum DelaySlotFill
{
    DontFillDelaySlot = 0,
    FillDelaySlot = 1
};

struct ImmShiftedTag : public ImmWord
{
    explicit ImmShiftedTag(JSValueShiftedTag shtag)
      : ImmWord((uintptr_t)shtag)
    { }

    explicit ImmShiftedTag(JSValueType type)
      : ImmWord(uintptr_t(JSValueShiftedTag(JSVAL_TYPE_TO_SHIFTED_TAG(type))))
    { }
};

struct ImmTag : public Imm32
{
    ImmTag(JSValueTag mask)
      : Imm32(int32_t(mask))
    { }
};

static const ValueOperand JSReturnOperand = ValueOperand(JSReturnReg);

static Register CallReg = t9;
static const int defaultShift = 3;
static_assert(1 << defaultShift == sizeof(jsval), "The defaultShift is wrong");

class MacroAssemblerMIPS64 : public Assembler
{
  public:

    void convertBoolToInt32(Register source, Register dest);
    void convertInt32ToDouble(Register src, FloatRegister dest);
    void convertInt32ToDouble(const Address& src, FloatRegister dest);
    void convertInt32ToDouble(const BaseIndex& src, FloatRegister dest);
    void convertUInt32ToDouble(Register src, FloatRegister dest);
    void convertUInt32ToFloat32(Register src, FloatRegister dest);
    void convertDoubleToFloat32(FloatRegister src, FloatRegister dest);
    void branchTruncateDouble(FloatRegister src, Register dest, Label* fail);
    void convertDoubleToInt32(FloatRegister src, Register dest, Label* fail,
                              bool negativeZeroCheck = true);
    void convertFloat32ToInt32(FloatRegister src, Register dest, Label* fail,
                               bool negativeZeroCheck = true);

    void convertFloat32ToDouble(FloatRegister src, FloatRegister dest);
    void branchTruncateFloat32(FloatRegister src, Register dest, Label* fail);
    void convertInt32ToFloat32(Register src, FloatRegister dest);
    void convertInt32ToFloat32(const Address& src, FloatRegister dest);


    void addDouble(FloatRegister src, FloatRegister dest);
    void subDouble(FloatRegister src, FloatRegister dest);
    void mulDouble(FloatRegister src, FloatRegister dest);
    void divDouble(FloatRegister src, FloatRegister dest);

    void negateDouble(FloatRegister reg);
    void inc64(AbsoluteAddress dest);

    void movq(Register rs, Register rd);
    void ma_move(Register rd, Register rs);

    void ma_li(Register dest, ImmGCPtr ptr);
    void ma_li(Register dest, AbsoluteLabel* label);

    void ma_li(Register dest, Imm32 imm);
    void ma_li(Register dest, ImmWord imm);
    void ma_liPatchable(Register dest, ImmPtr imm);
    void ma_liPatchable(Register dest, ImmWord imm);

    // Shift operations
    void ma_sll(Register rd, Register rt, Imm32 shift);
    void ma_dsll(Register rd, Register rt, Imm32 shift);
    void ma_srl(Register rd, Register rt, Imm32 shift);
    void ma_dsrl(Register rd, Register rt, Imm32 shift);
    void ma_sra(Register rd, Register rt, Imm32 shift);
    void ma_dsra(Register rd, Register rt, Imm32 shift);
    void ma_dror(Register rd, Register rt, Imm32 shift);
    void ma_drol(Register rd, Register rt, Imm32 shift);

    void ma_sll(Register rd, Register rt, Register shift);
    void ma_dsll(Register rd, Register rt, Register shift);
    void ma_srl(Register rd, Register rt, Register shift);
    void ma_dsrl(Register rd, Register rt, Register shift);
    void ma_sra(Register rd, Register rt, Register shift);
    void ma_dsra(Register rd, Register rt, Register shift);
    void ma_dror(Register rd, Register rt, Register shift);
    void ma_drol(Register rd, Register rt, Register shift);

    void ma_dins(Register rt, Register rs, Imm32 pos, Imm32 size);
    void ma_dext(Register rt, Register rs, Imm32 pos, Imm32 size);

    // Negate
    void ma_negu(Register rd, Register rs);

    void ma_not(Register rd, Register rs);

    // and
    void ma_and(Register rd, Register rs);
    void ma_and(Register rd, Imm32 imm);
    void ma_and(Register rd, Register rs, Imm32 imm);

    // or
    void ma_or(Register rd, Register rs);
    void ma_or(Register rd, Imm32 imm);
    void ma_or(Register rd, Register rs, Imm32 imm);

    // xor
    void ma_xor(Register rd, Register rs);
    void ma_xor(Register rd, Imm32 imm);
    void ma_xor(Register rd, Register rs, Imm32 imm);

    // load
    void ma_load(Register dest, Address address, LoadStoreSize size = SizeWord,
                 LoadStoreExtension extension = SignExtend);
    void ma_load(Register dest, const BaseIndex& src, LoadStoreSize size = SizeWord,
                 LoadStoreExtension extension = SignExtend);

    // store
    void ma_store(Register data, Address address, LoadStoreSize size = SizeWord,
                  LoadStoreExtension extension = SignExtend);
    void ma_store(Register data, const BaseIndex& dest, LoadStoreSize size = SizeWord,
                  LoadStoreExtension extension = SignExtend);
    void ma_store(Imm32 imm, const BaseIndex& dest, LoadStoreSize size = SizeWord,
                  LoadStoreExtension extension = SignExtend);

    void computeScaledAddress(const BaseIndex& address, Register dest);

    void computeEffectiveAddress(const Address& address, Register dest) {
        ma_daddu(dest, address.base, Imm32(address.offset));
    }

    void computeEffectiveAddress(const BaseIndex& address, Register dest) {
        computeScaledAddress(address, dest);
        if (address.offset) {
            ma_daddu(dest, dest, Imm32(address.offset));
        }
    }

    // arithmetic based ops
    // add
    void ma_addu(Register rd, Register rs, Imm32 imm);
    void ma_addu(Register rd, Imm32 imm);
    void ma_daddu(Register rd, Register rs, Imm32 imm);
    void ma_daddu(Register rd, Register rs);
    void ma_daddu(Register rd, Imm32 imm);
    void ma_addTestOverflow(Register rd, Register rs, Register rt, Label* overflow);
    void ma_addTestOverflow(Register rd, Register rs, Imm32 imm, Label* overflow);

    // subtract
    void ma_subu(Register rd, Register rs, Imm32 imm);
    void ma_subu(Register rd, Imm32 imm);
    void ma_dsubu(Register rd, Register rs, Imm32 imm);
    void ma_dsubu(Register rd, Imm32 imm);
    void ma_subTestOverflow(Register rd, Register rs, Register rt, Label* overflow);
    void ma_subTestOverflow(Register rd, Register rs, Imm32 imm, Label* overflow);

    // multiplies.  For now, there are only few that we care about.
    void ma_mult(Register rs, Imm32 imm);
    void ma_dmult(Register rs, Imm32 imm);
    void ma_mul_branch_overflow(Register rd, Register rs, Register rt, Label* overflow);
    void ma_mul_branch_overflow(Register rd, Register rs, Imm32 imm, Label* overflow);

    // divisions
    void ma_div_branch_overflow(Register rd, Register rs, Register rt, Label* overflow);
    void ma_div_branch_overflow(Register rd, Register rs, Imm32 imm, Label* overflow);

    // fast mod, uses scratch registers, and thus needs to be in the assembler
    // implicitly assumes that we can overwrite dest at the beginning of the sequence
    void ma_mod_mask(Register src, Register dest, Register hold, Register remain,
                     int32_t shift, Label* negZero = nullptr);

    // stack
    void ma_pop(Register r);
    void ma_push(Register r);

    // branches when done from within mips-specific code
    void ma_b(Register lhs, Register rhs, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Register lhs, Imm32 imm, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Register lhs, ImmWord imm, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Register lhs, ImmPtr imm, Label* l, Condition c, JumpKind jumpKind = LongJump) {
        ma_b(lhs, ImmWord(uintptr_t(imm.value)), l, c, jumpKind);
    }
    void ma_b(Register lhs, ImmGCPtr imm, Label* l, Condition c, JumpKind jumpKind = LongJump) {
        MOZ_ASSERT(lhs != ScratchRegister);
        ma_li(ScratchRegister, imm);
        ma_b(lhs, ScratchRegister, l, c, jumpKind);
    }
    void ma_b(Register lhs, Address addr, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Address addr, Imm32 imm, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Address addr, ImmGCPtr imm, Label* l, Condition c, JumpKind jumpKind = LongJump);
    void ma_b(Address addr, Register rhs, Label* l, Condition c, JumpKind jumpKind = LongJump) {
        MOZ_ASSERT(rhs != ScratchRegister);
        ma_load(ScratchRegister, addr, SizeDouble);
        ma_b(ScratchRegister, rhs, l, c, jumpKind);
    }

    void ma_b(Label* l, JumpKind jumpKind = LongJump);
    void ma_bal(Label* l, DelaySlotFill delaySlotFill = FillDelaySlot);

    // fp instructions
    void ma_lis(FloatRegister dest, float value);
    void ma_lid(FloatRegister dest, double value);
    void ma_liNegZero(FloatRegister dest);

    void ma_mv(FloatRegister src, ValueOperand dest);
    void ma_mv(ValueOperand src, FloatRegister dest);

    void ma_ls(FloatRegister fd, Address address);
    void ma_ld(FloatRegister fd, Address address);
    void ma_sd(FloatRegister fd, Address address);
    void ma_sd(FloatRegister fd, BaseIndex address);
    void ma_ss(FloatRegister fd, Address address);
    void ma_ss(FloatRegister fd, BaseIndex address);

    void ma_pop(FloatRegister fs);
    void ma_push(FloatRegister fs);

    //FP branches
    void ma_bc1s(FloatRegister lhs, FloatRegister rhs, Label* label, DoubleCondition c,
                 JumpKind jumpKind = LongJump, FPConditionBit fcc = FCC0);
    void ma_bc1d(FloatRegister lhs, FloatRegister rhs, Label* label, DoubleCondition c,
                 JumpKind jumpKind = LongJump, FPConditionBit fcc = FCC0);


    // These fuctions abstract the access to high part of the double precision
    // float register. It is intended to work on both 32 bit and 64 bit
    // floating point coprocessor.
    void moveToDoubleHi(Register src, FloatRegister dest) {
        as_mthc1(src, dest);
    }
    void moveFromDoubleHi(FloatRegister src, Register dest) {
        as_mfhc1(dest, src);
    }

    void moveToDouble(Register src, FloatRegister dest) {
        as_dmtc1(src, dest);
    }
    void moveFromDouble(FloatRegister src, Register dest) {
        as_dmfc1(dest, src);
    }

    void moveToDoubleLo(Register src, FloatRegister dest) {
        as_mtc1(src, dest);
    }
    void moveFromDoubleLo(FloatRegister src, Register dest) {
        as_mfc1(dest, src);
    }

    void moveToFloat32(Register src, FloatRegister dest) {
        as_mtc1(src, dest);
    }
    void moveFromFloat32(FloatRegister src, Register dest) {
        as_mfc1(dest, src);
    }

  protected:
    void branchWithCode(InstImm code, Label* label, JumpKind jumpKind);
    Condition ma_cmp(Register rd, Register lhs, Register rhs, Condition c);

    void compareFloatingPoint(FloatFormat fmt, FloatRegister lhs, FloatRegister rhs,
                              DoubleCondition c, FloatTestKind* testKind,
                              FPConditionBit fcc = FCC0);

  public:
    void ma_callJit(const Register reg);
    void ma_callJit(Label* label);

    void ma_call(ImmPtr dest);

    void ma_jump(ImmPtr dest);

    void ma_cmp_set(Register dst, Register lhs, Register rhs, Condition c);
    void ma_cmp_set(Register dst, Register lhs, Imm32 imm, Condition c);
    void ma_cmp_set(Register dst, Register lhs, ImmPtr imm, Condition c);
    void ma_cmp_set(Register dst, Register lhs, ImmWord imm, Condition c);

    void ma_cmp_set_double(Register dst, FloatRegister lhs, FloatRegister rhs, DoubleCondition c);
    void ma_cmp_set_float32(Register dst, FloatRegister lhs, FloatRegister rhs, DoubleCondition c);
};

class MacroAssembler;

class MacroAssemblerMIPS64Compat : public MacroAssemblerMIPS64
{
  private:
    // Perform a downcast. Should be removed by Bug 996602.
    MacroAssembler& asMasm();
    const MacroAssembler& asMasm() const;

  private:
    // Number of bytes the stack is adjusted inside a call to C. Calls to C may
    // not be nested.
    bool inCall_;
    uint32_t args_;
    // The actual number of arguments that were passed, used to assert that
    // the initial number of arguments declared was correct.
    uint32_t passedArgs_;
    uint32_t passedArgTypes_;

    uint32_t usedArgSlots_;
    MoveOp::Type firstArgType;

    bool dynamicAlignment_;

    // Compute space needed for the function call and set the properties of the
    // callee.  It returns the space which has to be allocated for calling the
    // function.
    //
    // arg            Number of arguments of the function.
    void setupABICall(uint32_t arg);

  protected:
    MoveResolver moveResolver_;

    // Extra bytes currently pushed onto the frame beyond frameDepth_. This is
    // needed to compute offsets to stack slots while temporary space has been
    // reserved for unexpected spills or C++ function calls. It is maintained
    // by functions which track stack alignment, which for clear distinction
    // use StudlyCaps (for example, Push, Pop).
    uint32_t framePushed_;
    void adjustFrame(int value) {
        setFramePushed(framePushed_ + value);
    }
  public:
    MacroAssemblerMIPS64Compat()
      : inCall_(false),
        framePushed_(0)
    { }

  public:
    using MacroAssemblerMIPS64::call;

    void j(Label* dest) {
        ma_b(dest);
    }

    void mov(Register src, Register dest) {
        as_ori(dest, src, 0);
    }
    void mov(ImmWord imm, Register dest) {
        ma_li(dest, imm);
    }
    void mov(ImmPtr imm, Register dest) {
        mov(ImmWord(uintptr_t(imm.value)), dest);
    }
    void mov(Register src, Address dest) {
        MOZ_CRASH("NYI-IC");
    }
    void mov(Address src, Register dest) {
        MOZ_CRASH("NYI-IC");
    }

    void call(const Register reg) {
        as_jalr(reg);
        as_nop();
    }

    void call(Label* label) {
        ma_bal(label);
    }

    void call(ImmWord imm) {
        call(ImmPtr((void*)imm.value));
    }
    void call(ImmPtr imm) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, imm, Relocation::HARDCODED);
        ma_call(imm);
    }
    void call(AsmJSImmPtr imm) {
        movePtr(imm, CallReg);
        call(CallReg);
    }
    void call(JitCode* c) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, ImmPtr(c->raw()), Relocation::JITCODE);
        ma_liPatchable(ScratchRegister, ImmWord((uintptr_t)c->raw()));
        ma_callJit(ScratchRegister);
    }
    void call(const CallSiteDesc& desc, const Register reg) {
        call(reg);
        append(desc, currentOffset(), framePushed_);
    }
    void call(const CallSiteDesc& desc, Label* label) {
        call(label);
        append(desc, currentOffset(), framePushed_);
    }

    void callAndPushReturnAddress(Label* label) {
        ma_callJit(label);
    }

    void writeDataRelocation(const Value& val) {
        if (val.isMarkable()) {
            gc::Cell* cell = reinterpret_cast<gc::Cell *>(val.toGCThing());
            if (cell && gc::IsInsideNursery(cell))
                embedsNurseryPointers_ = true;
            dataRelocations_.writeUnsigned(currentOffset());
        }
    }

    void branch(JitCode* c) {
        BufferOffset bo = m_buffer.nextOffset();
        addPendingJump(bo, ImmPtr(c->raw()), Relocation::JITCODE);
        ma_liPatchable(ScratchRegister, ImmWord((uintptr_t)c->raw()));
        as_jr(ScratchRegister);
        as_nop();
    }
    void branch(const Register reg) {
        as_jr(reg);
        as_nop();
    }
    void nop() {
        as_nop();
    }
    void ret() {
        ma_pop(ra);
        as_jr(ra);
        as_nop();
    }
    void retn(Imm32 n) {
        // pc <- [sp]; sp += n
        loadPtr(Address(StackPointer, 0), ra);
        ma_daddu(StackPointer, StackPointer, n);
        as_jr(ra);
        as_nop();
    }
    void push(Imm32 imm) {
        ma_li(ScratchRegister, imm);
        ma_push(ScratchRegister);
    }
    void push(ImmWord imm) {
        ma_li(ScratchRegister, imm);
        ma_push(ScratchRegister);
    }
    void push(ImmGCPtr imm) {
        ma_li(ScratchRegister, imm);
        ma_push(ScratchRegister);
    }
    void push(const Address& address) {
        loadPtr(address, ScratchRegister);
        ma_push(ScratchRegister);
    }
    void push(Register reg) {
        ma_push(reg);
    }
    void push(FloatRegister reg) {
        ma_push(reg);
    }
    void pop(Register reg) {
        ma_pop(reg);
    }
    void pop(FloatRegister reg) {
        ma_pop(reg);
    }

    // Emit a branch that can be toggled to a non-operation. On MIPS64 we use
    // "andi" instruction to toggle the branch.
    // See ToggleToJmp(), ToggleToCmp().
    CodeOffsetLabel toggledJump(Label* label);

    // Emit a "jalr" or "nop" instruction. ToggleCall can be used to patch
    // this instruction.
    CodeOffsetLabel toggledCall(JitCode* target, bool enabled);

    static size_t ToggledCallSize(uint8_t* code) {
        // Eight instructions used in: MacroAssemblerMIPS64Compat::toggledCall
        return 8 * sizeof(uint32_t);
    }

    CodeOffsetLabel pushWithPatch(ImmWord imm) {
        CodeOffsetLabel label = movWithPatch(imm, ScratchRegister);
        ma_push(ScratchRegister);
        return label;
    }

    CodeOffsetLabel movWithPatch(ImmWord imm, Register dest) {
        CodeOffsetLabel label = CodeOffsetLabel(currentOffset());
        ma_liPatchable(dest, imm);
        return label;
    }
    CodeOffsetLabel movWithPatch(ImmPtr imm, Register dest) {
        return movWithPatch(ImmWord(uintptr_t(imm.value)), dest);
    }

    void jump(Label* label) {
        ma_b(label);
    }
    void jump(Register reg) {
        as_jr(reg);
        as_nop();
    }
    void jump(const Address& address) {
        loadPtr(address, ScratchRegister);
        as_jr(ScratchRegister);
        as_nop();
    }

    void jump(JitCode* code) {
        branch(code);
    }

    void neg32(Register reg) {
        ma_negu(reg, reg);
    }

    void splitTag(Register src, Register dest) {
        ma_dsrl(dest, src, Imm32(JSVAL_TAG_SHIFT));
    }

    void splitTag(const ValueOperand& operand, Register dest) {
        splitTag(operand.valueReg(), dest);
    }

    // Returns the register containing the type tag.
    Register splitTagForTest(const ValueOperand& value) {
        splitTag(value, SecondScratchReg);
        return SecondScratchReg;
    }

    void branchTestGCThing(Condition cond, const Address& address, Label* label);
    void branchTestGCThing(Condition cond, const BaseIndex& src, Label* label);

    void branchTestPrimitive(Condition cond, const ValueOperand& value, Label* label);
    void branchTestPrimitive(Condition cond, Register tag, Label* label);

    void branchTestValue(Condition cond, const ValueOperand& value, const Value& v, Label* label);
    void branchTestValue(Condition cond, const Address& valaddr, const ValueOperand& value,
                         Label* label);

    // unboxing code
    void unboxNonDouble(const ValueOperand& operand, Register dest);
    void unboxNonDouble(const Address& src, Register dest);
    void unboxNonDouble(const BaseIndex& src, Register dest);
    void unboxInt32(const ValueOperand& operand, Register dest);
    void unboxInt32(const Operand& operand, Register dest);
    void unboxInt32(const Address& src, Register dest);
    void unboxInt32(const BaseIndex& src, Register dest);
    void unboxBoolean(const ValueOperand& operand, Register dest);
    void unboxBoolean(const Operand& operand, Register dest);
    void unboxBoolean(const Address& src, Register dest);
    void unboxBoolean(const BaseIndex& src, Register dest);
    void unboxDouble(const ValueOperand& operand, FloatRegister dest);
    void unboxDouble(const Address& src, FloatRegister dest);
    void unboxString(const ValueOperand& operand, Register dest);
    void unboxString(const Operand& operand, Register dest);
    void unboxString(const Address& src, Register dest);
    void unboxSymbol(const ValueOperand& src, Register dest);
    void unboxSymbol(const Operand& src, Register dest);
    void unboxSymbol(const Address& src, Register dest);
    void unboxObject(const ValueOperand& src, Register dest);
    void unboxObject(const Operand& src, Register dest);
    void unboxObject(const Address& src, Register dest);
    void unboxObject(const BaseIndex& src, Register dest) { unboxNonDouble(src, dest); }
    void unboxValue(const ValueOperand& src, AnyRegister dest);
    void unboxPrivate(const ValueOperand& src, Register dest);

    void notBoolean(const ValueOperand& val) {
        as_xori(val.valueReg(), val.valueReg(), 1);
    }

    // boxing code
    void boxDouble(FloatRegister src, const ValueOperand& dest);
    void boxNonDouble(JSValueType type, Register src, const ValueOperand& dest);

    // Extended unboxing API. If the payload is already in a register, returns
    // that register. Otherwise, provides a move to the given scratch register,
    // and returns that.
    Register extractObject(const Address& address, Register scratch);
    Register extractObject(const ValueOperand& value, Register scratch) {
        unboxObject(value, scratch);
        return scratch;
    }
    Register extractInt32(const ValueOperand& value, Register scratch) {
        unboxInt32(value, scratch);
        return scratch;
    }
    Register extractBoolean(const ValueOperand& value, Register scratch) {
        unboxBoolean(value, scratch);
        return scratch;
    }
    Register extractTag(const Address& address, Register scratch);
    Register extractTag(const BaseIndex& address, Register scratch);
    Register extractTag(const ValueOperand& value, Register scratch) {
        MOZ_ASSERT(scratch != ScratchRegister);
        splitTag(value, scratch);
        return scratch;
    }

    void boolValueToDouble(const ValueOperand& operand, FloatRegister dest);
    void int32ValueToDouble(const ValueOperand& operand, FloatRegister dest);
    void loadInt32OrDouble(const Address& src, FloatRegister dest);
    void loadInt32OrDouble(const BaseIndex& addr, FloatRegister dest);
    void loadConstantDouble(double dp, FloatRegister dest);

    void boolValueToFloat32(const ValueOperand& operand, FloatRegister dest);
    void int32ValueToFloat32(const ValueOperand& operand, FloatRegister dest);
    void loadConstantFloat32(float f, FloatRegister dest);

    void branchTestInt32(Condition cond, const ValueOperand& value, Label* label);
    void branchTestInt32(Condition cond, Register tag, Label* label);
    void branchTestInt32(Condition cond, const Address& address, Label* label);
    void branchTestInt32(Condition cond, const BaseIndex& src, Label* label);

    void branchTestBoolean(Condition cond, const ValueOperand& value, Label* label);
    void branchTestBoolean(Condition cond, Register tag, Label* label);
    void branchTestBoolean(Condition cond, const BaseIndex& src, Label* label);

    void branch32(Condition cond, Register lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, Register lhs, Imm32 imm, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, imm, label, cond, jumpKind);
    }
    void branch32(Condition cond, const Operand& lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        if (lhs.getTag() == Operand::REG) {
            ma_b(lhs.toReg(), rhs, label, cond, jumpKind);
        } else {
            branch32(cond, lhs.toAddress(), rhs, label, jumpKind);
        }
    }
    void branch32(Condition cond, const Operand& lhs, Imm32 rhs, Label* label, JumpKind jumpKind = LongJump) {
        if (lhs.getTag() == Operand::REG) {
            ma_b(lhs.toReg(), rhs, label, cond, jumpKind);
        } else {
            branch32(cond, lhs.toAddress(), rhs, label, jumpKind);
        }
    }
    void branch32(Condition cond, const Address& lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        load32(lhs, SecondScratchReg); // ma_b might use scratch
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, const Address& lhs, Imm32 rhs, Label* label, JumpKind jumpKind = LongJump) {
        load32(lhs, SecondScratchReg);
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, const BaseIndex& lhs, Imm32 rhs, Label* label, JumpKind jumpKind = LongJump) {
        load32(lhs, SecondScratchReg);
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Imm32 rhs, Label* label, JumpKind jumpKind = LongJump) {
        load32(lhs, SecondScratchReg);
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, AbsoluteAddress lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        load32(lhs, SecondScratchReg);
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branch32(Condition cond, AsmJSAbsoluteAddress addr, Imm32 imm, Label* label, JumpKind jumpKind = LongJump) {
        load32(addr, SecondScratchReg);
        ma_b(SecondScratchReg, imm, label, cond, jumpKind);
    }

    void branchPtr(Condition cond, Register lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, rhs, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Register lhs, ImmGCPtr ptr, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Register lhs, ImmWord imm, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, imm, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Register lhs, ImmPtr imm, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, imm, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Register lhs, AsmJSImmPtr imm, Label* label, JumpKind jumpKind = LongJump) {
        movePtr(imm, SecondScratchReg);
        ma_b(lhs, SecondScratchReg, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Register lhs, Imm32 imm, Label* label, JumpKind jumpKind = LongJump) {
        ma_b(lhs, imm, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, const Address& lhs, Register rhs, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(lhs, SecondScratchReg);
        ma_b(SecondScratchReg, rhs, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Address addr, ImmGCPtr ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Address addr, ImmMaybeNurseryPtr ptr, Label* label, JumpKind jumpKind = LongJump) {
        branchPtr(cond, addr, noteMaybeNurseryPtr(ptr), label, jumpKind);
    }
    void branchPtr(Condition cond, Address addr, ImmWord ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, Address addr, ImmPtr ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, AbsoluteAddress addr, Register ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, AbsoluteAddress addr, ImmWord ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }
    void branchPtr(Condition cond, AsmJSAbsoluteAddress addr, Register ptr, Label* label, JumpKind jumpKind = LongJump) {
        loadPtr(addr, SecondScratchReg);
        ma_b(SecondScratchReg, ptr, label, cond, jumpKind);
    }

    void branchPrivatePtr(Condition cond, const Address& lhs, ImmPtr ptr, Label* label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, const Address& lhs, Register ptr, Label* label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void branchPrivatePtr(Condition cond, Register lhs, ImmWord ptr, Label* label) {
        branchPtr(cond, lhs, ptr, label);
    }

    void decBranchPtr(Condition cond, Register lhs, Imm32 imm, Label* label) {
        subPtr(imm, lhs);
        branchPtr(cond, lhs, Imm32(0), label);
    }

    void branchTestDouble(Condition cond, const ValueOperand& value, Label* label);
    void branchTestDouble(Condition cond, Register tag, Label* label);
    void branchTestDouble(Condition cond, const Address& address, Label* label);
    void branchTestDouble(Condition cond, const BaseIndex& src, Label* label);

    void branchTestNull(Condition cond, const ValueOperand& value, Label* label);
    void branchTestNull(Condition cond, Register tag, Label* label);
    void branchTestNull(Condition cond, const BaseIndex& src, Label* label);
    void branchTestNull(Condition cond, const Address& address, Label* label);
    void testNullSet(Condition cond, const ValueOperand& value, Register dest);

    void branchTestObject(Condition cond, const ValueOperand& value, Label* label);
    void branchTestObject(Condition cond, Register tag, Label* label);
    void branchTestObject(Condition cond, const BaseIndex& src, Label* label);
    void branchTestObject(Condition cond, const Address& src, Label* label);
    void testObjectSet(Condition cond, const ValueOperand& value, Register dest);

    void branchTestString(Condition cond, const ValueOperand& value, Label* label);
    void branchTestString(Condition cond, Register tag, Label* label);
    void branchTestString(Condition cond, const BaseIndex& src, Label* label);

    void branchTestSymbol(Condition cond, const ValueOperand& value, Label* label);
    void branchTestSymbol(Condition cond, const Register& tag, Label* label);
    void branchTestSymbol(Condition cond, const BaseIndex& src, Label* label);

    void branchTestUndefined(Condition cond, const ValueOperand& value, Label* label);
    void branchTestUndefined(Condition cond, Register tag, Label* label);
    void branchTestUndefined(Condition cond, const BaseIndex& src, Label* label);
    void branchTestUndefined(Condition cond, const Address& address, Label* label);
    void testUndefinedSet(Condition cond, const ValueOperand& value, Register dest);

    void branchTestNumber(Condition cond, const ValueOperand& value, Label* label);
    void branchTestNumber(Condition cond, Register tag, Label* label);

    void branchTestMagic(Condition cond, const ValueOperand& value, Label* label);
    void branchTestMagic(Condition cond, Register tag, Label* label);
    void branchTestMagic(Condition cond, const Address& address, Label* label);
    void branchTestMagic(Condition cond, const BaseIndex& src, Label* label);

    void branchTestMagicValue(Condition cond, const ValueOperand& val, JSWhyMagic why,
                              Label* label) {
        MOZ_ASSERT(cond == Equal || cond == NotEqual);
        branchTestValue(cond, val, MagicValue(why), label);
    }

    void branchTestInt32Truthy(bool b, const ValueOperand& value, Label* label);

    void branchTestStringTruthy(bool b, const ValueOperand& value, Label* label);

    void branchTestDoubleTruthy(bool b, FloatRegister value, Label* label);

    void branchTestBooleanTruthy(bool b, const ValueOperand& operand, Label* label);

    void branchTest32(Condition cond, Register lhs, Register rhs, Label* label) {
        MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed || cond == NotSigned);
        if (lhs == rhs) {
            ma_b(lhs, rhs, label, cond);
        } else {
            as_and(ScratchRegister, lhs, rhs);
            ma_b(ScratchRegister, ScratchRegister, label, cond);
        }
    }
    void branchTest32(Condition cond, Register lhs, Imm32 imm, Label* label) {
        ma_li(ScratchRegister, imm);
        branchTest32(cond, lhs, ScratchRegister, label);
    }
    void branchTest32(Condition cond, const Address& address, Imm32 imm, Label* label) {
        load32(address, SecondScratchReg);
        branchTest32(cond, SecondScratchReg, imm, label);
    }
    void branchTest32(Condition cond, AbsoluteAddress address, Imm32 imm, Label* label) {
        load32(address, ScratchRegister);
        branchTest32(cond, ScratchRegister, imm, label);
    }
    void branchTestPtr(Condition cond, Register lhs, Register rhs, Label* label) {
        MOZ_ASSERT(cond == Zero || cond == NonZero || cond == Signed || cond == NotSigned);
        if (lhs == rhs) {
            ma_b(lhs, rhs, label, cond);
        } else {
            as_and(ScratchRegister, lhs, rhs);
            ma_b(ScratchRegister, ScratchRegister, label, cond);
        }
    }
    void branchTestPtr(Condition cond, Register lhs, const Imm32 rhs, Label* label) {
        ma_li(ScratchRegister, rhs);
        branchTestPtr(cond, lhs, ScratchRegister, label);
    }
    void branchTestPtr(Condition cond, const Address& lhs, Imm32 imm, Label* label) {
        loadPtr(lhs, SecondScratchReg);
        branchTestPtr(cond, SecondScratchReg, imm, label);
    }

public:
    void moveValue(const Value& val, Register dest);

    CodeOffsetJump backedgeJump(RepatchLabel* label);
    CodeOffsetJump jumpWithPatch(RepatchLabel* label);

    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Register reg, T ptr, RepatchLabel* label) {
        movePtr(ptr, ScratchRegister);
        Label skipJump;
        ma_b(reg, ScratchRegister, &skipJump, InvertCondition(cond), ShortJump);
        CodeOffsetJump off = jumpWithPatch(label);
        bind(&skipJump);
        return off;
    }

    template <typename T>
    CodeOffsetJump branchPtrWithPatch(Condition cond, Address addr, T ptr, RepatchLabel* label) {
        loadPtr(addr, SecondScratchReg);
        movePtr(ptr, ScratchRegister);
        Label skipJump;
        ma_b(SecondScratchReg, ScratchRegister, &skipJump, InvertCondition(cond), ShortJump);
        CodeOffsetJump off = jumpWithPatch(label);
        bind(&skipJump);
        return off;
    }

    template <typename T>
    void loadUnboxedValue(const T& address, MIRType type, AnyRegister dest) {
        if (dest.isFloat())
            loadInt32OrDouble(address, dest.fpu());
        else if (type == MIRType_Int32)
            unboxInt32(address, dest.gpr());
        else if (type == MIRType_Boolean)
            unboxBoolean(address, dest.gpr());
        else
            unboxNonDouble(address, dest.gpr());
    }

    template <typename T>
    void storeUnboxedValue(ConstantOrRegister value, MIRType valueType, const T& dest,
                           MIRType slotType);

    template <typename T>
    void storeUnboxedPayload(ValueOperand value, T address, size_t nbytes) {
        switch (nbytes) {
          case 8:
            unboxNonDouble(value, ScratchRegister);
            storePtr(ScratchRegister, address);
            return;
          case 4:
            store32(value.valueReg(), address);
            return;
          case 1:
            store8(value.valueReg(), address);
            return;
          default: MOZ_CRASH("Bad payload width");
        }
    }

    void moveValue(const Value& val, const ValueOperand& dest);

    void moveValue(const ValueOperand& src, const ValueOperand& dest) {
        if (src.valueReg() != dest.valueReg())
          ma_move(dest.valueReg(), src.valueReg());
    }
    void boxValue(JSValueType type, Register src, Register dest) {
        MOZ_ASSERT(src != dest);

        JSValueTag tag = (JSValueTag)JSVAL_TYPE_TO_TAG(type);
        ma_li(dest, Imm32(tag));
        ma_dsll(dest, dest, Imm32(JSVAL_TAG_SHIFT));
        ma_dins(dest, src, Imm32(0), Imm32(JSVAL_TAG_SHIFT));
    }

    Address ToPayload(Address value) {
        return value;
    }

    void storeValue(ValueOperand val, Operand dst);
    void storeValue(ValueOperand val, const BaseIndex& dest);
    void storeValue(JSValueType type, Register reg, BaseIndex dest);
    void storeValue(ValueOperand val, const Address& dest);
    void storeValue(JSValueType type, Register reg, Address dest);
    void storeValue(const Value& val, Address dest);
    void storeValue(const Value& val, BaseIndex dest);

    void loadValue(Address src, ValueOperand val);
    void loadValue(Operand dest, ValueOperand val) {
        loadValue(dest.toAddress(), val);
    }
    void loadValue(const BaseIndex& addr, ValueOperand val);
    void tagValue(JSValueType type, Register payload, ValueOperand dest);

    void pushValue(ValueOperand val);
    void popValue(ValueOperand val);
    void pushValue(const Value& val) {
        jsval_layout jv = JSVAL_TO_IMPL(val);
        if (val.isMarkable()) {
            writeDataRelocation(val);
            movWithPatch(ImmWord(jv.asBits), ScratchRegister);
            push(ScratchRegister);
        } else {
            push(ImmWord(jv.asBits));
        }
    }
    void pushValue(JSValueType type, Register reg) {
        boxValue(type, reg, ScratchRegister);
        push(ScratchRegister);
    }
    void pushValue(const Address& addr);

    void makeFrameDescriptor(Register frameSizeReg, FrameType type) {
        ma_dsll(frameSizeReg, frameSizeReg, Imm32(FRAMESIZE_SHIFT));
        ma_or(frameSizeReg, frameSizeReg, Imm32(type));
    }

    void handleFailureWithHandlerTail(void* handler);

  private:
    void atomicEffectOpMIPSr2(int nbytes, AtomicOp op, const Register& value, const Register& addr,
                            AllocatableGeneralRegisterSet& regs);
    void atomicFetchOpMIPSr2(int nbytes, bool signExtend, AtomicOp op, const Register& value, const Register& addr,
                            const Register& temp, const Register& output, AllocatableGeneralRegisterSet& regs);

    void atomicEffectOp(int nbytes, AtomicOp op, const Imm32& value, const Address& address);
    void atomicEffectOp(int nbytes, AtomicOp op, const Imm32& value, const BaseIndex& address);
    void atomicEffectOp(int nbytes, AtomicOp op, const Register& value, const Address& address);
    void atomicEffectOp(int nbytes, AtomicOp op, const Register& value, const BaseIndex& address);

    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Imm32& value,
                       const Address& address, Register temp, Register output);
    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Imm32& value,
                       const BaseIndex& address, Register temp, Register output);
    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Register& value,
                       const Address& address, Register temp, Register output);
    void atomicFetchOp(int nbytes, bool signExtend, AtomicOp op, const Register& value,
                       const BaseIndex& address, Register temp, Register output);

    void compareExchangeMIPSr2(int nbytes, bool signExtend, const Register& addr, Register oldval,
                         Register newval, Register output, AllocatableGeneralRegisterSet& regs);

    void compareExchange(int nbytes, bool signExtend, const Address& address, Register oldval,
                         Register newval, Register output);
    void compareExchange(int nbytes, bool signExtend, const BaseIndex& address, Register oldval,
                         Register newval, Register output);

    /////////////////////////////////////////////////////////////////
    // Common interface.
    /////////////////////////////////////////////////////////////////
  public:
    // The following functions are exposed for use in platform-shared code.

    template<typename T>
    void compareExchange8SignExtend(const T& mem, Register oldval, Register newval, Register output)
    {
        compareExchange(1, true, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange8ZeroExtend(const T& mem, Register oldval, Register newval, Register output)
    {
        compareExchange(1, false, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange16SignExtend(const T& mem, Register oldval, Register newval, Register output)
    {
        compareExchange(2, true, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange16ZeroExtend(const T& mem, Register oldval, Register newval, Register output)
    {
        compareExchange(2, false, mem, oldval, newval, output);
    }
    template<typename T>
    void compareExchange32(const T& mem, Register oldval, Register newval, Register output)
    {
        compareExchange(4, false, mem, oldval, newval, output);
    }

    template<typename T, typename S>
    void atomicFetchAdd8SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd8ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd16SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd16ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAdd32(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchAddOp, value, mem, temp, output);
    }
    template <typename T, typename S>
    void atomicAdd8(const T& value, const S& mem) {
        atomicEffectOp(1, AtomicFetchAddOp, value, mem);
    }
    template <typename T, typename S>
    void atomicAdd16(const T& value, const S& mem) {
        atomicEffectOp(2, AtomicFetchAddOp, value, mem);
    }
    template <typename T, typename S>
    void atomicAdd32(const T& value, const S& mem) {
        atomicEffectOp(4, AtomicFetchAddOp, value, mem);
    }

    template<typename T, typename S>
    void atomicFetchSub8SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub8ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub16SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub16ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchSubOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchSub32(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchSubOp, value, mem, temp, output);
    }
    template <typename T, typename S>
    void atomicSub8(const T& value, const S& mem) {
        atomicEffectOp(1, AtomicFetchSubOp, value, mem);
    }
    template <typename T, typename S>
    void atomicSub16(const T& value, const S& mem) {
        atomicEffectOp(2, AtomicFetchSubOp, value, mem);
    }
    template <typename T, typename S>
    void atomicSub32(const T& value, const S& mem) {
        atomicEffectOp(4, AtomicFetchSubOp, value, mem);
    }

    template<typename T, typename S>
    void atomicFetchAnd8SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd8ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd16SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd16ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchAndOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchAnd32(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchAndOp, value, mem, temp, output);
    }
    template <typename T, typename S>
    void atomicAnd8(const T& value, const S& mem) {
        atomicEffectOp(1, AtomicFetchAndOp, value, mem);
    }
    template <typename T, typename S>
    void atomicAnd16(const T& value, const S& mem) {
        atomicEffectOp(2, AtomicFetchAndOp, value, mem);
    }
    template <typename T, typename S>
    void atomicAnd32(const T& value, const S& mem) {
        atomicEffectOp(4, AtomicFetchAndOp, value, mem);
    }

    template<typename T, typename S>
    void atomicFetchOr8SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr8ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr16SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr16ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchOrOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchOr32(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchOrOp, value, mem, temp, output);
    }
    template <typename T, typename S>
    void atomicOr8(const T& value, const S& mem) {
        atomicEffectOp(1, AtomicFetchOrOp, value, mem);
    }
    template <typename T, typename S>
    void atomicOr16(const T& value, const S& mem) {
        atomicEffectOp(2, AtomicFetchOrOp, value, mem);
    }
    template <typename T, typename S>
    void atomicOr32(const T& value, const S& mem) {
        atomicEffectOp(4, AtomicFetchOrOp, value, mem);
    }

    template<typename T, typename S>
    void atomicFetchXor8SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, true, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor8ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(1, false, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor16SignExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, true, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor16ZeroExtend(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(2, false, AtomicFetchXorOp, value, mem, temp, output);
    }
    template<typename T, typename S>
    void atomicFetchXor32(const S& value, const T& mem, Register temp, Register output) {
        atomicFetchOp(4, false, AtomicFetchXorOp, value, mem, temp, output);
    }
    template <typename T, typename S>
    void atomicXor8(const T& value, const S& mem) {
        atomicEffectOp(1, AtomicFetchXorOp, value, mem);
    }
    template <typename T, typename S>
    void atomicXor16(const T& value, const S& mem) {
        atomicEffectOp(2, AtomicFetchXorOp, value, mem);
    }
    template <typename T, typename S>
    void atomicXor32(const T& value, const S& mem) {
        atomicEffectOp(4, AtomicFetchXorOp, value, mem);
    }

    CodeOffsetLabel PushWithPatch(ImmWord word) {
        framePushed_ += sizeof(word.value);
        return pushWithPatch(word);
    }
    CodeOffsetLabel PushWithPatch(ImmPtr imm) {
        return PushWithPatch(ImmWord(uintptr_t(imm.value)));
    }

    void implicitPop(uint32_t args) {
        MOZ_ASSERT(args % sizeof(intptr_t) == 0);
        adjustFrame(-args);
    }
    uint32_t framePushed() const {
        return framePushed_;
    }
    void setFramePushed(uint32_t framePushed) {
        framePushed_ = framePushed;
    }

    // Builds an exit frame on the stack, with a return address to an internal
    // non-function. Returns offset to be passed to markSafepointAt().
    void buildFakeExitFrame(Register scratch, uint32_t* offset);

    void callWithExitFrame(Label* target);
    void callWithExitFrame(JitCode* target);
    void callWithExitFrame(JitCode* target, Register dynStack);

    // Makes a call using the only two methods that it is sane for indep code
    // to make a call.
    void callJit(Register callee);
    void callJitFromAsmJS(Register callee) { callJit(callee); }

    void reserveStack(uint32_t amount);
    void freeStack(uint32_t amount);
    void freeStack(Register amount);

    void add32(Register src, Register dest);
    void add32(Imm32 imm, Register dest);
    void add32(Imm32 imm, const Address& dest);
    void sub32(Imm32 imm, Register dest);
    void sub32(Register src, Register dest);

    void incrementInt32Value(const Address& addr) {
        add32(Imm32(1), addr);
    }

    template <typename T>
    void branchAdd32(Condition cond, T src, Register dest, Label* overflow) {
        switch (cond) {
          case Overflow:
            ma_addTestOverflow(dest, dest, src, overflow);
            break;
          default:
            MOZ_CRASH("NYI");
        }
    }
    template <typename T>
    void branchSub32(Condition cond, T src, Register dest, Label* overflow) {
        switch (cond) {
          case Overflow:
            ma_subTestOverflow(dest, dest, src, overflow);
            break;
          case NonZero:
          case Zero:
            sub32(src, dest);
            ma_b(dest, dest, overflow, cond);
            break;
          default:
            MOZ_CRASH("NYI");
        }
    }

    void and32(Register src, Register dest);
    void and32(Imm32 imm, Register dest);
    void and32(Imm32 imm, const Address& dest);
    void and32(const Address& src, Register dest);
    void or32(Imm32 imm, Register dest);
    void or32(Imm32 imm, const Address& dest);
    void xor32(Imm32 imm, Register dest);
    void xorPtr(Imm32 imm, Register dest);
    void xorPtr(Register src, Register dest);
    void orPtr(Imm32 imm, Register dest);
    void orPtr(Register src, Register dest);
    void andPtr(Imm32 imm, Register dest);
    void andPtr(ImmWord imm, Register dest);
    void andPtr(Register src, Register dest);
    void addPtr(Register src, Register dest);
    void subPtr(Register src, Register dest);
    void addPtr(const Address& src, Register dest);
    void not32(Register reg);

    void move32(Imm32 imm, Register dest);
    void move32(Register src, Register dest);

    void movePtr(Register src, Register dest);
    void movePtr(ImmWord imm, Register dest);
    void movePtr(ImmPtr imm, Register dest);
    void movePtr(AsmJSImmPtr imm, Register dest);
    void movePtr(ImmGCPtr imm, Register dest);
    void movePtr(ImmMaybeNurseryPtr imm, Register dest);

    void load8SignExtend(const Address& address, Register dest);
    void load8SignExtend(const BaseIndex& src, Register dest);

    void load8ZeroExtend(const Address& address, Register dest);
    void load8ZeroExtend(const BaseIndex& src, Register dest);

    void load16SignExtend(const Address& address, Register dest);
    void load16SignExtend(const BaseIndex& src, Register dest);

    void load16ZeroExtend(const Address& address, Register dest);
    void load16ZeroExtend(const BaseIndex& src, Register dest);

    void load32(const Address& address, Register dest);
    void load32(const BaseIndex& address, Register dest);
    void load32(AbsoluteAddress address, Register dest);
    void load32(AsmJSAbsoluteAddress address, Register dest);

    void loadPtr(const Address& address, Register dest);
    void loadPtr(const BaseIndex& src, Register dest);
    void loadPtr(AbsoluteAddress address, Register dest);
    void loadPtr(AsmJSAbsoluteAddress address, Register dest);

    void loadPrivate(const Address& address, Register dest);

    void loadInt32x1(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadInt32x1(const BaseIndex& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadInt32x2(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadInt32x2(const BaseIndex& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadInt32x3(const Address& src, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadInt32x3(const BaseIndex& src, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeInt32x1(FloatRegister src, const Address& dest) { MOZ_CRASH("NYI"); }
    void storeInt32x1(FloatRegister src, const BaseIndex& dest) { MOZ_CRASH("NYI"); }
    void storeInt32x2(FloatRegister src, const Address& dest) { MOZ_CRASH("NYI"); }
    void storeInt32x2(FloatRegister src, const BaseIndex& dest) { MOZ_CRASH("NYI"); }
    void storeInt32x3(FloatRegister src, const Address& dest) { MOZ_CRASH("NYI"); }
    void storeInt32x3(FloatRegister src, const BaseIndex& dest) { MOZ_CRASH("NYI"); }
    void loadAlignedInt32x4(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeAlignedInt32x4(FloatRegister src, Address addr) { MOZ_CRASH("NYI"); }
    void loadUnalignedInt32x4(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadUnalignedInt32x4(const BaseIndex& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeUnalignedInt32x4(FloatRegister src, Address addr) { MOZ_CRASH("NYI"); }
    void storeUnalignedInt32x4(FloatRegister src, BaseIndex addr) { MOZ_CRASH("NYI"); }

    void loadFloat32x3(const Address& src, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadFloat32x3(const BaseIndex& src, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeFloat32x3(FloatRegister src, const Address& dest) { MOZ_CRASH("NYI"); }
    void storeFloat32x3(FloatRegister src, const BaseIndex& dest) { MOZ_CRASH("NYI"); }
    void loadAlignedFloat32x4(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeAlignedFloat32x4(FloatRegister src, Address addr) { MOZ_CRASH("NYI"); }
    void loadUnalignedFloat32x4(const Address& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void loadUnalignedFloat32x4(const BaseIndex& addr, FloatRegister dest) { MOZ_CRASH("NYI"); }
    void storeUnalignedFloat32x4(FloatRegister src, Address addr) { MOZ_CRASH("NYI"); }
    void storeUnalignedFloat32x4(FloatRegister src, BaseIndex addr) { MOZ_CRASH("NYI"); }

    void loadDouble(const Address& addr, FloatRegister dest);
    void loadDouble(const BaseIndex& src, FloatRegister dest);

    // Load a float value into a register, then expand it to a double.
    void loadFloatAsDouble(const Address& addr, FloatRegister dest);
    void loadFloatAsDouble(const BaseIndex& src, FloatRegister dest);

    void loadFloat32(const Address& addr, FloatRegister dest);
    void loadFloat32(const BaseIndex& src, FloatRegister dest);

    void store8(Register src, const Address& address);
    void store8(Imm32 imm, const Address& address);
    void store8(Register src, const BaseIndex& address);
    void store8(Imm32 imm, const BaseIndex& address);

    void store16(Register src, const Address& address);
    void store16(Imm32 imm, const Address& address);
    void store16(Register src, const BaseIndex& address);
    void store16(Imm32 imm, const BaseIndex& address);

    void store32(Register src, AbsoluteAddress address);
    void store32(Register src, const Address& address);
    void store32(Register src, const BaseIndex& address);
    void store32(Imm32 src, const Address& address);
    void store32(Imm32 src, const BaseIndex& address);

    // NOTE: This will use second scratch on MIPS64. Only ARM needs the
    // implementation without second scratch.
    void store32_NoSecondScratch(Imm32 src, const Address& address) {
        store32(src, address);
    }

    template <typename T> void storePtr(ImmWord imm, T address);
    template <typename T> void storePtr(ImmPtr imm, T address);
    template <typename T> void storePtr(ImmGCPtr imm, T address);
    void storePtr(Register src, const Address& address);
    void storePtr(Register src, const BaseIndex& address);
    void storePtr(Register src, AbsoluteAddress dest);
    void storeDouble(FloatRegister src, Address addr) {
        ma_sd(src, addr);
    }
    void storeDouble(FloatRegister src, BaseIndex addr) {
        MOZ_ASSERT(addr.offset == 0);
        ma_sd(src, addr);
    }
    void moveDouble(FloatRegister src, FloatRegister dest) {
        as_movd(dest, src);
    }

    void storeFloat32(FloatRegister src, Address addr) {
        ma_ss(src, addr);
    }
    void storeFloat32(FloatRegister src, BaseIndex addr) {
        MOZ_ASSERT(addr.offset == 0);
        ma_ss(src, addr);
    }

    void zeroDouble(FloatRegister reg) {
        moveToDoubleLo(zero, reg);
        moveToDoubleHi(zero, reg);
    }

    void clampIntToUint8(Register reg) {
        // look at (reg >> 8) if it is 0, then src shouldn't be clamped
        // if it is <0, then we want to clamp to 0,
        // otherwise, we wish to clamp to 255
        Label done;
        ma_move(ScratchRegister, reg);
        as_dsra(ScratchRegister, ScratchRegister, 8);
        ma_b(ScratchRegister, ScratchRegister, &done, Assembler::Zero, ShortJump);
        {
            Label negative;
            ma_b(ScratchRegister, ScratchRegister, &negative, Assembler::Signed, ShortJump);
            {
                ma_li(reg, Imm32(255));
                ma_b(&done, ShortJump);
            }
            bind(&negative);
            {
                ma_move(reg, zero);
            }
        }
        bind(&done);
    }

    void subPtr(Imm32 imm, const Register dest);
    void subPtr(const Address& addr, const Register dest);
    void subPtr(Register src, const Address& dest);
    void addPtr(Imm32 imm, const Register dest);
    void addPtr(Imm32 imm, const Address& dest);
    void addPtr(ImmWord imm, const Register dest) {
        movePtr(imm, ScratchRegister);
        addPtr(ScratchRegister, dest);
    }
    void addPtr(ImmPtr imm, const Register dest) {
        addPtr(ImmWord(uintptr_t(imm.value)), dest);
    }
    void mulBy3(const Register& src, const Register& dest) {
        as_daddu(dest, src, src);
        as_daddu(dest, dest, src);
    }

    void breakpoint();

    void branchDouble(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs,
                      Label* label);

    void branchFloat(DoubleCondition cond, FloatRegister lhs, FloatRegister rhs,
                     Label* label);

    void checkStackAlignment();

    void alignStackPointer();
    void restoreStackPointer();
    static void calculateAlignedStackPointer(void** stackPointer);

    void rshiftPtr(Imm32 imm, Register dest) {
        ma_dsrl(dest, dest, imm);
    }
    void rshiftPtrArithmetic(Imm32 imm, Register dest) {
        ma_dsra(dest, dest, imm);
    }
    void lshiftPtr(Imm32 imm, Register dest) {
        ma_dsll(dest, dest, imm);
    }

    // If source is a double, load it into dest. If source is int32,
    // convert it to double. Else, branch to failure.
    void ensureDouble(const ValueOperand& source, FloatRegister dest, Label* failure);

    template <typename T1, typename T2>
    void cmpPtrSet(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest)
    {
        ma_cmp_set(dest, lhs, rhs, cond);
    }
    void cmpPtrSet(Assembler::Condition cond, Address lhs, ImmPtr rhs, Register dest);
    void cmpPtrSet(Assembler::Condition cond, Register lhs, Address rhs, Register dest);

    template <typename T1, typename T2>
    void cmp32Set(Assembler::Condition cond, T1 lhs, T2 rhs, Register dest)
    {
        ma_cmp_set(dest, lhs, rhs, cond);
    }
    void cmp32Set(Assembler::Condition cond, Register lhs, Address rhs, Register dest);

    // Setup a call to C/C++ code, given the number of general arguments it
    // takes. Note that this only supports cdecl.
    //
    // In order for alignment to work correctly, the MacroAssembler must have a
    // consistent view of the stack displacement. It is okay to call "push"
    // manually, however, if the stack alignment were to change, the macro
    // assembler should be notified before starting a call.
    void setupAlignedABICall(uint32_t args);

    // Sets up an ABI call for when the alignment is not known. This may need a
    // scratch register.
    void setupUnalignedABICall(uint32_t args, Register scratch);

    // Arguments must be assigned in a left-to-right order. This process may
    // temporarily use more stack, in which case sp-relative addresses will be
    // automatically adjusted. It is extremely important that sp-relative
    // addresses are computed *after* setupABICall(). Furthermore, no
    // operations should be emitted while setting arguments.
    void passABIArg(const MoveOperand& from, MoveOp::Type type);
    void passABIArg(Register reg);
    void passABIArg(FloatRegister reg, MoveOp::Type type);
    void passABIArg(const ValueOperand& regs);

  protected:
    bool buildOOLFakeExitFrame(void* fakeReturnAddr);

  private:
    void callWithABIPre(uint32_t* stackAdjust, bool callFromAsmJS = false);
    void callWithABIPost(uint32_t stackAdjust, MoveOp::Type result);

  public:
    // Emits a call to a C/C++ function, resolving all argument moves.
    void callWithABI(void* fun, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(AsmJSImmPtr imm, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(const Address& fun, MoveOp::Type result = MoveOp::GENERAL);
    void callWithABI(Register fun, MoveOp::Type result = MoveOp::GENERAL);

    CodeOffsetLabel labelForPatch() {
        return CodeOffsetLabel(nextOffset().getOffset());
    }

    void memIntToValue(Address Source, Address Dest) {
        load32(Source, ScratchRegister);
        storeValue(JSVAL_TYPE_INT32, ScratchRegister, Dest);
    }

    void lea(Operand addr, Register dest) {
        ma_daddu(dest, addr.baseReg(), Imm32(addr.disp()));
    }

    void abiret() {
        as_jr(ra);
        as_nop();
    }

    BufferOffset ma_BoundsCheck(Register bounded) {
        BufferOffset bo = m_buffer.nextOffset();
        ma_liPatchable(bounded, ImmWord(0));
        return bo;
    }

    void moveFloat32(FloatRegister src, FloatRegister dest) {
        as_movs(dest, src);
    }

    void branchPtrInNurseryRange(Condition cond, Register ptr, Register temp, Label* label);
    void branchValueIsNurseryObject(Condition cond, ValueOperand value, Register temp,
                                    Label* label);

    void loadAsmJSActivation(Register dest) {
        loadPtr(Address(GlobalReg, AsmJSActivationGlobalDataOffset - AsmJSGlobalRegBias), dest);
    }
    void loadAsmJSHeapRegisterFromGlobalData() {
        MOZ_ASSERT(Imm16::IsInSignedRange(AsmJSHeapGlobalDataOffset - AsmJSGlobalRegBias));
        loadPtr(Address(GlobalReg, AsmJSHeapGlobalDataOffset - AsmJSGlobalRegBias), HeapReg);
    }

    // Instrumentation for entering and leaving the profiler.
    void profilerEnterFrame(Register framePtr, Register scratch);
    void profilerExitFrame();
};

typedef MacroAssemblerMIPS64Compat MacroAssemblerSpecific;

} // namespace jit
} // namespace js

#endif /* jit_mips64_MacroAssembler_mips64_h */
