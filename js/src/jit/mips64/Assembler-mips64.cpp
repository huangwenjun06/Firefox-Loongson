/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 4 -*-
 * vim: set ts=8 sts=4 et sw=4 tw=99:
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "jit/mips64/Assembler-mips64.h"

#include "mozilla/DebugOnly.h"
#include "mozilla/MathAlgorithms.h"

#include "jscompartment.h"
#include "jsutil.h"

#include "gc/Marking.h"
#include "jit/ExecutableAllocator.h"
#include "jit/JitCompartment.h"

using mozilla::DebugOnly;

using namespace js;
using namespace js::jit;

ABIArgGenerator::ABIArgGenerator()
  : usedArgSlots_(0),
    firstArgFloat(false),
    current_()
{}

ABIArg
ABIArgGenerator::next(MIRType type)
{
    switch (type) {
      case MIRType_Int32:
      case MIRType_Pointer: {
        Register destReg;
        if (GetIntArgReg(usedArgSlots_, &destReg))
            current_ = ABIArg(destReg);
        else
            current_ = ABIArg(GetArgStackDisp(usedArgSlots_));
        usedArgSlots_++;
        break;
      }
      case MIRType_Float32:
      case MIRType_Double: {
        FloatRegister destFReg;
        FloatRegister::ContentType contentType;
        if (!usedArgSlots_)
            firstArgFloat = true;
        contentType = (type == MIRType_Double) ?
            FloatRegisters::Double : FloatRegisters::Single;
        if (GetFloatArgReg(usedArgSlots_, &destFReg))
            current_ = ABIArg(FloatRegister(destFReg.id(), contentType));
        else
            current_ = ABIArg(GetArgStackDisp(usedArgSlots_));
        usedArgSlots_++;
        break;
      }
      default:
        MOZ_CRASH("Unexpected argument type");
    }
    return current_;
}

const Register ABIArgGenerator::NonArgReturnReg0 = t0;
const Register ABIArgGenerator::NonArgReturnReg1 = t1;
const Register ABIArgGenerator::NonArg_VolatileReg = v0;
const Register ABIArgGenerator::NonReturn_VolatileReg0 = a0;
const Register ABIArgGenerator::NonReturn_VolatileReg1 = a1;

// Encode a standard register when it is being used as rd, the rs, and
// an extra register(rt). These should never be called with an InvalidReg.
uint32_t
js::jit::RS(Register r)
{
    MOZ_ASSERT((r.code() & ~RegMask) == 0);
    return r.code() << RSShift;
}

uint32_t
js::jit::RT(Register r)
{
    MOZ_ASSERT((r.code() & ~RegMask) == 0);
    return r.code() << RTShift;
}

uint32_t
js::jit::RT(FloatRegister r)
{
    MOZ_ASSERT(r.id() < FloatRegisters::TotalPhys);
    return r.id() << RTShift;
}

uint32_t
js::jit::RD(Register r)
{
    MOZ_ASSERT((r.code() & ~RegMask) == 0);
    return r.code() << RDShift;
}

uint32_t
js::jit::RD(FloatRegister r)
{
    MOZ_ASSERT(r.id() < FloatRegisters::TotalPhys);
    return r.id() << RDShift;
}

uint32_t
js::jit::SA(uint32_t value)
{
    MOZ_ASSERT(value < 32);
    return value << SAShift;
}

uint32_t
js::jit::SA(FloatRegister r)
{
    MOZ_ASSERT(r.id() < FloatRegisters::TotalPhys);
    return r.id() << SAShift;
}

Register
js::jit::toRS(Instruction& i)
{
    return Register::FromCode((i.encode() & RSMask ) >> RSShift);
}

Register
js::jit::toRT(Instruction& i)
{
    return Register::FromCode((i.encode() & RTMask ) >> RTShift);
}

Register
js::jit::toRD(Instruction& i)
{
    return Register::FromCode((i.encode() & RDMask ) >> RDShift);
}

Register
js::jit::toR(Instruction& i)
{
    return Register::FromCode(i.encode() & RegMask);
}

void
InstImm::extractImm16(BOffImm16* dest)
{
    *dest = BOffImm16(*this);
}

// Used to patch jumps created by MacroAssemblerMIPS64Compat::jumpWithPatch.
void
jit::PatchJump(CodeLocationJump& jump_, CodeLocationLabel label)
{
    Instruction* inst = (Instruction*)jump_.raw();

    Assembler::UpdateLoad64Value(inst, (uint64_t)label.raw());

    AutoFlushICache::flush(uintptr_t(inst), 24);
}

// For more infromation about backedges look at comment in
// MacroAssemblerMIPS64Compat::backedgeJump()
void
jit::PatchBackedge(CodeLocationJump& jump, CodeLocationLabel label,
                   JitRuntime::BackedgeTarget target)
{
    uintptr_t sourceAddr = (uintptr_t)jump.raw();
    uintptr_t targetAddr = (uintptr_t)label.raw();
    InstImm* branch = (InstImm*)jump.raw();

    MOZ_ASSERT(branch->extractOpcode() == (uint32_t(op_beq) >> OpcodeShift));

    if (BOffImm16::IsInRange(targetAddr - sourceAddr)) {
        branch->setBOffImm16(BOffImm16(targetAddr - sourceAddr));
    } else {
        if (target == JitRuntime::BackedgeLoopHeader) {
            Instruction* inst = &branch[1];
            Assembler::UpdateLoad64Value(inst, targetAddr);
            // Jump to first ori. The lui will be executed in delay slot.
            branch->setBOffImm16(BOffImm16(2 * sizeof(uint32_t)));
        } else {
            Instruction* inst = &branch[8];
            Assembler::UpdateLoad64Value(inst, targetAddr);
            // Jump to first ori of interrupt loop.
            branch->setBOffImm16(BOffImm16(8 * sizeof(uint32_t)));
        }
    }
}

void
Assembler::finish()
{
    MOZ_ASSERT(!isFinished);
    isFinished = true;
}

void
Assembler::executableCopy(uint8_t* buffer)
{
    MOZ_ASSERT(isFinished);
    m_buffer.executableCopy(buffer);

    // Patch all long jumps during code copy.
    for (size_t i = 0; i < longJumps_.length(); i++) {
        Instruction* inst = (Instruction*) ((uintptr_t)buffer + longJumps_[i]);

        uint64_t value = ExtractLoad64Value(inst);
        UpdateLoad64Value(inst, (uint64_t)buffer + value);
    }

    AutoFlushICache::setRange(uintptr_t(buffer), m_buffer.size());
}

uint32_t
Assembler::actualOffset(uint32_t off_) const
{
    return off_;
}

uint32_t
Assembler::actualIndex(uint32_t idx_) const
{
    return idx_;
}

uint8_t*
Assembler::PatchableJumpAddress(JitCode* code, uint32_t pe_)
{
    return code->raw() + pe_;
}

class RelocationIterator
{
    CompactBufferReader reader_;
    // offset in bytes
    uint32_t offset_;

  public:
    RelocationIterator(CompactBufferReader& reader)
      : reader_(reader)
    { }

    bool read() {
        if (!reader_.more())
            return false;
        offset_ = reader_.readUnsigned();
        return true;
    }

    uint32_t offset() const {
        return offset_;
    }
};

uintptr_t
Assembler::GetPointer(uint8_t* instPtr)
{
    Instruction* inst = (Instruction*)instPtr;
    return Assembler::ExtractLoad64Value(inst);
}

static JitCode *
CodeFromJump(Instruction* jump)
{
    uint8_t* target = (uint8_t*)Assembler::ExtractLoad64Value(jump);
    return JitCode::FromExecutable(target);
}

void
Assembler::TraceJumpRelocations(JSTracer* trc, JitCode* code, CompactBufferReader& reader)
{
    RelocationIterator iter(reader);
    while (iter.read()) {
        JitCode* child = CodeFromJump((Instruction*)(code->raw() + iter.offset()));
        TraceManuallyBarrieredEdge(trc, &child, "rel32");
    }
}

static void
TraceOneDataRelocation(JSTracer* trc, Instruction* inst)
{
    void* ptr = (void*)Assembler::ExtractLoad64Value(inst);
    void* prior = ptr;

    // All pointers on MIPS64 will have the top bits cleared. If those bits
    // are not cleared, this must be a Value.
    uintptr_t word = reinterpret_cast<uintptr_t>(ptr);
    if (word >> JSVAL_TAG_SHIFT) {
        jsval_layout layout;
        layout.asBits = word;
        Value v = IMPL_TO_JSVAL(layout);
        TraceManuallyBarrieredEdge(trc, &v, "ion-masm-value");
        ptr = (void*)JSVAL_TO_IMPL(v).asBits;
    } else {
        // The low bit shouldn't be set. If it is, we probably got a dummy
        // pointer inserted by CodeGenerator::visitNurseryObject, but we
        // shouldn't be able to trigger GC before those are patched to their
        // real values.
        MOZ_ASSERT(!(uintptr_t(ptr) & 0x1));

        // No barrier needed since these are constants.
        gc::TraceManuallyBarrieredGenericPointerEdge(trc, reinterpret_cast<gc::Cell**>(&ptr),
                                                     "ion-masm-ptr");
    }

    if (ptr != prior) {
        Assembler::UpdateLoad64Value(inst, uint64_t(ptr));
        AutoFlushICache::flush(uintptr_t(inst), 24);
    }
}

static void
TraceDataRelocations(JSTracer* trc, uint8_t* buffer, CompactBufferReader& reader)
{
    while (reader.more()) {
        size_t offset = reader.readUnsigned();
        Instruction* inst = (Instruction*)(buffer + offset);
        TraceOneDataRelocation(trc, inst);
    }
}

static void
TraceDataRelocations(JSTracer* trc, MIPSBuffer* buffer, CompactBufferReader& reader)
{
    while (reader.more()) {
        BufferOffset bo (reader.readUnsigned());
        MIPSBuffer::AssemblerBufferInstIterator iter(bo, buffer);
        TraceOneDataRelocation(trc, iter.cur());
    }
}

void
Assembler::TraceDataRelocations(JSTracer* trc, JitCode* code, CompactBufferReader& reader)
{
    ::TraceDataRelocations(trc, code->raw(), reader);
}

void
Assembler::FixupNurseryObjects(JSContext* cx, JitCode* code, CompactBufferReader& reader,
                               const ObjectVector& nurseryObjects)
{
    MOZ_ASSERT(!nurseryObjects.empty());

    uint8_t* buffer = code->raw();
    bool hasNurseryPointers = false;

    while (reader.more()) {
        size_t offset = reader.readUnsigned();
        Instruction* inst = (Instruction*)(buffer + offset);

        void* ptr = (void*)Assembler::ExtractLoad64Value(inst);
        uintptr_t word = uintptr_t(ptr);

        if (word >> JSVAL_TAG_SHIFT)
            continue; // This is a Value.

        if (!(word & 0x1))
            continue;

        uint32_t index = word >> 1;
        JSObject* obj = nurseryObjects[index];

        Assembler::UpdateLoad64Value(inst, uint64_t(obj));
        AutoFlushICache::flush(uintptr_t(inst), 24);

        // Either all objects are still in the nursery, or all objects are
        // tenured.
        MOZ_ASSERT_IF(hasNurseryPointers, IsInsideNursery(obj));

        if (!hasNurseryPointers && IsInsideNursery(obj))
            hasNurseryPointers = true;
    }

    if (hasNurseryPointers)
        cx->runtime()->gc.storeBuffer.putWholeCellFromMainThread(code);
}

void
Assembler::copyJumpRelocationTable(uint8_t* dest)
{
    if (jumpRelocations_.length())
        memcpy(dest, jumpRelocations_.buffer(), jumpRelocations_.length());
}

void
Assembler::copyDataRelocationTable(uint8_t* dest)
{
    if (dataRelocations_.length())
        memcpy(dest, dataRelocations_.buffer(), dataRelocations_.length());
}

void
Assembler::copyPreBarrierTable(uint8_t* dest)
{
    if (preBarriers_.length())
        memcpy(dest, preBarriers_.buffer(), preBarriers_.length());
}

void
Assembler::trace(JSTracer* trc)
{
    for (size_t i = 0; i < jumps_.length(); i++) {
        RelativePatch& rp = jumps_[i];
        if (rp.kind == Relocation::JITCODE) {
            JitCode* code = JitCode::FromExecutable((uint8_t*)rp.target);
            TraceManuallyBarrieredEdge(trc, &code, "masmrel32");
            MOZ_ASSERT(code == JitCode::FromExecutable((uint8_t*)rp.target));
        }
    }
    if (dataRelocations_.length()) {
        CompactBufferReader reader(dataRelocations_);
        ::TraceDataRelocations(trc, &m_buffer, reader);
    }
}

void
Assembler::processCodeLabels(uint8_t* rawCode)
{
    for (size_t i = 0; i < codeLabels_.length(); i++) {
        CodeLabel label = codeLabels_[i];
        Bind(rawCode, label.dest(), rawCode + actualOffset(label.src()->offset()));
    }
}

int64_t
Assembler::ExtractCodeLabelOffset(uint8_t* code) {
    Instruction* inst = (Instruction*)code;
    return Assembler::ExtractLoad64Value(inst);
}

void
Assembler::Bind(uint8_t* rawCode, AbsoluteLabel* label, const void* address)
{
    if (label->used()) {
        int64_t src = label->offset();
        do {
            Instruction* inst = (Instruction*) (rawCode + src);
            uint64_t next = Assembler::ExtractLoad64Value(inst);
            Assembler::UpdateLoad64Value(inst, (uint64_t)address);
            src = next;
        } while (src != AbsoluteLabel::INVALID_OFFSET);
    }
    label->bind();
}

Assembler::Condition
Assembler::InvertCondition(Condition cond)
{
    switch (cond) {
      case Equal:
        return NotEqual;
      case NotEqual:
        return Equal;
      case Zero:
        return NonZero;
      case NonZero:
        return Zero;
      case LessThan:
        return GreaterThanOrEqual;
      case LessThanOrEqual:
        return GreaterThan;
      case GreaterThan:
        return LessThanOrEqual;
      case GreaterThanOrEqual:
        return LessThan;
      case Above:
        return BelowOrEqual;
      case AboveOrEqual:
        return Below;
      case Below:
        return AboveOrEqual;
      case BelowOrEqual:
        return Above;
      case Signed:
        return NotSigned;
      case NotSigned:
        return Signed;
      default:
        MOZ_CRASH("unexpected condition");
    }
}

Assembler::DoubleCondition
Assembler::InvertCondition(DoubleCondition cond)
{
    switch (cond) {
      case DoubleOrdered:
        return DoubleUnordered;
      case DoubleEqual:
        return DoubleNotEqualOrUnordered;
      case DoubleNotEqual:
        return DoubleEqualOrUnordered;
      case DoubleGreaterThan:
        return DoubleLessThanOrEqualOrUnordered;
      case DoubleGreaterThanOrEqual:
        return DoubleLessThanOrUnordered;
      case DoubleLessThan:
        return DoubleGreaterThanOrEqualOrUnordered;
      case DoubleLessThanOrEqual:
        return DoubleGreaterThanOrUnordered;
      case DoubleUnordered:
        return DoubleOrdered;
      case DoubleEqualOrUnordered:
        return DoubleNotEqual;
      case DoubleNotEqualOrUnordered:
        return DoubleEqual;
      case DoubleGreaterThanOrUnordered:
        return DoubleLessThanOrEqual;
      case DoubleGreaterThanOrEqualOrUnordered:
        return DoubleLessThan;
      case DoubleLessThanOrUnordered:
        return DoubleGreaterThanOrEqual;
      case DoubleLessThanOrEqualOrUnordered:
        return DoubleGreaterThan;
      default:
        MOZ_CRASH("unexpected condition");
    }
}

BOffImm16::BOffImm16(InstImm inst)
  : data(inst.encode() & Imm16Mask)
{
}

bool
Assembler::oom() const
{
    return AssemblerShared::oom() ||
           m_buffer.oom() ||
           jumpRelocations_.oom() ||
           dataRelocations_.oom() ||
           preBarriers_.oom();
}

void
Assembler::addCodeLabel(CodeLabel label)
{
    propagateOOM(codeLabels_.append(label));
}

// Size of the instruction stream, in bytes.
size_t
Assembler::size() const
{
    return m_buffer.size();
}

// Size of the relocation table, in bytes.
size_t
Assembler::jumpRelocationTableBytes() const
{
    return jumpRelocations_.length();
}

size_t
Assembler::dataRelocationTableBytes() const
{
    return dataRelocations_.length();
}

size_t
Assembler::preBarrierTableBytes() const
{
    return preBarriers_.length();
}

// Size of the data table, in bytes.
size_t
Assembler::bytesNeeded() const
{
    return size() +
           jumpRelocationTableBytes() +
           dataRelocationTableBytes() +
           preBarrierTableBytes();
}

// write a blob of binary into the instruction stream
BufferOffset
Assembler::writeInst(uint32_t x, uint32_t* dest)
{
    if (dest == nullptr)
        return m_buffer.putInt(x);

    WriteInstStatic(x, dest);
    return BufferOffset();
}

void
Assembler::WriteInstStatic(uint32_t x, uint32_t* dest)
{
    MOZ_ASSERT(dest != nullptr);
    *dest = x;
}

BufferOffset
Assembler::haltingAlign(int alignment)
{
    // TODO: Implement a proper halting align.
    nopAlign(alignment);
}

BufferOffset
Assembler::nopAlign(int alignment)
{
    BufferOffset ret;
    MOZ_ASSERT(m_buffer.isAligned(4));
    if (alignment == 8) {
        if (!m_buffer.isAligned(alignment)) {
            BufferOffset tmp = as_nop();
            if (!ret.assigned())
                ret = tmp;
        }
    } else {
        MOZ_ASSERT((alignment & (alignment - 1)) == 0);
        while (size() & (alignment - 1)) {
            BufferOffset tmp = as_nop();
            if (!ret.assigned())
                ret = tmp;
        }
    }
    return ret;
}

BufferOffset
Assembler::as_nop()
{
    return writeInst(op_special | ff_sll);
}

// Logical operations.
BufferOffset
Assembler::as_and(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_and).encode());
}

BufferOffset
Assembler::as_or(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_or).encode());
}

BufferOffset
Assembler::as_xor(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_xor).encode());
}

BufferOffset
Assembler::as_nor(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_nor).encode());
}

BufferOffset
Assembler::as_andi(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInUnsignedRange(j));
    return writeInst(InstImm(op_andi, rs, rd, Imm16(j)).encode());
}

BufferOffset
Assembler::as_ori(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInUnsignedRange(j));
    return writeInst(InstImm(op_ori, rs, rd, Imm16(j)).encode());
}

BufferOffset
Assembler::as_xori(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInUnsignedRange(j));
    return writeInst(InstImm(op_xori, rs, rd, Imm16(j)).encode());
}

// Branch and jump instructions
BufferOffset
Assembler::as_bal(BOffImm16 off)
{
    BufferOffset bo = writeInst(InstImm(op_regimm, zero, rt_bgezal, off).encode());
    return bo;
}

BufferOffset
Assembler::as_b(BOffImm16 off)
{
    BufferOffset bo = writeInst(InstImm(op_beq, zero, zero, off).encode());
    return bo;
}

InstImm
Assembler::getBranchCode(JumpOrCall jumpOrCall)
{
    if (jumpOrCall == BranchIsCall)
        return InstImm(op_regimm, zero, rt_bgezal, BOffImm16(0));

    return InstImm(op_beq, zero, zero, BOffImm16(0));
}

InstImm
Assembler::getBranchCode(Register s, Register t, Condition c)
{
    MOZ_ASSERT(c == Assembler::Equal || c == Assembler::NotEqual);
    return InstImm(c == Assembler::Equal ? op_beq : op_bne, s, t, BOffImm16(0));
}

InstImm
Assembler::getBranchCode(Register s, Condition c)
{
    switch (c) {
      case Assembler::Equal:
      case Assembler::Zero:
      case Assembler::BelowOrEqual:
        return InstImm(op_beq, s, zero, BOffImm16(0));
      case Assembler::NotEqual:
      case Assembler::NonZero:
      case Assembler::Above:
        return InstImm(op_bne, s, zero, BOffImm16(0));
      case Assembler::GreaterThan:
        return InstImm(op_bgtz, s, zero, BOffImm16(0));
      case Assembler::GreaterThanOrEqual:
      case Assembler::NotSigned:
        return InstImm(op_regimm, s, rt_bgez, BOffImm16(0));
      case Assembler::LessThan:
      case Assembler::Signed:
        return InstImm(op_regimm, s, rt_bltz, BOffImm16(0));
      case Assembler::LessThanOrEqual:
        return InstImm(op_blez, s, zero, BOffImm16(0));
      default:
        MOZ_CRASH("Condition not supported.");
    }
}

InstImm
Assembler::getBranchCode(FloatTestKind testKind, FPConditionBit fcc)
{
    MOZ_ASSERT(!(fcc && FccMask));
    uint32_t rtField = ((testKind == TestForTrue ? 1 : 0) | (fcc << FccShift)) << RTShift;

    return InstImm(op_cop1, rs_bc1, rtField, BOffImm16(0));
}

BufferOffset
Assembler::as_j(JOffImm26 off)
{
    BufferOffset bo = writeInst(InstJump(op_j, off).encode());
    return bo;
}
BufferOffset
Assembler::as_jal(JOffImm26 off)
{
    BufferOffset bo = writeInst(InstJump(op_jal, off).encode());
    return bo;
}

BufferOffset
Assembler::as_jr(Register rs)
{
    BufferOffset bo = writeInst(InstReg(op_special, rs, zero, zero, ff_jr).encode());
    return bo;
}
BufferOffset
Assembler::as_jalr(Register rs)
{
    BufferOffset bo = writeInst(InstReg(op_special, rs, zero, ra, ff_jalr).encode());
    return bo;
}


// Arithmetic instructions
BufferOffset
Assembler::as_addu(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_addu).encode());
}

BufferOffset
Assembler::as_addiu(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(j));
    return writeInst(InstImm(op_addiu, rs, rd, Imm16(j)).encode());
}

BufferOffset
Assembler::as_daddu(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_daddu).encode());
}

BufferOffset
Assembler::as_daddiu(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(j));
    return writeInst(InstImm(op_daddiu, rs, rd, Imm16(j)).encode());
}

BufferOffset
Assembler::as_subu(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_subu).encode());
}

BufferOffset
Assembler::as_dsubu(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_dsubu).encode());
}

BufferOffset
Assembler::as_mul(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special2, rs, rt, rd, ff_mul).encode());
}

BufferOffset
Assembler::as_mult(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_mult).encode());
}

BufferOffset
Assembler::as_multu(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_multu).encode());
}

BufferOffset
Assembler::as_dmult(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_dmult).encode());
}

BufferOffset
Assembler::as_dmultu(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_dmultu).encode());
}

BufferOffset
Assembler::as_div(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_div).encode());
}

BufferOffset
Assembler::as_divu(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_divu).encode());
}

BufferOffset
Assembler::as_ddiv(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_ddiv).encode());
}

BufferOffset
Assembler::as_ddivu(Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, ff_ddivu).encode());
}

BufferOffset
Assembler::as_lui(Register rd, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInUnsignedRange(j));
    return writeInst(InstImm(op_lui, zero, rd, Imm16(j)).encode());
}

// Shift instructions
BufferOffset
Assembler::as_sll(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_sll).encode());
}

BufferOffset
Assembler::as_dsll(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_dsll).encode());
}

BufferOffset
Assembler::as_dsll32(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(31 < sa && sa < 64);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa - 32, ff_dsll32).encode());
}

BufferOffset
Assembler::as_sllv(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_sllv).encode());
}

BufferOffset
Assembler::as_dsllv(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_dsllv).encode());
}

BufferOffset
Assembler::as_srl(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_srl).encode());
}

BufferOffset
Assembler::as_dsrl(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_dsrl).encode());
}

BufferOffset
Assembler::as_dsrl32(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(31 < sa && sa < 64);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa - 32, ff_dsrl32).encode());
}

BufferOffset
Assembler::as_srlv(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_srlv).encode());
}

BufferOffset
Assembler::as_dsrlv(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_dsrlv).encode());
}

BufferOffset
Assembler::as_sra(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_sra).encode());
}

BufferOffset
Assembler::as_dsra(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa, ff_dsra).encode());
}

BufferOffset
Assembler::as_dsra32(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(31 < sa && sa < 64);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa - 32, ff_dsra32).encode());
}

BufferOffset
Assembler::as_srav(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_srav).encode());
}

BufferOffset
Assembler::as_dsrav(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_dsrav).encode());
}

BufferOffset
Assembler::as_drotr(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(sa < 32);
    return writeInst(InstReg(op_special, rs_one, rt, rd, sa, ff_dsrl).encode());
}

BufferOffset
Assembler::as_drotr32(Register rd, Register rt, uint16_t sa)
{
    MOZ_ASSERT(31 < sa && sa < 64);
    return writeInst(InstReg(op_special, rs_zero, rt, rd, sa - 32, ff_dsrl32).encode());
}

BufferOffset
Assembler::as_drotrv(Register rd, Register rt, Register rs)
{
    return writeInst(InstReg(op_special, rs, rt, rd, 1, ff_dsrlv).encode());
}

// Load and store instructions
BufferOffset
Assembler::as_lb(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lb, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lbu(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lbu, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lh(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lh, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lhu(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lhu, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lw(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lw, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lwu(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lwu, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lwl(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lwl, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_lwr(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_lwr, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ll(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_ll, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ld(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_ld, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ldl(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_ldl, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ldr(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_ldr, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sb(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sb, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sh(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sh, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sw(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sw, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_swl(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_swl, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_swr(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_swr, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sc(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sc, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sd(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sd, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sdl(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sdl, rs, rd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sdr(Register rd, Register rs, int16_t off)
{
    return writeInst(InstImm(op_sdr, rs, rd, Imm16(off)).encode());
}

// Move from HI/LO register.
BufferOffset
Assembler::as_mfhi(Register rd)
{
    return writeInst(InstReg(op_special, rd, ff_mfhi).encode());
}

BufferOffset
Assembler::as_mflo(Register rd)
{
    return writeInst(InstReg(op_special, rd, ff_mflo).encode());
}

// Set on less than.
BufferOffset
Assembler::as_slt(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_slt).encode());
}

BufferOffset
Assembler::as_sltu(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_sltu).encode());
}

BufferOffset
Assembler::as_slti(Register rd, Register rs, int32_t j)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(j));
    return writeInst(InstImm(op_slti, rs, rd, Imm16(j)).encode());
}

BufferOffset
Assembler::as_sltiu(Register rd, Register rs, uint32_t j)
{
    MOZ_ASSERT(Imm16::IsInUnsignedRange(j));
    return writeInst(InstImm(op_sltiu, rs, rd, Imm16(j)).encode());
}

// Conditional move.
BufferOffset
Assembler::as_movz(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_movz).encode());
}

BufferOffset
Assembler::as_movn(Register rd, Register rs, Register rt)
{
    return writeInst(InstReg(op_special, rs, rt, rd, ff_movn).encode());
}

BufferOffset
Assembler::as_movt(Register rd, Register rs, uint16_t cc)
{
    Register rt;
    rt = Register::FromCode((cc & 0x7) << 2 | 1);
    return writeInst(InstReg(op_special, rs, rt, rd, ff_movci).encode());
}

BufferOffset
Assembler::as_movf(Register rd, Register rs, uint16_t cc)
{
    Register rt;
    rt = Register::FromCode((cc & 0x7) << 2 | 0);
    return writeInst(InstReg(op_special, rs, rt, rd, ff_movci).encode());
}

// Bit twiddling.
BufferOffset
Assembler::as_clz(Register rd, Register rs)
{
    return writeInst(InstReg(op_special2, rs, rd, rd, ff_clz).encode());
}

BufferOffset
Assembler::as_dclz(Register rd, Register rs)
{
    return writeInst(InstReg(op_special2, rs, rd, rd, ff_dclz).encode());
}

BufferOffset
Assembler::as_ins(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size != 0 && size <= 32 && pos + size != 0 && pos + size <= 32);
    Register rd;
    rd = Register::FromCode(pos + size - 1);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_ins).encode());
}

BufferOffset
Assembler::as_dins(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size != 0 && size <= 32 && pos + size != 0 && pos + size <= 32);
    Register rd;
    rd = Register::FromCode(pos + size - 1);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_dins).encode());
}

BufferOffset
Assembler::as_dinsm(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size >= 2 && size <= 64 && pos + size > 32 && pos + size <= 64);
    Register rd;
    rd = Register::FromCode(pos + size - 1 - 32);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_dinsm).encode());
}

BufferOffset
Assembler::as_dinsu(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos >= 32 && pos < 64 && size >= 1 && size <= 32 && pos + size > 32 && pos + size <= 64);
    Register rd;
    rd = Register::FromCode(pos + size - 1 - 32);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos - 32, ff_dinsu).encode());
}

BufferOffset
Assembler::as_ext(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size != 0 && size <= 32 && pos + size != 0 && pos + size <= 32);
    Register rd;
    rd = Register::FromCode(size - 1);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_ext).encode());
}

BufferOffset
Assembler::as_dext(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size != 0 && size <= 32 && pos + size != 0 && pos + size <= 63);
    Register rd;
    rd = Register::FromCode(size - 1);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_dext).encode());
}

BufferOffset
Assembler::as_dextm(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos < 32 && size > 32 && size <= 64 && pos + size > 32 && pos + size <= 64);
    Register rd;
    rd = Register::FromCode(size - 1 - 32);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos, ff_dextm).encode());
}

BufferOffset
Assembler::as_dextu(Register rt, Register rs, uint16_t pos, uint16_t size)
{
    MOZ_ASSERT(pos >= 32 && pos < 64 && size != 0 && size <= 32 && pos + size > 32 && pos + size <= 64);
    Register rd;
    rd = Register::FromCode(size - 1);
    return writeInst(InstReg(op_special3, rs, rt, rd, pos - 32, ff_dextu).encode());
}

// Sign extend
BufferOffset
Assembler::as_seb(Register rd, Register rt)
{
    return writeInst(InstReg(op_special3, zero, rt, rd, 16, ff_bshfl).encode());
}

BufferOffset
Assembler::as_seh(Register rd, Register rt)
{
    return writeInst(InstReg(op_special3, zero, rt, rd, 24, ff_bshfl).encode());
}

// FP instructions
BufferOffset
Assembler::as_ld(FloatRegister fd, Register base, int32_t off)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(off));
    return writeInst(InstImm(op_ldc1, base, fd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_sd(FloatRegister fd, Register base, int32_t off)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(off));
    return writeInst(InstImm(op_sdc1, base, fd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ls(FloatRegister fd, Register base, int32_t off)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(off));
    return writeInst(InstImm(op_lwc1, base, fd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_ss(FloatRegister fd, Register base, int32_t off)
{
    MOZ_ASSERT(Imm16::IsInSignedRange(off));
    return writeInst(InstImm(op_swc1, base, fd, Imm16(off)).encode());
}

BufferOffset
Assembler::as_movs(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_mov_fmt).encode());
}

BufferOffset
Assembler::as_movd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_mov_fmt).encode());
}

BufferOffset
Assembler::as_mtc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_mtc1, rt, fs).encode());
}

BufferOffset
Assembler::as_mfc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_mfc1, rt, fs).encode());
}

BufferOffset
Assembler::as_mthc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_mthc1, rt, fs).encode());
}

BufferOffset
Assembler::as_mfhc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_mfhc1, rt, fs).encode());
}

BufferOffset
Assembler::as_dmtc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_dmtc1, rt, fs).encode());
}

BufferOffset
Assembler::as_dmfc1(Register rt, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_dmfc1, rt, fs).encode());
}

// FP convert instructions
BufferOffset
Assembler::as_ceilws(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_ceil_w_fmt).encode());
}

BufferOffset
Assembler::as_floorws(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_floor_w_fmt).encode());
}

BufferOffset
Assembler::as_roundws(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_round_w_fmt).encode());
}

BufferOffset
Assembler::as_truncws(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_trunc_w_fmt).encode());
}

BufferOffset
Assembler::as_ceilwd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_ceil_w_fmt).encode());
}

BufferOffset
Assembler::as_floorwd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_floor_w_fmt).encode());
}

BufferOffset
Assembler::as_roundwd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_round_w_fmt).encode());
}

BufferOffset
Assembler::as_truncwd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_trunc_w_fmt).encode());
}

BufferOffset
Assembler::as_cvtds(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_cvt_d_fmt).encode());
}

BufferOffset
Assembler::as_cvtdw(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_w, zero, fs, fd, ff_cvt_d_fmt).encode());
}

BufferOffset
Assembler::as_cvtsd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_cvt_s_fmt).encode());
}

BufferOffset
Assembler::as_cvtsw(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_w, zero, fs, fd, ff_cvt_s_fmt).encode());
}

BufferOffset
Assembler::as_cvtwd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_cvt_w_fmt).encode());
}

BufferOffset
Assembler::as_cvtws(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_cvt_w_fmt).encode());
}

// FP arithmetic instructions
BufferOffset
Assembler::as_adds(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_s, ft, fs, fd, ff_add_fmt).encode());
}

BufferOffset
Assembler::as_addd(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_d, ft, fs, fd, ff_add_fmt).encode());
}

BufferOffset
Assembler::as_subs(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_s, ft, fs, fd, ff_sub_fmt).encode());
}

BufferOffset
Assembler::as_subd(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_d, ft, fs, fd, ff_sub_fmt).encode());
}

BufferOffset
Assembler::as_abss(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_abs_fmt).encode());
}

BufferOffset
Assembler::as_absd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_abs_fmt).encode());
}

BufferOffset
Assembler::as_negs(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_neg_fmt).encode());
}

BufferOffset
Assembler::as_negd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_neg_fmt).encode());
}

BufferOffset
Assembler::as_muls(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_s, ft, fs, fd, ff_mul_fmt).encode());
}

BufferOffset
Assembler::as_muld(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_d, ft, fs, fd, ff_mul_fmt).encode());
}

BufferOffset
Assembler::as_divs(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_s, ft, fs, fd, ff_div_fmt).encode());
}

BufferOffset
Assembler::as_divd(FloatRegister fd, FloatRegister fs, FloatRegister ft)
{
    return writeInst(InstReg(op_cop1, rs_d, ft, fs, fd, ff_div_fmt).encode());
}

BufferOffset
Assembler::as_sqrts(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_s, zero, fs, fd, ff_sqrt_fmt).encode());
}

BufferOffset
Assembler::as_sqrtd(FloatRegister fd, FloatRegister fs)
{
    return writeInst(InstReg(op_cop1, rs_d, zero, fs, fd, ff_sqrt_fmt).encode());
}

// FP compare instructions
BufferOffset
Assembler::as_cf(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_f_fmt).encode());
}

BufferOffset
Assembler::as_cun(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_un_fmt).encode());
}

BufferOffset
Assembler::as_ceq(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_eq_fmt).encode());
}

BufferOffset
Assembler::as_cueq(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_ueq_fmt).encode());
}

BufferOffset
Assembler::as_colt(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_olt_fmt).encode());
}

BufferOffset
Assembler::as_cult(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_ult_fmt).encode());
}

BufferOffset
Assembler::as_cole(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_ole_fmt).encode());
}

BufferOffset
Assembler::as_cule(FloatFormat fmt, FloatRegister fs, FloatRegister ft, FPConditionBit fcc)
{
    RSField rs = fmt == DoubleFloat ? rs_d : rs_s;
    return writeInst(InstReg(op_cop1, rs, ft, fs, fcc << FccShift, ff_c_ule_fmt).encode());
}


void
Assembler::bind(Label* label, BufferOffset boff)
{
    // If our caller didn't give us an explicit target to bind to
    // then we want to bind to the location of the next instruction
    BufferOffset dest = boff.assigned() ? boff : nextOffset();
    if (label->used()) {
        int32_t next;

        // A used label holds a link to branch that uses it.
        BufferOffset b(label);
        do {
            Instruction* inst = editSrc(b);

            // Second word holds a pointer to the next branch in label's chain.
            next = inst[1].encode();
            bind(reinterpret_cast<InstImm*>(inst), b.getOffset(), dest.getOffset());

            b = BufferOffset(next);
        } while (next != LabelBase::INVALID_OFFSET);
    }
    label->bind(dest.getOffset());
}

void
Assembler::bind(InstImm* inst, uint64_t branch, uint64_t target)
{
    int64_t offset = target - branch;
    InstImm inst_bgezal = InstImm(op_regimm, zero, rt_bgezal, BOffImm16(0));
    InstImm inst_beq = InstImm(op_beq, zero, zero, BOffImm16(0));

    // If encoded offset is 4, then the jump must be short
    if (BOffImm16(inst[0]).decode() == 4) {
        MOZ_ASSERT(BOffImm16::IsInRange(offset));
        inst[0].setBOffImm16(BOffImm16(offset));
        inst[1].makeNop();
        return;
    }

    // Generate the long jump for calls because return address has to be the
    // address after the reserved block.
    if (inst[0].encode() == inst_bgezal.encode()) {
        addLongJump(BufferOffset(branch));
        WriteLoad64Instructions(inst, ScratchRegister, target);
        inst[6] = InstReg(op_special, ScratchRegister, zero, ra, ff_jalr).encode();
        // There is 1 nop after this.
        return;
    }

    if (BOffImm16::IsInRange(offset)) {
        bool conditional = (inst[0].encode() != inst_bgezal.encode() &&
                            inst[0].encode() != inst_beq.encode());

        inst[0].setBOffImm16(BOffImm16(offset));
        inst[1].makeNop();

        // Skip the trailing nops in conditional branches.
        if (conditional) {
            inst[2] = InstImm(op_regimm, zero, rt_bgez, BOffImm16(7 * sizeof(uint32_t))).encode();
            // There are 6 nops after this
        }
        return;
    }

    if (inst[0].encode() == inst_beq.encode()) {
        // Handle long unconditional jump.
        addLongJump(BufferOffset(branch));
        WriteLoad64Instructions(inst, ScratchRegister, target);
        inst[6] = InstReg(op_special, ScratchRegister, zero, zero, ff_jr).encode();
        // There is 1 nop after this.
    } else {
        // Handle long conditional jump.
        inst[0] = invertBranch(inst[0], BOffImm16(9 * sizeof(uint32_t)));
        // No need for a "nop" here because we can clobber scratch.
        addLongJump(BufferOffset(branch + sizeof(uint32_t)));
        WriteLoad64Instructions(&inst[1], ScratchRegister, target);
        inst[7] = InstReg(op_special, ScratchRegister, zero, zero, ff_jr).encode();
        // There is 1 nop after this.
    }
}

void
Assembler::bind(RepatchLabel* label)
{
    BufferOffset dest = nextOffset();
    if (label->used()) {
        // If the label has a use, then change this use to refer to
        // the bound label;
        BufferOffset b(label->offset());
        InstImm* inst1 = (InstImm*)editSrc(b);

        // If first instruction is branch, then this is a loop backedge.
        if (inst1->extractOpcode() == ((uint32_t)op_beq >> OpcodeShift)) {
            // Backedges are short jumps when bound, but can become long
            // when patched.
            uint64_t offset = dest.getOffset() - label->offset();
            MOZ_ASSERT(BOffImm16::IsInRange(offset));
            inst1->setBOffImm16(BOffImm16(offset));
        } else {
            UpdateLoad64Value(inst1, dest.getOffset());
        }

    }
    label->bind(dest.getOffset());
}

void
Assembler::retarget(Label* label, Label* target)
{
    if (label->used()) {
        if (target->bound()) {
            bind(label, BufferOffset(target));
        } else if (target->used()) {
            // The target is not bound but used. Prepend label's branch list
            // onto target's.
            int32_t next;
            BufferOffset labelBranchOffset(label);

            // Find the head of the use chain for label.
            do {
                Instruction* inst = editSrc(labelBranchOffset);

                // Second word holds a pointer to the next branch in chain.
                next = inst[1].encode();
                labelBranchOffset = BufferOffset(next);
            } while (next != LabelBase::INVALID_OFFSET);

            // Then patch the head of label's use chain to the tail of
            // target's use chain, prepending the entire use chain of target.
            Instruction* inst = editSrc(labelBranchOffset);
            int32_t prev = target->use(label->offset());
            inst[1].setData(prev);
        } else {
            // The target is unbound and unused.  We can just take the head of
            // the list hanging off of label, and dump that into target.
            DebugOnly<uint32_t> prev = target->use(label->offset());
            MOZ_ASSERT((int32_t)prev == Label::INVALID_OFFSET);
        }
    }
    label->reset();
}

void dbg_break() {}
void
Assembler::as_break(uint32_t code)
{
    MOZ_ASSERT(code <= MAX_BREAK_CODE);
    writeInst(op_special | code << FunctionBits | ff_break);
}

void
Assembler::as_sync(uint32_t stype)
{
    MOZ_ASSERT(stype <= 31);
    writeInst(InstReg(op_special, zero, zero, zero, stype, ff_sync).encode());
}

uint32_t
Assembler::PatchWrite_NearCallSize()
{
    // Load a 64-bit imm needs 6 instructions, and a jump with a delay slot.
    return (6 + 2) * sizeof(uint32_t);
}

void
Assembler::PatchWrite_NearCall(CodeLocationLabel start, CodeLocationLabel toCall)
{
    Instruction* inst = (Instruction*) start.raw();
    uint8_t* dest = toCall.raw();

    // Overwrite whatever instruction used to be here with a call.
    // Always use long jump for two reasons:
    // - Jump has to be the same size because of PatchWrite_NearCallSize.
    // - Return address has to be at the end of replaced block.
    // Short jump wouldn't be more efficient.
    WriteLoad64Instructions(inst, ScratchRegister, (uint64_t)dest);
    inst[6] = InstReg(op_special, ScratchRegister, zero, ra, ff_jalr);
    inst[7] = InstNOP();

    // Ensure everyone sees the code that was just written into memory.
    AutoFlushICache::flush(uintptr_t(inst), PatchWrite_NearCallSize());
}

uint64_t
Assembler::ExtractLoad64Value(Instruction* inst0)
{
    InstImm* i0 = (InstImm*) inst0;
    InstImm* i1 = (InstImm*) &inst0[1];
    InstImm* i3 = (InstImm*) &inst0[3];
    InstImm* i5 = (InstImm*) &inst0[5];

    MOZ_ASSERT(i0->extractOpcode() == ((uint32_t)op_lui >> OpcodeShift));
    MOZ_ASSERT(i1->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(i3->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(i5->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));

    uint64_t value = (uint64_t(i0->extractImm16Value()) << 48) |
                     (uint64_t(i1->extractImm16Value()) << 32) |
                     (uint64_t(i3->extractImm16Value()) << 16) |
                     uint64_t(i5->extractImm16Value());
    return value;
}

void
Assembler::UpdateLoad64Value(Instruction* inst0, uint64_t value)
{
    Instruction* inst1 = &inst0[1];
    Instruction* inst3 = &inst0[3];
    Instruction* inst5 = &inst0[5];

    MOZ_ASSERT(inst0->extractOpcode() == ((uint32_t)op_lui >> OpcodeShift));
    MOZ_ASSERT(inst1->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(inst3->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(inst5->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));

    ((InstImm*) inst0)->setImm16(Imm16::Upper(Imm32(value >> 32)));
    ((InstImm*) inst1)->setImm16(Imm16::Lower(Imm32(value >> 32)));
    ((InstImm*) inst3)->setImm16(Imm16::Upper(Imm32(value)));
    ((InstImm*) inst5)->setImm16(Imm16::Lower(Imm32(value)));
}

void
Assembler::WriteLoad64Instructions(Instruction* inst0, Register reg, uint64_t value)
{
    Instruction* inst1 = &inst0[1];
    Instruction* inst2 = &inst0[2];
    Instruction* inst3 = &inst0[3];
    Instruction* inst4 = &inst0[4];
    Instruction* inst5 = &inst0[5];

    *inst0 = InstImm(op_lui, zero, reg, Imm16::Upper(Imm32(value >> 32)));
    *inst1 = InstImm(op_ori, reg, reg, Imm16::Lower(Imm32(value >> 32)));
    *inst2 = InstReg(op_special, rs_zero, reg, reg, 16, ff_dsll);
    *inst3 = InstImm(op_ori, reg, reg, Imm16::Upper(Imm32(value)));
    *inst4 = InstReg(op_special, rs_zero, reg, reg, 16, ff_dsll);
    *inst5 = InstImm(op_ori, reg, reg, Imm16::Lower(Imm32(value)));
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, PatchedImmPtr newValue,
                                   PatchedImmPtr expectedValue)
{
    Instruction* inst = (Instruction*) label.raw();

    // Extract old Value
    DebugOnly<uint64_t> value = Assembler::ExtractLoad64Value(inst);
    MOZ_ASSERT(value == uint64_t(expectedValue.value));

    // Replace with new value
    Assembler::UpdateLoad64Value(inst, uint64_t(newValue.value));

    AutoFlushICache::flush(uintptr_t(inst), 24);
}

void
Assembler::PatchDataWithValueCheck(CodeLocationLabel label, ImmPtr newValue, ImmPtr expectedValue)
{
    PatchDataWithValueCheck(label, PatchedImmPtr(newValue.value),
                            PatchedImmPtr(expectedValue.value));
}

// This just stomps over memory with 32 bits of raw data. Its purpose is to
// overwrite the call of JITed code with 32 bits worth of an offset. This will
// is only meant to function on code that has been invalidated, so it should
// be totally safe. Since that instruction will never be executed again, a
// ICache flush should not be necessary
void
Assembler::PatchWrite_Imm32(CodeLocationLabel label, Imm32 imm)
{
    // Raw is going to be the return address.
    uint32_t* raw = (uint32_t*)label.raw();
    // Overwrite the 4 bytes before the return address, which will
    // end up being the call instruction.
    *(raw - 1) = imm.value;
}

void
Assembler::PatchInstructionImmediate(uint8_t* code, PatchedImmPtr imm)
{
    InstImm* inst = (InstImm*)code;
    Assembler::UpdateLoad64Value(inst, (uint64_t)imm.value);
}

uint8_t *
Assembler::NextInstruction(uint8_t* inst_, uint32_t* count)
{
    Instruction* inst = reinterpret_cast<Instruction*>(inst_);
    if (count != nullptr)
        *count += sizeof(Instruction);
    return reinterpret_cast<uint8_t*>(inst->next());
}

// Since there are no pools in MIPS implementation, this should be simple.
Instruction *
Instruction::next()
{
    return this + 1;
}

InstImm Assembler::invertBranch(InstImm branch, BOffImm16 skipOffset)
{
    uint32_t rt = 0;
    Opcode op = (Opcode) (branch.extractOpcode() << OpcodeShift);
    switch(op) {
      case op_beq:
        branch.setBOffImm16(skipOffset);
        branch.setOpcode(op_bne);
        return branch;
      case op_bne:
        branch.setBOffImm16(skipOffset);
        branch.setOpcode(op_beq);
        return branch;
      case op_bgtz:
        branch.setBOffImm16(skipOffset);
        branch.setOpcode(op_blez);
        return branch;
      case op_blez:
        branch.setBOffImm16(skipOffset);
        branch.setOpcode(op_bgtz);
        return branch;
      case op_regimm:
        branch.setBOffImm16(skipOffset);
        rt = branch.extractRT();
        if (rt == (rt_bltz >> RTShift)) {
            branch.setRT(rt_bgez);
            return branch;
        }
        if (rt == (rt_bgez >> RTShift)) {
            branch.setRT(rt_bltz);
            return branch;
        }

        MOZ_CRASH("Error creating long branch.");

      case op_cop1:
        MOZ_ASSERT(branch.extractRS() == rs_bc1 >> RSShift);

        branch.setBOffImm16(skipOffset);
        rt = branch.extractRT();
        if (rt & 0x1)
            branch.setRT((RTField) ((rt & ~0x1) << RTShift));
        else
            branch.setRT((RTField) ((rt | 0x1) << RTShift));
        return branch;
      default:
        MOZ_CRASH("Error creating long branch.");
    }
}

void
Assembler::ToggleToJmp(CodeLocationLabel inst_)
{
    InstImm * inst = (InstImm*)inst_.raw();

    MOZ_ASSERT(inst->extractOpcode() == ((uint32_t)op_andi >> OpcodeShift));
    // We converted beq to andi, so now we restore it.
    inst->setOpcode(op_beq);

    AutoFlushICache::flush(uintptr_t(inst), 4);
}

void
Assembler::ToggleToCmp(CodeLocationLabel inst_)
{
    InstImm * inst = (InstImm*)inst_.raw();

    // toggledJump is allways used for short jumps.
    MOZ_ASSERT(inst->extractOpcode() == ((uint32_t)op_beq >> OpcodeShift));
    // Replace "beq $zero, $zero, offset" with "andi $zero, $zero, offset"
    inst->setOpcode(op_andi);

    AutoFlushICache::flush(uintptr_t(inst), 4);
}

void
Assembler::ToggleCall(CodeLocationLabel inst_, bool enabled)
{
    Instruction* inst = (Instruction*)inst_.raw();
    InstImm* i0 = (InstImm*) inst;
    InstImm* i1 = (InstImm*) i0->next();
    InstImm* i3 = (InstImm*) i1->next()->next();
    InstImm* i5 = (InstImm*) i3->next()->next();
    Instruction* i6 = (Instruction*) i5->next();

    MOZ_ASSERT(i0->extractOpcode() == ((uint32_t)op_lui >> OpcodeShift));
    MOZ_ASSERT(i1->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(i3->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));
    MOZ_ASSERT(i5->extractOpcode() == ((uint32_t)op_ori >> OpcodeShift));

    if (enabled) {
        MOZ_ASSERT(i6->extractOpcode() != ((uint32_t)op_lui >> OpcodeShift));
        InstReg jalr = InstReg(op_special, ScratchRegister, zero, ra, ff_jalr);
        *i6 = jalr;
    } else {
        InstNOP nop;
        *i6 = nop;
    }

    AutoFlushICache::flush(uintptr_t(i6), 4);
}

void Assembler::UpdateBoundsCheck(uint64_t heapSize, Instruction* inst)
{
    // Replace with new value
    Assembler::UpdateLoad64Value(inst, heapSize);
}
