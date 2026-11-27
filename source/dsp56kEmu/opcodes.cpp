#include "opcodes.h"
#include "opcodeanalysis.h"

namespace dsp56k
{
	struct RuntimeFieldInfo
	{
		FieldInfo fieldInfos[Field_COUNT];

		RuntimeFieldInfo() : fieldInfos() {}
		RuntimeFieldInfo(const Instruction _i) : fieldInfos()
		{
			for (auto f = 0; f < Field::Field_COUNT; ++f)
				fieldInfos[f] = initField(g_opcodes[_i].m_opcode, g_fieldParseConfigs[f].ch, g_fieldParseConfigs[f].count);
		}
	};

	struct RuntimeFieldInfos
	{
		RuntimeFieldInfo fieldInfos[g_opcodeCount];

		RuntimeFieldInfos() : fieldInfos()
		{
			for (size_t i = 0; i < g_opcodeCount; ++i)
				fieldInfos[i] = RuntimeFieldInfo(static_cast<Instruction>(i));
		}
	};

	const RuntimeFieldInfos g_runtimeFieldInfos;

	static_assert(getFieldInfoCE<Bsset_S, Field_DDDDDD>().bit == 8, "invalid");
	static_assert(getFieldInfoCE<Bsset_S, Field_DDDDDD>().len == 6, "invalid");
	static_assert(getFieldInfoCE<Bsset_S, Field_bbbbb>().bit == 0, "invalid");
	static_assert(getFieldInfoCE<Bsset_S, Field_bbbbb>().len == 5, "invalid");

	static_assert(getFieldInfoCE<Add_xxxx, Field_oo>().bit == 4, "invalid");
	static_assert(getFieldInfoCE<Add_xxxx, Field_oo>().len == 2, "invalid");
	static_assert(getFieldInfoCE<Add_xxxx, Field_ooooo>().len == 5, "invalid");
	static_assert(getFieldInfoCE<Add_xxxx, Field_ooooo>().bit == 9, "invalid");

	const FieldInfo& getFieldInfo(const Instruction _i, const Field _f)
	{
		return g_runtimeFieldInfos.fieldInfos[_i].fieldInfos[_f];
	}

	namespace
	{
		/*	The decoders below searched the opcode lists linearly for every word, about 10% of the JIT's compile time.
			Their results depend on the word only, so one direct-mapped memo per decoder serves every DSP of the process.
			An entry is one word, valid bit | result << 24 | opcode, so a reader on another thread sees a whole entry or
			a miss, never a torn one. Static storage: zero, so empty, before any use.
		*/
		constexpr size_t g_memoSize = 8192;
		constexpr uint64_t g_memoValid = 1ull << 63;
		constexpr uint16_t g_noOpcode = 0xffff;

		using Memo = std::array<std::atomic<uint64_t>, g_memoSize>;
		Memo g_typeMemo;
		Memo g_nonParallelMemo;
		Memo g_parallelMemo;

		std::atomic<uint64_t>& memoSlot(Memo& _memo, const TWord _op)
		{
			return _memo[((_op & 0xffffff) * 0x9E3779B1u) >> (32 - 13) & (g_memoSize - 1)];
		}

		// the 40 bits of a result for _op, or false
		bool memoGet(Memo& _memo, const TWord _op, uint64_t& _result)
		{
			const uint64_t e = memoSlot(_memo, _op).load(std::memory_order_relaxed);
			if(!(e & g_memoValid) || (e & 0xffffff) != _op)
				return false;
			_result = (e >> 24) & 0xffffffffffull;
			return true;
		}

		void memoPut(Memo& _memo, const TWord _op, const uint64_t _result)
		{
			if(_op > 0xffffff)
				return;
			memoSlot(_memo, _op).store(g_memoValid | ((_result & 0xffffffffffull) << 24) | _op, std::memory_order_relaxed);
		}

		uint16_t indexOf(const OpcodeInfo* _oi)
		{
			return _oi ? static_cast<uint16_t>(_oi - &Opcodes::getOpcodeInfoAt(0)) : g_noOpcode;
		}

		const OpcodeInfo* fromIndex(const uint16_t _index)
		{
			return _index == g_noOpcode ? nullptr : &Opcodes::getOpcodeInfoAt(_index);
		}
	}

	Opcodes::Opcodes()
	{

		constexpr auto len = g_opcodeCount;

		m_opcodesAlu.reserve(len);
		m_opcodesMove.reserve(len);
		m_opcodesNonParallel.reserve(len);

		for(size_t i=0; i<len; ++i)
		{
			const auto& opcode = g_opcodes[i];

			if(opcode.getInstruction() == ResolveCache)
				continue;

			assert(opcode.getInstruction() == i && "programming error, list sorting is faulty");

			if(dsp56k::isNonParallelOpcode(opcode))
				m_opcodesNonParallel.push_back(&opcode);
			else if(hasField(opcode, Field_AluOperation))
				m_opcodesMove.push_back(&opcode);
			else if(hasField(opcode, Field_MoveOperation))
				m_opcodesAlu.push_back(&opcode);
		}
	}

	const OpcodeInfo* Opcodes::findNonParallelOpcodeInfo(const TWord _opcode) const
	{
		uint64_t r;
		if(memoGet(g_nonParallelMemo, _opcode, r))
			return fromIndex(static_cast<uint16_t>(r));
		const auto* oi = findNonParallelOpcodeInfoUncached(_opcode);
		memoPut(g_nonParallelMemo, _opcode, indexOf(oi));
		return oi;
	}

	const OpcodeInfo* Opcodes::findNonParallelOpcodeInfoUncached(TWord _opcode) const
	{
		assert(isNonParallelOpcode(_opcode));

		if(const auto* oi = findOpcodeInfo(_opcode, m_opcodesNonParallel))
			return oi;

		/*	The chip runs any undefined operation code as ILLEGAL (56300 FM 2.3.2.2): a NOP, then the illegal instruction
			interrupt. Firmware has been seen to run $000040, which the JIT compiled as an empty block that it entered over
			and over, until the stack overflowed. The disassembler prints such a word as data, see
			Disassembler::disassemble
		*/
		return &getOpcodeInfoAt(Illegal);
	}

	const OpcodeInfo* Opcodes::findParallelMoveOpcodeInfo(TWord _opcode) const
	{
		assert(isParallelOpcode(_opcode));
		return findOpcodeInfo(_opcode, m_opcodesMove);
	}

	const OpcodeInfo* Opcodes::findParallelAluOpcodeInfo(TWord _opcode) const
	{
		assert(isParallelOpcode(_opcode));
		return findOpcodeInfo(_opcode, m_opcodesAlu);
	}

	const OpcodeInfo* Opcodes::findParallelOpcodeInfo(const TWord _opcode, const OpcodeInfo*& _alu) const
	{
		uint64_t r;
		if(memoGet(g_parallelMemo, _opcode, r))
		{
			_alu = fromIndex(static_cast<uint16_t>(r >> 16));
			return fromIndex(static_cast<uint16_t>(r));
		}
		const auto* move = findParallelOpcodeInfoUncached(_opcode, _alu);
		memoPut(g_parallelMemo, _opcode, (uint64_t(indexOf(_alu)) << 16) | indexOf(move));
		return move;
	}

	const OpcodeInfo* Opcodes::findParallelOpcodeInfoUncached(const TWord _opcode, const OpcodeInfo*& _alu) const
	{
		/*	As the reference simulator runs them: a word whose move field matches no move is ILLEGAL and its ALU operation
			does not run, $083100 and $083110 take the illegal instruction interrupt and $083110 adds nothing. An ALU byte
			that matches no operation does nothing, the move runs alone, $080004 runs as $080000. Both used to decode to
			nullptr, which the JIT dereferenced, or to a word of length 0 that the JIT decoded over and over.
			The disassembler prints both as data, as the simulator does, see Disassembler::disassemble
		*/
		const auto* oiMove = findParallelMoveOpcodeInfo(_opcode);

		if(!oiMove)
		{
			_alu = nullptr;
			return &getOpcodeInfoAt(Illegal);
		}

		_alu = (_opcode & 0xff) ? findParallelAluOpcodeInfo(_opcode) : nullptr;
		return oiMove;
	}

	const OpcodeInfo& Opcodes::getOpcodeInfoAt(size_t _index)
	{
		return g_opcodes[_index];
	}

	uint32_t Opcodes::getOpcodeLength(const TWord _op) const
	{
		if(!_op)
			return 1;

		Instruction instA, instB;
		getInstructionTypes(_op, instA, instB);

		return getOpcodeLength(_op, instA, instB);
	}

	uint32_t Opcodes::getOpcodeLength(const TWord _op, Instruction _instA, Instruction _instB)
	{
		const auto lenA = _instA != Invalid ? dsp56k::getOpcodeLength(_instA, _op) : 0;
		const auto lenB = _instB != Invalid ? dsp56k::getOpcodeLength(_instB, _op) : 0;
		return std::max(lenA, lenB);
	}

	bool Opcodes::writesToPMemory(const TWord _op) const
	{
		if(!_op)
			return false;

		if(isNonParallelOpcode(_op))
		{
			const auto oi = findNonParallelOpcodeInfo(_op);
			return oi ? dsp56k::writesToPMemory(oi->getInstruction(), _op) : false;
		}

		const auto* oiMove = findParallelMoveOpcodeInfo(_op);

		return oiMove && dsp56k::writesToPMemory(oiMove->getInstruction(), _op);
	}

	uint32_t Opcodes::getInstructionTypes(const TWord _op, Instruction& _a, Instruction& _b) const
	{
		if(!_op)
		{
			_a = Nop;
			_b = Invalid;
			return 1;
		}

		uint64_t r;
		if(memoGet(g_typeMemo, _op, r))
		{
			_a = static_cast<Instruction>(static_cast<int16_t>(r));
			_b = static_cast<Instruction>(static_cast<int16_t>(r >> 16));
			return static_cast<uint32_t>((r >> 32) & 0xff);
		}
		const auto res = getInstructionTypesUncached(_op, _a, _b);
		if(res <= 0xff)
		{
			memoPut(g_typeMemo, _op, (uint64_t(res) << 32) | (uint64_t(static_cast<uint16_t>(static_cast<int16_t>(_b))) << 16) |
				uint64_t(static_cast<uint16_t>(static_cast<int16_t>(_a))));
		}
		return res;
	}

	uint32_t Opcodes::getInstructionTypesUncached(const TWord _op, Instruction& _a, Instruction& _b) const
	{

		_a = _b = Invalid;

		if(isNonParallelOpcode(_op))
		{
			const auto oi = findNonParallelOpcodeInfo(_op);

			if(oi)
				_a = oi->getInstruction();
			return 1;
		}

		const OpcodeInfo* oiAlu;
		const auto* oiMove = findParallelOpcodeInfo(_op, oiAlu);

		uint32_t res = 0;

		if(oiAlu)
		{
			_a = oiAlu->getInstruction();
			++res;
		}

		if(oiMove)
		{
			++res;

			if(oiAlu)
				_b = oiMove->getInstruction();
			else
				_a = oiMove->getInstruction();
		}
		return res;
	}

	bool Opcodes::getRegisters(RegisterMask& _written, RegisterMask& _read, const TWord _opA) const
	{
		Instruction instA, instB;
		getInstructionTypes(_opA, instA, instB);
		return getRegisters(_written, _read, _opA, instA, instB);
	}

	bool Opcodes::getRegisters(RegisterMask& _written, RegisterMask& _read, const TWord _opA, const Instruction _instA, const Instruction _instB)
	{
		_written = _read = RegisterMask::None;

		if(_instA != Invalid && _instA != Nop)
		{
			dsp56k::getRegisters(_written, _read, _instA, _opA);
		}

		if(_instB != Invalid && _instB != Nop)
		{
			auto written = RegisterMask::None;
			auto read = RegisterMask::None;
			dsp56k::getRegisters(written, read, _instB, _opA);
			_written |= written;
			_read |= read;
		}

		return _written != RegisterMask::None || _read != RegisterMask::None;
	}

	uint32_t Opcodes::getFlags(Instruction _instA, Instruction _instB)
	{
		uint32_t flags = 0;
		if(_instA != Invalid)
			flags = g_opcodes[_instA].m_flags;
		if(_instB != Invalid)
			flags |= g_opcodes[_instB].m_flags;
		return flags;
	}

	bool Opcodes::getMemoryAddress(TWord& _addr, EMemArea& _area, const TWord opA, const TWord opB) const
	{
		Instruction instA, instB;
		getInstructionTypes(opA, instA, instB);
		if(dsp56k::getMemoryAddress(_addr, _area, instA, opA, opB))
			return true;
		if(instB != Invalid)
			return dsp56k::getMemoryAddress(_addr, _area, instB, opA, opB);
		return false;
	}

	const OpcodeInfo* Opcodes::findOpcodeInfo(TWord _opcode, const std::vector<const OpcodeInfo*>& _opcodes)
	{
		const OpcodeInfo* res = nullptr;

		for (const auto* oi : _opcodes)
		{
			if(match(*oi, _opcode))
			{
#ifdef _DEBUG
				if(res != nullptr)
				{
					// it is unexpected that we have a second match. debugging helpers below, two opcodes for the same opcode should not happen
					match(*oi, _opcode);
					match(*res, _opcode);
					assert(res == nullptr && "opcode collision, more than one instruction possible");
				}
				res = oi;
#else
				return oi;
#endif
			}
		}
		return res;
	}
}
