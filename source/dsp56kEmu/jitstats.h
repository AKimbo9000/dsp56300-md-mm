#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

namespace dsp56k
{
	// Totals of the JIT's compile work, updated only when it compiles, never per executed instruction, so a host can
	// tell how much of an audio block went into compiling code that runs for the first time. Written by the thread
	// that runs the DSP; relaxed atomics so another thread may read them.
	struct JitStats
	{
		std::atomic<uint64_t> emitNs{0};		// JitBlockChain::emit as a whole
		std::atomic<uint64_t> emitBlocks{0};
		std::atomic<uint64_t> codegenNs{0};		// JitBlock::emit: DSP instructions to host instructions
		std::atomic<uint64_t> finalizeNs{0};	// the asmjit builder's finalize (serialisation to machine code)
		std::atomic<uint64_t> addNs{0};			// the runtime's add: relocation, copy into the code region, icache flush
		std::atomic<uint64_t> chainNs{0};		// new block chains, the first entry into a DSP mode
		std::atomic<uint64_t> chains{0};
		std::atomic<uint64_t> codeBytes{0};		// host code emitted
		std::atomic<uint64_t> createNs{0};		// JitBlockChain::create up to running the block, emit included
		std::atomic<uint64_t> creates{0};

		static uint64_t now()
		{
			return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::steady_clock::now().time_since_epoch()).count());
		}
		static void add(std::atomic<uint64_t>& _total, const uint64_t _since)
		{
			_total.fetch_add(now() - _since, std::memory_order_relaxed);
		}
		// adds the time since _since and returns now, so consecutive phases share one clock read
		static uint64_t lap(std::atomic<uint64_t>& _total, const uint64_t _since)
		{
			const auto t = now();
			_total.fetch_add(t - _since, std::memory_order_relaxed);
			return t;
		}
	};

	extern JitStats g_jitStats;
}
