#pragma once

#include <cstddef>
#include <cstdint>

#include "asmjit/core/jitruntime.h"

namespace dsp56k
{
	// One process-wide region that all JIT code lives in, when one is installed. On iOS devices with TXM a debugged
	// app cannot execute memory it maps itself: the debugger (StikDebug) prepares one executable region and the app
	// writes into it through its own writable alias (see doc/JIT_SETUP.md). With no region installed, the JIT uses
	// asmjit's own allocator as before.
	class JitCodeRegion
	{
	public:
		// Install before the first DSP is created. _rx is the executable view, _rw a writable alias of the same
		// memory, both _size bytes. Returns false if a region is already installed or the arguments are invalid.
		static bool install(void* _rx, void* _rw, size_t _size);

		// For testing on a desktop: allocate a dual-mapped region with asmjit and install it.
		static bool installDualMapped(size_t _size);

		static bool installed();
		static bool contains(const void* _rx);
		static bool alloc(size_t _size, uint8_t*& _rx, uint8_t*& _rw);
		static bool release(const void* _rx);

		static size_t size();
		static size_t used();
		static size_t peakUsed();
		static uint64_t failedAllocations();
	};

	// asmjit::JitRuntime that places code in the JitCodeRegion when one is installed, and otherwise behaves exactly
	// like asmjit::JitRuntime.
	class RegionJitRuntime : public asmjit::JitRuntime
	{
	public:
		asmjit::Error _add(void** _dst, asmjit::CodeHolder* _code) noexcept override;
		asmjit::Error _release(void* _p) noexcept override;
	};
}
