#include "jitcoderegion.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>

#include "asmjit/core/codeholder.h"
#include "asmjit/core/virtmem.h"

#if defined(__APPLE__)
#include <TargetConditionals.h>
#include <sys/mman.h>
#endif

namespace dsp56k
{
	namespace
	{
		constexpr size_t g_alignment = 64;

		struct Region
		{
			std::mutex mutex;
			uint8_t* rx = nullptr;
			uint8_t* rw = nullptr;
			size_t size = 0;
			std::map<size_t, size_t> freeBlocks;		// offset -> size
			std::map<size_t, size_t> usedBlocks;		// offset -> size
			size_t used = 0;
			size_t peak = 0;
			uint64_t failed = 0;
		};

		Region& region()
		{
			static Region r;
			return r;
		}
	}

	bool JitCodeRegion::install(void* _rx, void* _rw, const size_t _size)
	{
		auto& r = region();
		std::lock_guard lock(r.mutex);
		if(r.rx || !_rx || !_rw || _size < g_alignment)
			return false;
		r.rx = static_cast<uint8_t*>(_rx);
		r.rw = static_cast<uint8_t*>(_rw);
		r.size = _size & ~(g_alignment - 1);
		r.freeBlocks.clear();
		r.freeBlocks.emplace(0, r.size);
		return true;
	}

	bool JitCodeRegion::installDualMapped(const size_t _size)
	{
#if defined(__APPLE__) && TARGET_OS_OSX
		// Apple Silicon macOS executes JIT code only from MAP_JIT memory, which cannot be given a second, writable
		// mapping. The desktop test region is therefore one MAP_JIT mapping used as both views; _add() writes it under
		// asmjit's per-thread JIT write protection. The two-view case is what iOS with StikDebug uses (tools/jitprobe).
		void* p = mmap(nullptr, _size, PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
		if(p == MAP_FAILED)
			return false;
		return install(p, p, _size);
#else
		asmjit::VirtMem::DualMapping dm{};
		if(asmjit::VirtMem::allocDualMapping(&dm, _size, asmjit::VirtMem::MemoryFlags::kAccessRWX) != asmjit::kErrorOk)
			return false;
		return install(dm.rx, dm.rw, _size);
#endif
	}

	bool JitCodeRegion::installed()
	{
		auto& r = region();
		std::lock_guard lock(r.mutex);
		return r.rx != nullptr;
	}

	bool JitCodeRegion::contains(const void* _rx)
	{
		auto& r = region();
		std::lock_guard lock(r.mutex);
		const auto* p = static_cast<const uint8_t*>(_rx);
		return r.rx && p >= r.rx && p < r.rx + r.size;
	}

	// first fit over the free list, blocks rounded up to the alignment
	bool JitCodeRegion::alloc(const size_t _size, uint8_t*& _rx, uint8_t*& _rw)
	{
		auto& r = region();
		std::lock_guard lock(r.mutex);
		const size_t size = (std::max<size_t>(_size, 1) + g_alignment - 1) & ~(g_alignment - 1);
		for(auto it = r.freeBlocks.begin(); it != r.freeBlocks.end(); ++it)
		{
			if(it->second < size)
				continue;
			const size_t offset = it->first;
			const size_t remaining = it->second - size;
			r.freeBlocks.erase(it);
			if(remaining)
				r.freeBlocks.emplace(offset + size, remaining);
			r.usedBlocks.emplace(offset, size);
			r.used += size;
			r.peak = std::max(r.peak, r.used);
			_rx = r.rx + offset;
			_rw = r.rw + offset;
			return true;
		}
		++r.failed;
		return false;
	}

	// return a block to the free list and merge it with free neighbours
	bool JitCodeRegion::release(const void* _rx)
	{
		auto& r = region();
		std::lock_guard lock(r.mutex);
		const auto* p = static_cast<const uint8_t*>(_rx);
		if(!r.rx || p < r.rx || p >= r.rx + r.size)
			return false;
		const size_t offset = static_cast<size_t>(p - r.rx);
		const auto used = r.usedBlocks.find(offset);
		if(used == r.usedBlocks.end())
			return false;
		size_t start = offset;
		size_t size = used->second;
		r.usedBlocks.erase(used);
		r.used -= size;

		auto next = r.freeBlocks.lower_bound(start);
		if(next != r.freeBlocks.end() && start + size == next->first)
		{
			size += next->second;
			next = r.freeBlocks.erase(next);
		}
		if(next != r.freeBlocks.begin())
		{
			auto prev = std::prev(next);
			if(prev->first + prev->second == start)
			{
				start = prev->first;
				size += prev->second;
				r.freeBlocks.erase(prev);
			}
		}
		r.freeBlocks.emplace(start, size);
		return true;
	}

	size_t JitCodeRegion::size() { auto& r = region(); std::lock_guard lock(r.mutex); return r.size; }
	size_t JitCodeRegion::used() { auto& r = region(); std::lock_guard lock(r.mutex); return r.used; }
	size_t JitCodeRegion::peakUsed() { auto& r = region(); std::lock_guard lock(r.mutex); return r.peak; }
	uint64_t JitCodeRegion::failedAllocations() { auto& r = region(); std::lock_guard lock(r.mutex); return r.failed; }

	// Mirrors asmjit::JitRuntime::_add, with the memory coming from the region: the code is relocated to the
	// executable view, copied through the writable alias, and the instruction cache is flushed for the copy.
	asmjit::Error RegionJitRuntime::_add(void** _dst, asmjit::CodeHolder* _code) noexcept
	{
		if(!JitCodeRegion::installed())
			return JitRuntime::_add(_dst, _code);

		*_dst = nullptr;

		ASMJIT_PROPAGATE(_code->flatten());
		ASMJIT_PROPAGATE(_code->resolveUnresolvedLinks());

		const size_t estimatedCodeSize = _code->codeSize();
		if(ASMJIT_UNLIKELY(estimatedCodeSize == 0))
			return asmjit::DebugUtils::errored(asmjit::kErrorNoCodeGenerated);

		uint8_t* rx = nullptr;
		uint8_t* rw = nullptr;
		if(!JitCodeRegion::alloc(estimatedCodeSize, rx, rw))
			return asmjit::DebugUtils::errored(asmjit::kErrorOutOfMemory);

		const asmjit::Error err = _code->relocateToBase(reinterpret_cast<uintptr_t>(rx));
		if(ASMJIT_UNLIKELY(err))
		{
			JitCodeRegion::release(rx);
			return err;
		}

		const size_t codeSize = _code->codeSize();

		const auto copySections = [&]()
		{
			for(const asmjit::Section* section : _code->_sections)
			{
				const size_t offset = static_cast<size_t>(section->offset());
				const size_t bufferSize = static_cast<size_t>(section->bufferSize());
				const size_t virtualSize = static_cast<size_t>(section->virtualSize());

				std::memcpy(rw + offset, section->data(), bufferSize);
				if(virtualSize > bufferSize)
					std::memset(rw + offset + bufferSize, 0, virtualSize - bufferSize);
			}
		};

		if(rw == rx)
		{
			// one mapping for both views (desktop test region): MAP_JIT write protection, flushed by the scope
			asmjit::VirtMem::ProtectJitReadWriteScope scope(rx, codeSize);
			copySections();
		}
		else
		{
			copySections();
			asmjit::VirtMem::flushInstructionCache(rx, codeSize);
		}

		*_dst = rx;
		return asmjit::kErrorOk;
	}

	asmjit::Error RegionJitRuntime::_release(void* _p) noexcept
	{
		if(JitCodeRegion::contains(_p))
			return JitCodeRegion::release(_p) ? asmjit::kErrorOk : asmjit::DebugUtils::errored(asmjit::kErrorInvalidArgument);
		return JitRuntime::_release(_p);
	}
}
