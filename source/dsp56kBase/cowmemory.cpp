#include "cowmemory.h"

#ifdef __APPLE__
#include <TargetConditionals.h>
#define DSP56K_COW_MACH 1
#include <mach/mach.h>
#if TARGET_OS_OSX
#include <mach/mach_vm.h>
#endif

namespace
{
	// The iOS SDK does not expose mach_vm_allocate/mach_vm_remap/mach_vm_deallocate. Their vm_* forms are the same
	// calls with vm_address_t, which is 64 bits on arm64. Without a copy-on-write clone, every new JIT block chain filled
	// its dispatch table (one entry per used P word) in fresh memory: about 15 ms on an iPhone 17 Pro Max when the DSP
	// first entered a new addressing mode, such as at the Monomachine's first Play (IOS_PORT_STATUS).
#if TARGET_OS_OSX
	using CowAddress = mach_vm_address_t;
	kern_return_t cowAllocate(CowAddress* _address, const size_t _size)
	{
		return mach_vm_allocate(mach_task_self(), _address, _size, VM_FLAGS_ANYWHERE);
	}
	kern_return_t cowRemapCopy(CowAddress* _address, const size_t _size, const CowAddress _source, vm_prot_t* _cur, vm_prot_t* _max)
	{
		return mach_vm_remap(mach_task_self(), _address, _size, 0, VM_FLAGS_ANYWHERE, mach_task_self(), _source, TRUE,
			_cur, _max, VM_INHERIT_NONE);
	}
	void cowDeallocate(const CowAddress _address, const size_t _size)
	{
		mach_vm_deallocate(mach_task_self(), _address, _size);
	}
#else
	using CowAddress = vm_address_t;
	kern_return_t cowAllocate(CowAddress* _address, const size_t _size)
	{
		return vm_allocate(mach_task_self(), _address, _size, VM_FLAGS_ANYWHERE);
	}
	kern_return_t cowRemapCopy(CowAddress* _address, const size_t _size, const CowAddress _source, vm_prot_t* _cur, vm_prot_t* _max)
	{
		return vm_remap(mach_task_self(), _address, _size, 0, VM_FLAGS_ANYWHERE, mach_task_self(), _source, TRUE,
			_cur, _max, VM_INHERIT_NONE);
	}
	void cowDeallocate(const CowAddress _address, const size_t _size)
	{
		vm_deallocate(mach_task_self(), _address, _size);
	}
#endif
}
#endif

namespace dsp56k
{
	CowMemory::~CowMemory() { clear(); }

	bool CowMemory::allocate(const size_t _bytes)
	{
		clear();
#ifdef DSP56K_COW_MACH
		if (!_bytes) return false;
		CowAddress address = 0;
		if (cowAllocate(&address, _bytes) != KERN_SUCCESS)
			return false;
		m_data = reinterpret_cast<void*>(address);
		m_size = _bytes;
		return true;
#else
		(void)_bytes;
		return false;
#endif
	}

	bool CowMemory::clone(const CowMemory& _source)
	{
		if (this == &_source) return false;
		clear();
#ifdef DSP56K_COW_MACH
		if (!_source.data()) return false;
		CowAddress address = 0;
		vm_prot_t current = 0, maximum = 0;
		const auto result = cowRemapCopy(&address, _source.size(), reinterpret_cast<CowAddress>(_source.data()), &current, &maximum);
		if (result != KERN_SUCCESS) return false;
		m_data = reinterpret_cast<void*>(address);
		m_size = _source.size();
		if ((current & (VM_PROT_READ | VM_PROT_WRITE)) != (VM_PROT_READ | VM_PROT_WRITE))
		{
			clear();
			return false;
		}
		return true;
#else
		(void)_source;
		return false;
#endif
	}

	void CowMemory::clear()
	{
#ifdef DSP56K_COW_MACH
		if (m_data)
			cowDeallocate(reinterpret_cast<CowAddress>(m_data), m_size);
#endif
		m_data = nullptr;
		m_size = 0;
	}
}
