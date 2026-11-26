#pragma once

#include "dsp56kBase/dspassert.h"
#include "dsp56kBase/logging.h"

#include <atomic>
#include <cstdint>

namespace dsp56k
{
}

// Rate limited per call site: the first 8 reports, then only the 16th, 32nd, 64th... with the running count.
// A firmware fault that repeats every instruction otherwise floods the log (3.48M lines in 90 s on iOS) and the
// formatting dominates the run. Emulation is unaffected; the assert still fires on every occurrence.
#define LOG_ERR_BASE(A,ERR)					\
{											\
	static std::atomic<uint32_t> s_logErrCount{0};	\
	const uint32_t logErrIndex = s_logErrCount.fetch_add(1, std::memory_order_relaxed);	\
	std::stringstream ss;					\
	if(logErrIndex < 8 || (logErrIndex & (logErrIndex - 1)) == 0 || !(A))	\
	{										\
		ss << __func__ << "@" << __LINE__ << ": " << "DSP 56300 ERROR: " << ERR;	\
		if(logErrIndex >= 8)				\
			ss << " [occurrence " << (logErrIndex + 1) << ", further ones are counted]";	\
		LOGTOCONSOLE( ss );					\
	}										\
	assertf( A, ss.str().c_str() );			\
}

// emulation errors
#define LOG_ERR_NOTIMPLEMENTED(T)					LOG_ERR_BASE( false, "Not implemented: " << T )

// memory errors
#define LOG_ERR_MEM_WRITE(ADDR)						LOG_ERR_BASE( true, "Memory Write: " << HEX(ADDR) )
#define LOG_ERR_MEM_READ(ADDR)						LOG_ERR_BASE( true, "Memory Read: " << HEX(ADDR) )
#define LOG_ERR_MEM_READ_UNINITIALIZED(AREA,ADDR)	LOG_ERR_BASE( true, "Read uninitialized memory: area " << AREA << " @" << HEX(ADDR) )
#define LOG_ERR_DSP_ILLEGALINSTRUCTION(OP)			LOG_ERR_BASE( false, "Illegal instruction: " << HEX(OP) )
