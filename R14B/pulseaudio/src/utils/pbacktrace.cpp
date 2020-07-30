#ifndef _cplusplus
#define _cplusplus
#include <android/log.h>
#include<utils/CallStack.h>
#include <dlfcn.h>
#include <Hlog.h>
#include <unwind.h>
#include "pbacktrace.h"
#endif

namespace {

struct BacktraceState
{
    void** current;
    void** end;
};

static _Unwind_Reason_Code unwindCallback(struct _Unwind_Context* context, void* arg)
{
    BacktraceState* state = static_cast<BacktraceState*>(arg);
    uintptr_t pc = _Unwind_GetIP(context);
    if (pc) {
        if (state->current == state->end) {
            return _URC_END_OF_STACK;
        } else {
            *state->current++ = reinterpret_cast<void*>(pc);
        }
    }
    return _URC_NO_REASON;
}

}

size_t captureBacktrace(void** buffer, size_t max)
{
    BacktraceState state = {buffer, buffer + max};
    _Unwind_Backtrace(unwindCallback, &state);

    return state.current - buffer;
}

void dumpBacktrace(void** buffer, size_t count)
{
    for (size_t idx = 0; idx < count; ++idx) {
        const void* addr = buffer[idx];
        const char* symbol = "";

        Dl_info info;
        if (dladdr(addr, &info) && info.dli_sname) {
            symbol = info.dli_sname;
        }
        HLOGE(TAG_HARMAN_PULSEAUDIO,"#%02d pc 0x%016llx %s (%s)\n",idx,(long)addr-(long)info.dli_fbase,info.dli_fname,strlen(symbol)? symbol:"...");
    }
}

int pbacktrace(void)
{
    const size_t max = 200;
    void* buffer[max];
    dumpBacktrace(buffer, captureBacktrace(buffer, max));

    return 0;

}
