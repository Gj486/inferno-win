// Dante Bridge ASIO driver - STAGE A (test tones)
// Drop-in replacement for the Stage B driver: same CLSID, same driver name,
// same 4 in / 4 out @ 48 kHz / Int32 layout.
// Instead of talking to DanteBridge.exe through shared memory, it generates
// 4 fixed test tones on the outputs (1 kHz / 2 kHz / 3 kHz / 4 kHz) and
// returns silence on the inputs. Use it to verify that an ASIO host (Reaper,
// Cubase, ASIO4ALL test panel, ...) can load and run the driver at all.
//
// The ASIO interface is declared here from its public binary layout
// (no Steinberg SDK files are used). ASIO is a trademark of Steinberg
// Media Technologies GmbH. 64-bit build only.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <objbase.h>
#include <mmsystem.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>

// ------------------------------------------------------------ ASIO types

typedef long ASIOBool;
typedef long ASIOError;
typedef double ASIOSampleRate;
typedef long ASIOSampleType;

enum { ASIOFalse = 0, ASIOTrue = 1 };
enum {
    ASE_OK = 0,
    ASE_SUCCESS = 0x3f4847a0,
    ASE_NotPresent = -1000,
    ASE_HWMalfunction,
    ASE_InvalidParameter,
    ASE_InvalidMode,
    ASE_SPNotAdvancing,
    ASE_NoClock,
    ASE_NoMemory
};
enum { ASIOSTInt32LSB = 18 };

struct ASIOSamples { unsigned long hi; unsigned long lo; };
struct ASIOTimeStamp { unsigned long hi; unsigned long lo; };
struct ASIOTime;  // opaque: we never use the time-info callback

struct ASIOClockSource {
    long index;
    long associatedChannel;
    long associatedGroup;
    ASIOBool isCurrentSource;
    char name[32];
};

struct ASIOChannelInfo {
    long channel;
    ASIOBool isInput;
    ASIOBool isActive;
    long channelGroup;
    ASIOSampleType type;
    char name[32];
};

struct ASIOBufferInfo {
    ASIOBool isInput;
    long channelNum;
    void* buffers[2];
};

struct ASIOCallbacks {
    void (*bufferSwitch)(long doubleBufferIndex, ASIOBool directProcess);
    void (*sampleRateDidChange)(ASIOSampleRate sRate);
    long (*asioMessage)(long selector, long value, void* message, double* opt);
    ASIOTime* (*bufferSwitchTimeInfo)(ASIOTime* params, long doubleBufferIndex, ASIOBool directProcess);
};

// ------------------------------------------------------------ constants

// Same CLSID as Stage B, so a Stage A install replaces a Stage B install 1:1
// (and vice versa) without the host having to re-select the driver.
// {7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}
static const GUID CLSID_DanteBridgeAsio =
    {0x7b1d3e52, 0x9c4a, 0x4f0e, {0x8a, 0x2b, 0x5d, 0x6e, 0x7f, 0x80, 0x9a, 0x1b}};
static const char* kDriverName = "Dante Bridge ASIO";
static const char* kClsidString = "{7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}";

static const int kInputs = 4;
static const int kOutputs = 4;
static const double kRate = 48000.0;
static const long kMinBuf = 64, kMaxBuf = 2048, kPrefBuf = 512;

// Test tone frequencies, one per output channel (Hz).
static const double kToneHz[kInputs] = {1000.0, 2000.0, 3000.0, 4000.0};
// Peak amplitude (-20 dBFS): audible but far from clipping.
static const double kToneAmp = 0.1;

static HMODULE g_module = nullptr;
static std::atomic<long> g_objects{0};

// ------------------------------------------------------------ driver

class DanteAsio : public IUnknown {
public:
    DanteAsio() : refs_(1) { g_objects++; }
    // NOT virtual: a virtual destructor would add a vtable slot and break the ASIO layout.
    ~DanteAsio() { disposeBuffers(); g_objects--; }

    // IUnknown. ASIO hosts call CoCreateInstance with riid == our CLSID.
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (IsEqualIID(riid, IID_IUnknown) || IsEqualIID(riid, CLSID_DanteBridgeAsio)) {
            *ppv = static_cast<DanteAsio*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return ++refs_; }
    ULONG STDMETHODCALLTYPE Release() override {
        ULONG n = --refs_;
        if (n == 0) delete this;
        return n;
    }

    // IASIO methods, in the exact order of the ASIO interface.
    virtual ASIOBool init(void* sysHandle) {
        (void)sysHandle;
        return ASIOTrue;
    }
    virtual void getDriverName(char* name) { std::strcpy(name, kDriverName); }
    virtual long getDriverVersion() { return 1; }
    virtual void getErrorMessage(char* string) { std::strcpy(string, "No error"); }

    virtual ASIOError start() {
        if (!callbacks_) return ASE_NotPresent;
        if (running_) return ASE_OK;
        stop_flag_ = false;
        sample_pos_ = 0;
        sample_index_ = 0;
        timeBeginPeriod(1);
        thread_ = CreateThread(nullptr, 0, &DanteAsio::threadEntry, this, 0, nullptr);
        if (!thread_) {
            timeEndPeriod(1);
            return ASE_HWMalfunction;
        }
        SetThreadPriority(thread_, THREAD_PRIORITY_TIME_CRITICAL);
        running_ = true;
        return ASE_OK;
    }

    virtual ASIOError stop() {
        if (!running_) return ASE_OK;
        stop_flag_ = true;
        if (thread_) {
            WaitForSingleObject(thread_, 3000);
            CloseHandle(thread_);
            thread_ = nullptr;
        }
        timeEndPeriod(1);
        running_ = false;
        return ASE_OK;
    }

    virtual ASIOError getChannels(long* numInputChannels, long* numOutputChannels) {
        *numInputChannels = kInputs;
        *numOutputChannels = kOutputs;
        return ASE_OK;
    }

    virtual ASIOError getLatencies(long* inputLatency, long* outputLatency) {
        long b = buffer_size_ ? buffer_size_ : kPrefBuf;
        *inputLatency = b;
        *outputLatency = b;
        return ASE_OK;
    }

    virtual ASIOError getBufferSize(long* minSize, long* maxSize, long* preferredSize, long* granularity) {
        *minSize = kMinBuf;
        *maxSize = kMaxBuf;
        *preferredSize = kPrefBuf;
        *granularity = -1;  // powers of two
        return ASE_OK;
    }

    virtual ASIOError canSampleRate(ASIOSampleRate rate) {
        return (rate == kRate) ? ASE_OK : ASE_NoClock;
    }
    virtual ASIOError getSampleRate(ASIOSampleRate* rate) {
        *rate = kRate;
        return ASE_OK;
    }
    virtual ASIOError setSampleRate(ASIOSampleRate rate) {
        return (rate == kRate) ? ASE_OK : ASE_NoClock;
    }

    virtual ASIOError getClockSources(ASIOClockSource* clocks, long* numSources) {
        if (!clocks || !numSources || *numSources < 1) return ASE_InvalidParameter;
        clocks[0].index = 0;
        clocks[0].associatedChannel = -1;
        clocks[0].associatedGroup = -1;
        clocks[0].isCurrentSource = ASIOTrue;
        std::strcpy(clocks[0].name, "Internal");
        *numSources = 1;
        return ASE_OK;
    }
    virtual ASIOError setClockSource(long reference) {
        return reference == 0 ? ASE_OK : ASE_InvalidParameter;
    }

    virtual ASIOError getSamplePosition(ASIOSamples* sPos, ASIOTimeStamp* tStamp) {
        if (!running_) return ASE_SPNotAdvancing;
        uint64_t s = sample_pos_.load();
        uint64_t t = stamp_ns_.load();
        sPos->hi = (unsigned long)(s >> 32);
        sPos->lo = (unsigned long)(s & 0xffffffffu);
        tStamp->hi = (unsigned long)(t >> 32);
        tStamp->lo = (unsigned long)(t & 0xffffffffu);
        return ASE_OK;
    }

    virtual ASIOError getChannelInfo(ASIOChannelInfo* info) {
        if (!info) return ASE_InvalidParameter;
        int max = info->isInput ? kInputs : kOutputs;
        if (info->channel < 0 || info->channel >= max) return ASE_InvalidParameter;
        info->type = ASIOSTInt32LSB;
        info->channelGroup = 0;
        info->isActive = ASIOFalse;
        for (const auto& b : buffers_) {
            if (b.isInput == info->isInput && b.channelNum == info->channel) info->isActive = ASIOTrue;
        }
        if (info->isInput) {
            std::snprintf(info->name, sizeof(info->name), "Test Tone %d (%.0f Hz)",
                          (int)info->channel + 1, kToneHz[info->channel]);
        } else {
            std::snprintf(info->name, sizeof(info->name), "Test Out %d (unused)", (int)info->channel + 1);
        }
        return ASE_OK;
    }

    virtual ASIOError createBuffers(ASIOBufferInfo* bufferInfos, long numChannels, long bufferSize,
                                    ASIOCallbacks* callbacks) {
        if (!bufferInfos || numChannels < 1 || !callbacks) return ASE_InvalidParameter;
        if (bufferSize < kMinBuf || bufferSize > kMaxBuf) return ASE_InvalidMode;
        disposeBuffers();
        buffer_size_ = bufferSize;
        callbacks_ = callbacks;
        buffers_.assign(bufferInfos, bufferInfos + numChannels);
        storage_.assign((size_t)numChannels, std::vector<int32_t>((size_t)bufferSize * 2, 0));
        for (long i = 0; i < numChannels; i++) {
            int32_t* base = storage_[(size_t)i].data();
            bufferInfos[i].buffers[0] = base;
            bufferInfos[i].buffers[1] = base + bufferSize;
            buffers_[(size_t)i] = bufferInfos[i];
        }
        return ASE_OK;
    }

    virtual ASIOError disposeBuffers() {
        stop();
        buffers_.clear();
        storage_.clear();
        callbacks_ = nullptr;
        return ASE_OK;
    }

    virtual ASIOError controlPanel() { return ASE_NotPresent; }
    virtual ASIOError future(long selector, void* opt) {
        (void)selector;
        (void)opt;
        return ASE_NotPresent;
    }
    virtual ASIOError outputReady() { return ASE_NotPresent; }

private:
    static DWORD WINAPI threadEntry(LPVOID p) {
        static_cast<DanteAsio*>(p)->run();
        return 0;
    }

    void run() {
        LARGE_INTEGER freq, t0;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&t0);
        const double period_ticks = (double)freq.QuadPart * (double)buffer_size_ / kRate;
        long idx = 0;
        uint64_t n = 0;

        while (!stop_flag_) {
            // wait until the start of period n
            const double target = (double)t0.QuadPart + period_ticks * (double)n;
            for (;;) {
                LARGE_INTEGER now;
                QueryPerformanceCounter(&now);
                double remaining_ms = (target - (double)now.QuadPart) * 1000.0 / (double)freq.QuadPart;
                if (remaining_ms <= 0.0) break;
                if (stop_flag_) return;
                if (remaining_ms > 2.0) Sleep(1);
                else YieldProcessor();
            }

            // Inputs = test tones (this is what Open Sound Meter / Smaart record).
            // Output buffers belong to the host, so they are left alone.
            for (size_t k = 0; k < buffers_.size(); k++) {
                if (!buffers_[k].isInput) continue;
                int ch = (int)buffers_[k].channelNum;
                int32_t* dst = (int32_t*)buffers_[k].buffers[idx];
                if (ch < 0 || ch >= kInputs) {
                    std::memset(dst, 0, (size_t)buffer_size_ * sizeof(int32_t));
                    continue;
                }
                const double f = kToneHz[ch];
                const double two_pi_f_over_fs = 2.0 * 3.14159265358979323846 * f / kRate;
                for (long i = 0; i < buffer_size_; i++) {
                    double phase = two_pi_f_over_fs * (double)(sample_index_ + (uint64_t)i);
                    dst[i] = (int32_t)(kToneAmp * std::sin(phase) * 2147483647.0);
                }
            }
            sample_index_ += (uint64_t)buffer_size_;

            sample_pos_ = n * (uint64_t)buffer_size_;
            stamp_ns_ = (uint64_t)((double)(n * (uint64_t)buffer_size_) * 1e9 / kRate);
            if (callbacks_ && callbacks_->bufferSwitch) callbacks_->bufferSwitch(idx, ASIOTrue);

            idx ^= 1;
            n++;
        }
    }

    std::atomic<ULONG> refs_;
    ASIOCallbacks* callbacks_ = nullptr;
    long buffer_size_ = 0;
    std::vector<ASIOBufferInfo> buffers_;
    std::vector<std::vector<int32_t>> storage_;
    HANDLE thread_ = nullptr;
    std::atomic<bool> stop_flag_{false};
    bool running_ = false;
    std::atomic<uint64_t> sample_pos_{0};
    std::atomic<uint64_t> stamp_ns_{0};
    // Global sample counter for continuous tone phase across buffer boundaries.
    uint64_t sample_index_ = 0;
};

// ------------------------------------------------------------ class factory

class Factory : public IClassFactory {
public:
    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (IsEqualIID(riid, IID_IUnknown) || IsEqualIID(riid, IID_IClassFactory)) {
            *ppv = static_cast<IClassFactory*>(this);
            AddRef();
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }
    ULONG STDMETHODCALLTYPE AddRef() override { return ++refs_; }
    ULONG STDMETHODCALLTYPE Release() override {
        ULONG n = --refs_;
        if (n == 0) delete this;
        return n;
    }
    HRESULT STDMETHODCALLTYPE CreateInstance(IUnknown* outer, REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        *ppv = nullptr;
        if (outer) return CLASS_E_NOAGGREGATION;
        DanteAsio* drv = new DanteAsio();
        HRESULT hr = drv->QueryInterface(riid, ppv);
        drv->Release();
        return hr;
    }
    HRESULT STDMETHODCALLTYPE LockServer(BOOL lock) override {
        if (lock) g_objects++;
        else g_objects--;
        return S_OK;
    }

private:
    std::atomic<ULONG> refs_{1};
};

// ------------------------------------------------------------ DLL exports

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_module = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}

extern "C" __declspec(dllexport) HRESULT STDAPICALLTYPE DllGetClassObject(REFCLSID rclsid, REFIID riid, LPVOID* ppv) {
    if (!ppv) return E_POINTER;
    *ppv = nullptr;
    if (!IsEqualCLSID(rclsid, CLSID_DanteBridgeAsio)) return CLASS_E_CLASSNOTAVAILABLE;
    Factory* f = new Factory();
    HRESULT hr = f->QueryInterface(riid, ppv);
    f->Release();
    return hr;
}

extern "C" __declspec(dllexport) HRESULT STDAPICALLTYPE DllCanUnloadNow() {
    return g_objects.load() == 0 ? S_OK : S_FALSE;
}

static bool setValue(const char* subkey, const char* name, const char* value) {
    HKEY h = nullptr;
    LONG r = RegCreateKeyExA(HKEY_LOCAL_MACHINE, subkey, 0, nullptr, 0, KEY_WRITE, nullptr, &h, nullptr);
    if (r != ERROR_SUCCESS) return false;
    r = RegSetValueExA(h, name, 0, REG_SZ, (const BYTE*)value, (DWORD)std::strlen(value) + 1);
    RegCloseKey(h);
    return r == ERROR_SUCCESS;
}

extern "C" __declspec(dllexport) HRESULT STDAPICALLTYPE DllRegisterServer() {
    char path[MAX_PATH];
    if (!GetModuleFileNameA(g_module, path, MAX_PATH)) return E_FAIL;

    char key[256];
    std::snprintf(key, sizeof(key), "SOFTWARE\\Classes\\CLSID\\%s", kClsidString);
    bool ok = setValue(key, nullptr, kDriverName);
    std::snprintf(key, sizeof(key), "SOFTWARE\\Classes\\CLSID\\%s\\InprocServer32", kClsidString);
    ok = setValue(key, nullptr, path) && ok;
    ok = setValue(key, "ThreadingModel", "Apartment") && ok;

    std::snprintf(key, sizeof(key), "SOFTWARE\\ASIO\\%s", kDriverName);
    ok = setValue(key, "CLSID", kClsidString) && ok;
    ok = setValue(key, "Description", kDriverName) && ok;
    return ok ? S_OK : SELFREG_E_CLASS;
}

extern "C" __declspec(dllexport) HRESULT STDAPICALLTYPE DllUnregisterServer() {
    char key[256];
    std::snprintf(key, sizeof(key), "SOFTWARE\\Classes\\CLSID\\%s", kClsidString);
    RegDeleteTreeA(HKEY_LOCAL_MACHINE, key);
    std::snprintf(key, sizeof(key), "SOFTWARE\\ASIO\\%s", kDriverName);
    RegDeleteTreeA(HKEY_LOCAL_MACHINE, key);
    return S_OK;
}
