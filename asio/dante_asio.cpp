// Dante Bridge ASIO driver - STAGE B
// ASIO driver: 4 inputs / 4 outputs, 48 kHz, Int32 samples.
// Audio comes from / goes to DanteBridge.exe through a memory-mapped file
// (C:\Users\Public\DanteBridge\asio_shm.bin, same layout as bridge/shm.rs):
//   ASIO inputs  <- Dante receive channels 1-4
//   ASIO outputs -> Dante transmit channels 1-4
// The Stage A version (test tones) is kept in stageA/.
//
// The ASIO interface is declared here from its public binary layout
// (no Steinberg SDK files are used). ASIO is a trademark of Steinberg
// Media Technologies GmbH. 64-bit build only.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <objbase.h>
#include <olectl.h>  // SELFREG_E_CLASS
#include <mmsystem.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <string>
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

// {7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}
static const GUID CLSID_DanteBridgeAsio =
    {0x7b1d3e52, 0x9c4a, 0x4f0e, {0x8a, 0x2b, 0x5d, 0x6e, 0x7f, 0x80, 0x9a, 0x1b}};
static const char* kDriverName = "Dante Bridge ASIO";
static const char* kClsidString = "{7B1D3E52-9C4A-4F0E-8A2B-5D6E7F809A1B}";

static const int kInputs = 4;
static const int kOutputs = 4;
static const double kRate = 48000.0;
static const long kMinBuf = 64, kMaxBuf = 2048, kPrefBuf = 512;

static HMODULE g_module = nullptr;
static std::atomic<long> g_objects{0};

// ------------------------------------------------------------ shared memory (DanteBridge.exe)

namespace shm {
constexpr int CH = 4;
constexpr size_t FRAMES = 32768;  // frames per ring, power of two
constexpr size_t HEADER = 128;
constexpr size_t RING_BYTES = FRAMES * CH * 4;
constexpr size_t TOTAL = HEADER + 2 * RING_BYTES;
constexpr size_t OFF_IN_W = 16, OFF_OUT_W = 24, OFF_BRIDGE_BEAT = 32, OFF_DRIVER_BEAT = 40;
constexpr size_t OFF_FLAGS = 4, OFF_DRIVER_VER = 8, OFF_IN_UNDERRUNS = 56, OFF_IN_LEAD = 64, OFF_IN_PPM = 72;
constexpr uint32_t DRIVER_VERSION = 2;   // must match shm.rs DRIVER_VERSION
constexpr uint32_t FLAG_RX_SMOOTH = 1;   // header flag set by the bridge
enum Dir { In = 0, Out = 1 };

class Mapping {
public:
    ~Mapping() { close(); }

    bool open() {
        if (base_) return true;
        char dir[MAX_PATH];
        DWORD n = GetEnvironmentVariableA("PUBLIC", dir, MAX_PATH);
        if (n == 0 || n >= MAX_PATH) n = GetEnvironmentVariableA("PROGRAMDATA", dir, MAX_PATH);
        if (n == 0 || n >= MAX_PATH) return false;
        std::string folder = std::string(dir) + "\\DanteBridge";
        CreateDirectoryA(folder.c_str(), nullptr);
        std::string path = folder + "\\asio_shm.bin";
        file_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                            FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_ALWAYS,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
        if (file_ == INVALID_HANDLE_VALUE) {
            file_ = nullptr;
            return false;
        }
        LARGE_INTEGER sz;
        if (!GetFileSizeEx(file_, &sz)) sz.QuadPart = 0;
        if ((size_t)sz.QuadPart < TOTAL) {
            sz.QuadPart = (LONGLONG)TOTAL;
            SetFilePointerEx(file_, sz, nullptr, FILE_BEGIN);
            SetEndOfFile(file_);
        }
        map_ = CreateFileMappingA(file_, nullptr, PAGE_READWRITE, 0, (DWORD)TOTAL, nullptr);
        if (!map_) {
            close();
            return false;
        }
        base_ = (uint8_t*)MapViewOfFile(map_, FILE_MAP_ALL_ACCESS, 0, 0, TOTAL);
        if (!base_) {
            close();
            return false;
        }
        set_driver_version(DRIVER_VERSION);
        return true;
    }

    void close() {
        if (base_) UnmapViewOfFile(base_);
        if (map_) CloseHandle(map_);
        if (file_) CloseHandle(file_);
        base_ = nullptr;
        map_ = nullptr;
        file_ = nullptr;
    }

    bool ok() const { return base_ != nullptr; }

    uint64_t write_pos(Dir d) const { return a64(d == In ? OFF_IN_W : OFF_OUT_W)->load(std::memory_order_acquire); }
    void set_write_pos(Dir d, uint64_t v) { a64(d == In ? OFF_IN_W : OFF_OUT_W)->store(v, std::memory_order_release); }
    uint32_t flags() const { return a32(OFF_FLAGS)->load(std::memory_order_relaxed); }
    void set_driver_version(uint32_t v) { a32(OFF_DRIVER_VER)->store(v, std::memory_order_relaxed); }
    void set_in_metrics(uint64_t underruns, int64_t lead, int64_t ppm) {
        a64(OFF_IN_UNDERRUNS)->store(underruns, std::memory_order_relaxed);
        a64(OFF_IN_LEAD)->store((uint64_t)lead, std::memory_order_relaxed);
        a64(OFF_IN_PPM)->store((uint64_t)ppm, std::memory_order_relaxed);
    }
    uint64_t bridge_beat() const { return a64(OFF_BRIDGE_BEAT)->load(std::memory_order_relaxed); }
    void bump_driver_beat() { a64(OFF_DRIVER_BEAT)->fetch_add(1, std::memory_order_relaxed); }

    // data: interleaved frames (CH floats per frame)
    void write_frames(Dir d, uint64_t pos, const float* data, size_t frames) {
        std::atomic<uint32_t>* ring = ring_(d);
        for (size_t i = 0; i < frames; i++) {
            size_t slot = (size_t)(pos + i) & (FRAMES - 1);
            for (int c = 0; c < CH; c++) {
                uint32_t bits;
                std::memcpy(&bits, &data[i * CH + c], 4);
                ring[slot * CH + c].store(bits, std::memory_order_relaxed);
            }
        }
    }

    void read_frames(Dir d, uint64_t pos, float* out, size_t frames) const {
        std::atomic<uint32_t>* ring = ring_(d);
        for (size_t i = 0; i < frames; i++) {
            size_t slot = (size_t)(pos + i) & (FRAMES - 1);
            for (int c = 0; c < CH; c++) {
                uint32_t bits = ring[slot * CH + c].load(std::memory_order_relaxed);
                std::memcpy(&out[i * CH + c], &bits, 4);
            }
        }
    }

private:
    std::atomic<uint32_t>* a32(size_t off) const { return reinterpret_cast<std::atomic<uint32_t>*>(base_ + off); }
    std::atomic<uint64_t>* a64(size_t off) const { return reinterpret_cast<std::atomic<uint64_t>*>(base_ + off); }
    std::atomic<uint32_t>* ring_(Dir d) const {
        return reinterpret_cast<std::atomic<uint32_t>*>(base_ + HEADER + (size_t)d * RING_BYTES);
    }
    HANDLE file_ = nullptr;
    HANDLE map_ = nullptr;
    uint8_t* base_ = nullptr;
};
}  // namespace shm

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
        map_.open();  // if it fails the driver still loads and just outputs silence
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
        *inputLatency = b + 480;   // approximate: buffer + shared-memory queue
        *outputLatency = b + 480;
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
        std::snprintf(info->name, sizeof(info->name), "%s %d", info->isInput ? "Dante In" : "Dante Out",
                      (int)info->channel + 1);
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

    // Reads one block (n frames) of Dante audio from the shared ring.
    // Keeps ~20 ms in the queue; corrects clock drift by dropping/repeating one sample.
    void readInput(size_t n, float* out) {
        if (map_.ok()) {
            // If the bridge is not writing (stopped, crashed, not started) the write position does not
            // move. Never replay the old tail of the ring in that case: output silence.
            const uint64_t wp = map_.write_pos(shm::In);
            if (wp == in_last_wp_) {
                std::fill(out, out + n * shm::CH, 0.f);
                in_started_ = false;
                return;
            }
            in_last_wp_ = wp;
        }
        if (map_.ok() && (map_.flags() & shm::FLAG_RX_SMOOTH) != 0) {
            readInputSmooth(n, out);
        } else {
            readInputLegacy(n, out);
        }
    }

    // Smooth version: the queue lead is low-pass filtered and corrected by a tiny, continuous
    // change of the read speed (linear interpolation, at most +-1000 ppm). No sample is dropped.
    void readInputSmooth(size_t n, float* out) {
        const size_t total = n * shm::CH;
        const uint64_t wp = map_.write_pos(shm::In);
        const int64_t target = std::max<int64_t>(2 * (int64_t)n, 960);
        int64_t avail = (int64_t)(wp - in_rd_);
        if (!in_started_ || avail < 0 || avail > target * 4) {
            in_rd_ = wp > (uint64_t)target ? wp - (uint64_t)target : 0;
            in_frac_ = 0.0;
            in_ema_ = 0.0;
            in_started_ = true;
            avail = (int64_t)(wp - in_rd_);
        }
        // positive lead = more audio queued than needed -> read slightly faster
        const double alpha = std::min(1.0, (double)n / ((double)kRate * 0.5));
        in_ema_ += ((double)(avail - target) - in_ema_) * alpha;
        double rho = in_ema_ / ((double)kRate * 2.0);
        rho = std::max(-0.001, std::min(0.001, rho));
        const double step = 1.0 + rho;
        const size_t need = (size_t)std::floor(in_frac_ + (double)(n - 1) * step) + 2;
        if (avail < (int64_t)need) {  // not enough data: silence, re-prime on the next block
            std::fill(out, out + total, 0.f);
            in_started_ = false;
            in_underruns_++;
            map_.set_in_metrics(in_underruns_, avail - target, (int64_t)std::llround(rho * 1e6));
            return;
        }
        tmp_.resize(need * shm::CH);
        map_.read_frames(shm::In, in_rd_, tmp_.data(), need);
        for (size_t i = 0; i < n; i++) {
            const double p = in_frac_ + (double)i * step;
            const size_t i0 = (size_t)p;
            const float f = (float)(p - (double)i0);
            for (int c = 0; c < shm::CH; c++) {
                const float a = tmp_[i0 * shm::CH + (size_t)c];
                const float b = tmp_[(i0 + 1) * shm::CH + (size_t)c];
                out[i * shm::CH + (size_t)c] = a + (b - a) * f;
            }
        }
        const double end = in_frac_ + (double)n * step;
        const size_t adv = (size_t)std::floor(end);
        in_frac_ = end - (double)adv;
        in_rd_ += (uint64_t)adv;
        map_.set_in_metrics(in_underruns_, avail - target, (int64_t)std::llround(rho * 1e6));
    }

    // Legacy version (Stage B v1): drops or repeats one sample when the lead leaves +-5 ms.
    void readInputLegacy(size_t n, float* out) {
        const size_t total = n * shm::CH;
        if (!map_.ok()) {
            std::fill(out, out + total, 0.f);
            return;
        }
        uint64_t wp = map_.write_pos(shm::In);
        int64_t target = std::max<int64_t>(2 * (int64_t)n, 960);
        int64_t avail = (int64_t)(wp - in_rd_);
        if (!in_started_ || avail < 0 || avail > target * 4) {
            in_rd_ = wp > (uint64_t)target ? wp - (uint64_t)target : 0;
            in_started_ = true;
            avail = (int64_t)(wp - in_rd_);
        }
        if (avail < (int64_t)n) {  // not enough data: silence, re-prime on the next block
            std::fill(out, out + total, 0.f);
            in_started_ = false;
            return;
        }
        int64_t e = avail - target;
        int64_t s = (int64_t)n;
        if (e > 240) s += 1;
        else if (e < -240) s -= 1;
        if (s > avail) s = avail;
        tmp_.resize((size_t)s * shm::CH);
        map_.read_frames(shm::In, in_rd_, tmp_.data(), (size_t)s);
        for (size_t i = 0; i < n; i++) {
            size_t src = std::min<size_t>(i, (size_t)s - 1);
            for (int c = 0; c < shm::CH; c++) out[i * shm::CH + (size_t)c] = tmp_[src * shm::CH + (size_t)c];
        }
        in_rd_ += (uint64_t)s;
    }

    void run() {
        LARGE_INTEGER freq, t0;
        QueryPerformanceFrequency(&freq);
        QueryPerformanceCounter(&t0);
        const double period_ticks = (double)freq.QuadPart * (double)buffer_size_ / kRate;
        std::vector<float> inblk((size_t)buffer_size_ * shm::CH, 0.f), outblk((size_t)buffer_size_ * shm::CH, 0.f);
        uint64_t out_wp = map_.ok() ? map_.write_pos(shm::Out) : 0;
        in_started_ = false;
        in_last_wp_ = map_.ok() ? map_.write_pos(shm::In) : 0;
        // The bridge is considered NOT running until its heartbeat is seen moving. Without this the
        // driver would replay the last (stale) audio left in the shared file by a previous session.
        last_bridge_beat_ = map_.ok() ? map_.bridge_beat() : 0;
        last_bridge_change_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
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

            // inputs: Dante -> ASIO (shared memory written by DanteBridge.exe)
            const uint64_t bridge_beat = map_.ok() ? map_.bridge_beat() : 0;
            const auto now_tp = std::chrono::steady_clock::now();
            if (bridge_beat != last_bridge_beat_) {
                last_bridge_beat_ = bridge_beat;
                last_bridge_change_ = now_tp;
            }
            const bool bridge_alive =
                map_.ok() && (now_tp - last_bridge_change_) < std::chrono::milliseconds(500);
            if (bridge_alive) {
                readInput((size_t)buffer_size_, inblk.data());
            } else {
                std::fill(inblk.begin(), inblk.end(), 0.f);  // bridge stopped: silence
                in_started_ = false;
            }
            for (size_t k = 0; k < buffers_.size(); k++) {
                if (!buffers_[k].isInput) continue;
                int ch = (int)buffers_[k].channelNum;
                if (ch < 0 || ch >= kInputs) continue;
                int32_t* dst = (int32_t*)buffers_[k].buffers[idx];
                for (long i = 0; i < buffer_size_; i++) {
                    float v = inblk[(size_t)i * shm::CH + (size_t)ch];
                    if (v > 1.f) v = 1.f;
                    else if (v < -1.f) v = -1.f;
                    dst[i] = (int32_t)((double)v * 2147483647.0);
                }
            }

            sample_pos_ = n * (uint64_t)buffer_size_;
            stamp_ns_ = (uint64_t)((double)(n * (uint64_t)buffer_size_) * 1e9 / kRate);
            if (callbacks_ && callbacks_->bufferSwitch) callbacks_->bufferSwitch(idx, ASIOTrue);

            // outputs: ASIO -> Dante (shared memory read by DanteBridge.exe)
            if (map_.ok()) {
                std::fill(outblk.begin(), outblk.end(), 0.f);
                for (size_t k = 0; k < buffers_.size(); k++) {
                    if (buffers_[k].isInput) continue;
                    int ch = (int)buffers_[k].channelNum;
                    if (ch < 0 || ch >= kOutputs) continue;
                    const int32_t* src = (const int32_t*)buffers_[k].buffers[idx];
                    for (long i = 0; i < buffer_size_; i++) {
                        outblk[(size_t)i * shm::CH + (size_t)ch] = (float)((double)src[i] / 2147483648.0);
                    }
                }
                map_.write_frames(shm::Out, out_wp, outblk.data(), (size_t)buffer_size_);
                out_wp += (uint64_t)buffer_size_;
                map_.set_write_pos(shm::Out, out_wp);
                map_.bump_driver_beat();
            }
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
    shm::Mapping map_;
    uint64_t in_rd_ = 0;
    uint64_t in_last_wp_ = 0;  // write position seen on the previous block (detects a stopped bridge)
    uint64_t last_bridge_beat_ = 0;
    std::chrono::steady_clock::time_point last_bridge_change_{};
    double in_frac_ = 0.0;
    double in_ema_ = 0.0;
    uint64_t in_underruns_ = 0;
    bool in_started_ = false;
    std::vector<float> tmp_;
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

// The Windows SDK already declares these four functions (without dllexport), so they are
// exported by name through the linker instead of __declspec(dllexport).
#pragma comment(linker, "/EXPORT:DllGetClassObject,PRIVATE")
#pragma comment(linker, "/EXPORT:DllCanUnloadNow,PRIVATE")
#pragma comment(linker, "/EXPORT:DllRegisterServer,PRIVATE")
#pragma comment(linker, "/EXPORT:DllUnregisterServer,PRIVATE")

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_module = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}

extern "C" HRESULT STDAPICALLTYPE DllGetClassObject(REFCLSID rclsid, REFIID riid, LPVOID* ppv) {
    if (!ppv) return E_POINTER;
    *ppv = nullptr;
    if (!IsEqualCLSID(rclsid, CLSID_DanteBridgeAsio)) return CLASS_E_CLASSNOTAVAILABLE;
    Factory* f = new Factory();
    HRESULT hr = f->QueryInterface(riid, ppv);
    f->Release();
    return hr;
}

extern "C" HRESULT STDAPICALLTYPE DllCanUnloadNow() {
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

extern "C" HRESULT STDAPICALLTYPE DllRegisterServer() {
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

extern "C" HRESULT STDAPICALLTYPE DllUnregisterServer() {
    char key[256];
    std::snprintf(key, sizeof(key), "SOFTWARE\\Classes\\CLSID\\%s", kClsidString);
    RegDeleteTreeA(HKEY_LOCAL_MACHINE, key);
    std::snprintf(key, sizeof(key), "SOFTWARE\\ASIO\\%s", kDriverName);
    RegDeleteTreeA(HKEY_LOCAL_MACHINE, key);
    return S_OK;
}
