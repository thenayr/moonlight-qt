#include "pwpassthrough.h"

#include <Limelight.h>
#include "SDL_compat.h"

#include <dlfcn.h>
#include <algorithm>
#include <chrono>
#include <cstring>

#include <pipewire/pipewire.h>
#include <spa/param/audio/format.h>
#include <spa/param/audio/iec958.h>
// SPA_PARAM_BUFFERS_*: defined here before PipeWire 0.3.5x, included from here after
#include <spa/param/param.h>
#include <spa/pod/builder.h>

namespace {

// Frames queued before output starts, and the most allowed before the oldest
// are dropped to bound latency when the host's clock runs ahead of HDMI's.
constexpr size_t kPrebufferFrames = 2;
constexpr size_t kMaxQueuedFrames = 6;

// IEC 61937 burst data types (Pc & 0x7F)
constexpr int kBurstTypeAc3 = 1;
constexpr int kBurstTypeEac3 = 21;
constexpr int kBurstTypeMat = 22;

// The libpipewire entry points we use, resolved from libpipewire-0.3.so.0.
struct PwApi {
    decltype(&pw_init) init;
    decltype(&pw_thread_loop_new) thread_loop_new;
    decltype(&pw_thread_loop_destroy) thread_loop_destroy;
    decltype(&pw_thread_loop_start) thread_loop_start;
    decltype(&pw_thread_loop_stop) thread_loop_stop;
    decltype(&pw_thread_loop_lock) thread_loop_lock;
    decltype(&pw_thread_loop_unlock) thread_loop_unlock;
    decltype(&pw_thread_loop_get_loop) thread_loop_get_loop;
    decltype(&pw_properties_new) properties_new;
    decltype(&pw_properties_setf) properties_setf;
    decltype(&pw_stream_new_simple) stream_new_simple;
    decltype(&pw_stream_destroy) stream_destroy;
    decltype(&pw_stream_connect) stream_connect;
    decltype(&pw_stream_dequeue_buffer) stream_dequeue_buffer;
    decltype(&pw_stream_queue_buffer) stream_queue_buffer;
    decltype(&pw_stream_state_as_string) stream_state_as_string;
};

PwApi* loadPipeWire()
{
    static std::once_flag once;
    static PwApi api;
    static bool loaded = false;

    std::call_once(once, [] {
        void* lib = dlopen("libpipewire-0.3.so.0", RTLD_NOW | RTLD_LOCAL);
        if (lib == nullptr) {
            SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                        "Audio passthrough unavailable: %s", dlerror());
            return;
        }

        bool ok = true;
        auto resolve = [&](auto& fn, const char* name) {
            fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(dlsym(lib, name));
            if (fn == nullptr) {
                SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                            "Audio passthrough unavailable: libpipewire lacks %s", name);
                ok = false;
            }
        };
        resolve(api.init, "pw_init");
        resolve(api.thread_loop_new, "pw_thread_loop_new");
        resolve(api.thread_loop_destroy, "pw_thread_loop_destroy");
        resolve(api.thread_loop_start, "pw_thread_loop_start");
        resolve(api.thread_loop_stop, "pw_thread_loop_stop");
        resolve(api.thread_loop_lock, "pw_thread_loop_lock");
        resolve(api.thread_loop_unlock, "pw_thread_loop_unlock");
        resolve(api.thread_loop_get_loop, "pw_thread_loop_get_loop");
        resolve(api.properties_new, "pw_properties_new");
        resolve(api.properties_setf, "pw_properties_setf");
        resolve(api.stream_new_simple, "pw_stream_new_simple");
        resolve(api.stream_destroy, "pw_stream_destroy");
        resolve(api.stream_connect, "pw_stream_connect");
        resolve(api.stream_dequeue_buffer, "pw_stream_dequeue_buffer");
        resolve(api.stream_queue_buffer, "pw_stream_queue_buffer");
        resolve(api.stream_state_as_string, "pw_stream_state_as_string");
        if (!ok) {
            dlclose(lib);
            return;
        }

        api.init(nullptr, nullptr);
        loaded = true;
    });

    return loaded ? &api : nullptr;
}

} // namespace

struct PwStreamCallbacks {
    static void onStateChanged(void* data, enum pw_stream_state, enum pw_stream_state state, const char* error)
    {
        auto* self = static_cast<PipeWirePassthroughRenderer*>(data);
        PwApi* pw = loadPipeWire();

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Audio passthrough stream: %s%s%s",
                    pw->stream_state_as_string(state), error ? ": " : "", error ? error : "");

        if (state == PW_STREAM_STATE_ERROR || state == PW_STREAM_STATE_UNCONNECTED) {
            std::lock_guard<std::mutex> lock(self->m_Lock);
            self->m_StreamFailed = true;
            self->m_StreamError = error ? error : pw->stream_state_as_string(state);
            self->m_FormatCv.notify_all();
        }
    }

    static void onParamChanged(void* data, uint32_t id, const struct spa_pod* param)
    {
        auto* self = static_cast<PipeWirePassthroughRenderer*>(data);

        // PipeWire only negotiates audio/iec958 with an output that allows the codec
        if (id == SPA_PARAM_Format && param != nullptr) {
            std::lock_guard<std::mutex> lock(self->m_Lock);
            self->m_FormatAgreed = true;
            self->m_FormatCv.notify_all();
        }
    }

    static void onProcess(void* data)
    {
        auto* self = static_cast<PipeWirePassthroughRenderer*>(data);
        PwApi* pw = loadPipeWire();

        struct pw_buffer* b = pw->stream_dequeue_buffer(self->m_Stream);
        if (b == nullptr) {
            return;
        }

        struct spa_data* d = &b->buffer->datas[0];
        auto* dst = static_cast<unsigned char*>(d->data);
        if (dst == nullptr) {
            pw->stream_queue_buffer(self->m_Stream, b);
            return;
        }

        uint32_t maxFrames = d->maxsize / self->m_Stride;
#if PW_CHECK_VERSION(0, 3, 50)
        uint32_t frames = b->requested ? uint32_t(std::min<uint64_t>(b->requested, maxFrames)) : maxFrames;
#else
        // pw_buffer.requested arrived in 0.3.50; older graphs take a full buffer
        uint32_t frames = maxFrames;
#endif
        size_t wanted = size_t(frames) * self->m_Stride;

        {
            std::lock_guard<std::mutex> lock(self->m_Lock);

            if (!self->m_Primed && self->m_QueuedBytes >= kPrebufferFrames * self->m_BurstBytes) {
                self->m_Primed = true;
            }

            if (self->m_Primed && self->m_QueuedBytes >= wanted) {
                // The byte stream is continuous, so burst spacing holds across quanta
                size_t written = 0;
                while (written < wanted) {
                    auto& front = self->m_Frames.front();
                    size_t n = std::min(wanted - written, front.size() - self->m_FrontOffset);
                    memcpy(dst + written, front.data() + self->m_FrontOffset, n);
                    written += n;
                    self->m_FrontOffset += n;
                    if (self->m_FrontOffset == front.size()) {
                        self->m_Frames.pop_front();
                        self->m_FrontOffset = 0;
                    }
                }
                self->m_QueuedBytes -= wanted;
            }
            else {
                // Underrun: the receiver gets silence and resyncs on the next burst.
                // Refill to the prebuffer level before resuming.
                memset(dst, 0, wanted);
                if (self->m_Primed) {
                    self->m_Primed = false;
                    self->m_Underruns++;
                }
            }
        }

        d->chunk->offset = 0;
        d->chunk->stride = self->m_Stride;
        d->chunk->size = uint32_t(wanted);
        b->size = frames;
        pw->stream_queue_buffer(self->m_Stream, b);
    }
};

PipeWirePassthroughRenderer::PipeWirePassthroughRenderer()
    : m_Loop(nullptr),
      m_Stream(nullptr),
      m_StreamListener(new spa_hook()),
      m_Container(Container::None),
      m_Stride(0),
      m_BurstBytes(0),
      m_FrontOffset(0),
      m_QueuedBytes(0),
      m_Primed(false),
      m_WaitForAligned(true),
      m_FormatAgreed(false),
      m_StreamFailed(false),
      m_Underruns(0),
      m_DroppedFrames(0),
      m_LoggedUnknownBurst(false)
{
}

PipeWirePassthroughRenderer::~PipeWirePassthroughRenderer()
{
    PwApi* pw = loadPipeWire();

    if (m_Loop != nullptr) {
        closeStream();
        pw->thread_loop_stop(m_Loop);
        pw->thread_loop_destroy(m_Loop);
    }

    if (m_Underruns || m_DroppedFrames) {
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "Audio passthrough: %u underruns, %u frames dropped to bound latency",
                    m_Underruns, m_DroppedFrames);
    }
    delete m_StreamListener;
}

bool PipeWirePassthroughRenderer::initialize()
{
    PwApi* pw = loadPipeWire();
    if (pw == nullptr) {
        return false;
    }

    m_Loop = pw->thread_loop_new("moonlight-passthrough", nullptr);
    if (m_Loop == nullptr || pw->thread_loop_start(m_Loop) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Audio passthrough: couldn't start the PipeWire thread");
        if (m_Loop != nullptr) {
            pw->thread_loop_destroy(m_Loop);
            m_Loop = nullptr;
        }
        return false;
    }
    return true;
}

PipeWirePassthroughRenderer::Container PipeWirePassthroughRenderer::containerForBurst(const unsigned char* data, int length)
{
    // Pa/Pb sync words as 16-bit little-endian samples, then Pc's data type
    if (length < 8 || data[0] != 0x72 || data[1] != 0xF8 || data[2] != 0x1F || data[3] != 0x4E) {
        return Container::None;
    }

    switch (data[4] & 0x7F) {
    case kBurstTypeMat:
        return Container::Mat;
    case kBurstTypeEac3:
        return Container::Eac3;
    case kBurstTypeAc3:
        return Container::Ac3;
    default:
        return Container::None;
    }
}

bool PipeWirePassthroughRenderer::openStream(Container container)
{
    PwApi* pw = loadPipeWire();

    uint32_t codec, rate, channels;
    switch (container) {
    case Container::Mat:
        // PipeWire has no MAT codec; TrueHD is the same 8 ch / 192 kHz HBR container
        codec = SPA_AUDIO_IEC958_CODEC_TRUEHD;
        rate = 192000;
        channels = 8;
        break;
    case Container::Eac3:
        codec = SPA_AUDIO_IEC958_CODEC_EAC3;
        rate = 192000;
        channels = 2;
        break;
    case Container::Ac3:
        codec = SPA_AUDIO_IEC958_CODEC_AC3;
        rate = 48000;
        channels = 2;
        break;
    default:
        return false;
    }

    m_Container = container;
    m_Stride = channels * 2;
    // One 20 ms burst period per PipeWire quantum (MAT and E-AC-3 are 20 ms;
    // AC-3's 32 ms burst simply spans quanta)
    uint32_t quantum = rate / 50;
    m_BurstBytes = quantum * m_Stride;

    {
        std::lock_guard<std::mutex> lock(m_Lock);
        m_FormatAgreed = false;
        m_StreamFailed = false;
        m_StreamError.clear();
    }

    struct pw_properties* props = pw->properties_new(
        PW_KEY_MEDIA_TYPE, "Audio",
        PW_KEY_MEDIA_CATEGORY, "Playback",
        PW_KEY_MEDIA_ROLE, "Movie",
        PW_KEY_APP_NAME, "Moonlight",
        PW_KEY_NODE_NAME, "moonlight-passthrough",
        PW_KEY_NODE_DESCRIPTION, "Moonlight Dolby Atmos passthrough",
        // Don't let WirePlumber move a bitstream to an output that can't pass it through
        PW_KEY_NODE_DONT_RECONNECT, "true",
        nullptr);
    pw->properties_setf(props, PW_KEY_NODE_LATENCY, "%u/%u", quantum, rate);
    pw->properties_setf(props, PW_KEY_NODE_RATE, "1/%u", rate);

    static struct pw_stream_events events;
    events.version = PW_VERSION_STREAM_EVENTS;
    events.state_changed = PwStreamCallbacks::onStateChanged;
    events.param_changed = PwStreamCallbacks::onParamChanged;
    events.process = PwStreamCallbacks::onProcess;

    pw->thread_loop_lock(m_Loop);

    m_Stream = pw->stream_new_simple(pw->thread_loop_get_loop(m_Loop), "Moonlight passthrough",
                                     props, &events, this);
    if (m_Stream == nullptr) {
        pw->thread_loop_unlock(m_Loop);
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Audio passthrough: couldn't create a PipeWire stream");
        return false;
    }

    uint8_t podBuffer[1024];
    struct spa_pod_builder builder;
    spa_pod_builder_init(&builder, podBuffer, sizeof(podBuffer));

    const struct spa_pod* params[2];
    params[0] = static_cast<const struct spa_pod*>(spa_pod_builder_add_object(&builder,
        SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat,
        SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_audio),
        SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_iec958),
        SPA_FORMAT_AUDIO_iec958Codec, SPA_POD_Id(codec),
        SPA_FORMAT_AUDIO_rate, SPA_POD_Int(rate)));
    params[1] = static_cast<const struct spa_pod*>(spa_pod_builder_add_object(&builder,
        SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
        SPA_PARAM_BUFFERS_blocks, SPA_POD_Int(1),
        SPA_PARAM_BUFFERS_size, SPA_POD_CHOICE_RANGE_Int(int(m_BurstBytes), int(m_BurstBytes), INT32_MAX),
        SPA_PARAM_BUFFERS_stride, SPA_POD_Int(int(m_Stride))));

    int err = pw->stream_connect(m_Stream, PW_DIRECTION_OUTPUT, PW_ID_ANY,
                                 static_cast<enum pw_stream_flags>(PW_STREAM_FLAG_AUTOCONNECT |
                                                                   PW_STREAM_FLAG_MAP_BUFFERS |
                                                                   PW_STREAM_FLAG_EXCLUSIVE),
                                 params, 2);
    pw->thread_loop_unlock(m_Loop);

    if (err < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Audio passthrough: pw_stream_connect() failed: %d", err);
        closeStream();
        return false;
    }

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "Audio passthrough: opening %s, %u Hz, %u channels",
                container == Container::Mat ? "Dolby MAT (as TrueHD)" : container == Container::Eac3 ? "E-AC-3" : "AC-3",
                rate, channels);
    return true;
}

void PipeWirePassthroughRenderer::closeStream()
{
    PwApi* pw = loadPipeWire();

    if (m_Stream != nullptr) {
        pw->thread_loop_lock(m_Loop);
        pw->stream_destroy(m_Stream);
        m_Stream = nullptr;
        pw->thread_loop_unlock(m_Loop);
    }
    m_Container = Container::None;

    std::lock_guard<std::mutex> lock(m_Lock);
    m_Frames.clear();
    m_FrontOffset = 0;
    m_QueuedBytes = 0;
    m_Primed = false;
}

void PipeWirePassthroughRenderer::waitForFormat(int timeoutMs)
{
    std::unique_lock<std::mutex> lock(m_Lock);
    m_FormatCv.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                        [this] { return m_FormatAgreed || m_StreamFailed; });
}

bool PipeWirePassthroughRenderer::probe(int timeoutMs, std::string& reason)
{
    PipeWirePassthroughRenderer renderer;
    if (!renderer.initialize()) {
        reason = "libpipewire couldn't be loaded";
        return false;
    }
    if (!renderer.openStream(Container::Mat)) {
        reason = "PipeWire refused the passthrough stream";
        return false;
    }

    renderer.waitForFormat(timeoutMs);

    std::lock_guard<std::mutex> lock(renderer.m_Lock);
    if (renderer.m_FormatAgreed) {
        return true;
    }
    if (renderer.m_StreamFailed) {
        reason = "PipeWire couldn't connect it (" + renderer.m_StreamError + ")";
    }
    else {
        reason = "no output accepted TrueHD passthrough; allow it for your HDMI output "
                 "(pavucontrol > Output Devices > Advanced > TrueHD), and make sure nothing "
                 "else is playing to it";
    }
    return false;
}

void PipeWirePassthroughRenderer::submit(const unsigned char* data, int length, int flags)
{
    if (length <= 0) {
        return;
    }

    bool aligned = (flags & AUDIO_BITSTREAM_FLAG_BURST_ALIGNED) != 0;

    if (flags & AUDIO_BITSTREAM_FLAG_DISCONTINUITY) {
        // Drop what's queued and resume at a burst boundary
        std::lock_guard<std::mutex> lock(m_Lock);
        m_Frames.clear();
        m_FrontOffset = 0;
        m_QueuedBytes = 0;
        m_Primed = false;
        m_WaitForAligned = true;
    }

    if (aligned) {
        Container container = containerForBurst(data, length);
        if (container == Container::None) {
            if (!m_LoggedUnknownBurst) {
                SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                            "Audio passthrough: unsupported IEC 61937 burst type %d", data[4] & 0x7F);
                m_LoggedUnknownBurst = true;
            }
            return;
        }

        // A new container (e.g. AC-3 after MAT) needs a stream with that format
        if (container != m_Container) {
            closeStream();
            if (!openStream(container)) {
                return;
            }
        }

        std::lock_guard<std::mutex> lock(m_Lock);
        m_WaitForAligned = false;
    }

    std::lock_guard<std::mutex> lock(m_Lock);
    if (m_WaitForAligned || m_Stream == nullptr) {
        return;
    }

    m_Frames.emplace_back(data, data + length);
    m_QueuedBytes += size_t(length);

    // Bound latency if the host's clock runs ahead of the HDMI clock. Drop whole
    // frames behind the one being played, so output stays on burst boundaries.
    if (m_Frames.size() > kMaxQueuedFrames) {
        while (m_Frames.size() > kPrebufferFrames) {
            m_QueuedBytes -= m_Frames[1].size();
            m_Frames.erase(m_Frames.begin() + 1);
            m_DroppedFrames++;
        }
    }
}
