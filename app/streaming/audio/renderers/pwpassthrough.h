#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

struct pw_thread_loop;
struct pw_stream;
struct spa_hook;

// Plays an IEC 61937 bitstream (Dolby MAT from a Vibepollo host, or AC-3 /
// E-AC-3) to an HDMI output through PipeWire, unchanged, so the receiver,
// TV or soundbar decodes it. moonlight-common-c hands over each rebuilt frame
// (one burst period, e.g. 61440 bytes / 20 ms of MAT) via playBitstream.
//
// PipeWire does no IEC 61937 packing: the stream is opened as audio/iec958
// and the packed bytes are written as S16LE. For MAT the format is TrueHD at
// 192 kHz, which PipeWire's ALSA sink opens as 8 channels with the non-audio
// bit set. The HDMI output must allow the codec ("iec958.codecs", e.g.
// pavucontrol > Advanced > TrueHD); probe() checks that before a session
// advertises passthrough to the host.
//
// libpipewire is loaded at runtime, so builds don't depend on the system's
// PipeWire version and machines without it simply lack the feature.
class PipeWirePassthroughRenderer
{
public:
    PipeWirePassthroughRenderer();
    ~PipeWirePassthroughRenderer();

    // Starts the PipeWire thread. Returns false if libpipewire can't be loaded.
    bool initialize();

    // Opens a TrueHD passthrough stream and waits up to timeoutMs for PipeWire
    // to agree a format with an output. On failure, reason says why.
    static bool probe(int timeoutMs, std::string& reason);

    // Queues one rebuilt frame (AUDIO_BITSTREAM_FLAG_* flags). Never blocks;
    // called on moonlight-common-c's audio thread.
    void submit(const unsigned char* data, int length, int flags);

private:
    enum class Container { None, Mat, Eac3, Ac3 };

    static Container containerForBurst(const unsigned char* data, int length);
    bool openStream(Container container);
    void closeStream();
    void waitForFormat(int timeoutMs);

    // PipeWire stream event handlers (pwpassthrough.cpp)
    friend struct PwStreamCallbacks;

    pw_thread_loop* m_Loop;
    pw_stream* m_Stream;
    spa_hook* m_StreamListener;
    Container m_Container;
    uint32_t m_Stride;
    uint32_t m_BurstBytes;

    // Shared between submit() and the PipeWire thread
    std::mutex m_Lock;
    std::condition_variable m_FormatCv;
    std::deque<std::vector<unsigned char>> m_Frames;
    size_t m_FrontOffset;
    size_t m_QueuedBytes;
    bool m_Primed;
    bool m_WaitForAligned;
    bool m_FormatAgreed;
    bool m_StreamFailed;
    std::string m_StreamError;

    uint32_t m_Underruns;
    uint32_t m_DroppedFrames;
    bool m_LoggedUnknownBurst;
};
