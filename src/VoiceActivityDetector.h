#ifndef VOICE_ACTIVITY_DETECTOR_H
#define VOICE_ACTIVITY_DETECTOR_H

#include <stdint.h>

#define MAX_VAD_OUTPUT 24000

#ifdef __cplusplus
extern "C"
{
#endif

    // --- VAD Struct ---
    typedef struct
    {
        float threshold;      // Current adaptive threshold
        int frame_len;        // Frame length in samples
        int hangover;         // Hangover frames count
        float calibrationSum; // Sum of calibration samples
        int calibrationCount; // Number of calibration samples collected
    } VoiceActivityDetector;

    // --- VAD Functions ---
    void vad_init(VoiceActivityDetector *vad, float threshold, int frameLength, int hangoverFrames);

    void vad_reset_calibration(VoiceActivityDetector *vad);

    void vad_collect_calibration_sample(VoiceActivityDetector *vad, int16_t sample);

    void vad_compute_calibration_threshold(VoiceActivityDetector *vad);

    void vad_update_threshold_after_speech(VoiceActivityDetector *vad);

    int vad_extract(const VoiceActivityDetector *vad,
                    const int16_t *input, int input_len,
                    int16_t *output, int max_output_len);

#ifdef __cplusplus
}
#endif

#endif
