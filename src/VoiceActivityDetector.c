#include "VoiceActivityDetector.h"
#include <math.h>

// Initialize VAD with default threshold and frame params
void vad_init(VoiceActivityDetector *vad, float threshold, int frameLength, int hangoverFrames)
{
    vad->threshold = threshold;
    vad->frame_len = frameLength;
    vad->hangover = hangoverFrames;
    vad->calibrationSum = 0.0f;
    vad->calibrationCount = 0;
}

// Reset calibration accumulation
void vad_reset_calibration(VoiceActivityDetector *vad)
{
    vad->calibrationSum = 0.0f;
    vad->calibrationCount = 0;
}

// Collect a sample for calibration (absolute value)
void vad_collect_calibration_sample(VoiceActivityDetector *vad, int16_t sample)
{
    vad->calibrationSum += fabsf((float)sample);
    vad->calibrationCount++;
}

// Compute adaptive threshold from collected samples
void vad_compute_calibration_threshold(VoiceActivityDetector *vad)
{
    if (vad->calibrationCount == 0)
        return;

    float avg = vad->calibrationSum / vad->calibrationCount;
    float normalized = avg / 32768.0f;
    vad->threshold = normalized * 3.0f;  // Calibration multiplier
}

// Update threshold slightly after speech (adaptive tuning)
void vad_update_threshold_after_speech(VoiceActivityDetector *vad)
{
    vad->threshold *= 0.9f;  // Decay factor
    if (vad->threshold < 0.018f)
        vad->threshold = 0.018f;  // Minimum limit
}

// Perform VAD extraction with hangover
int vad_extract(const VoiceActivityDetector *vad,
                const int16_t *input, int input_len,
                int16_t *output, int max_output_len)
{
    int output_index = 0;
    int num_frames = input_len / vad->frame_len;
    int hangover_count = 0;

    for (int i = 0; i < num_frames; ++i)
    {
        const int16_t *frame = input + i * vad->frame_len;

        float sum = 0;
        for (int j = 0; j < vad->frame_len; ++j)
        {
            sum += fabsf((float)frame[j]);
        }
        float energy = sum / vad->frame_len / 32768.0f;

        int keep = 0;
        if (energy > vad->threshold)
        {
            keep = 1;
            hangover_count = vad->hangover;
        }
        else if (hangover_count > 0)
        {
            keep = 1;
            hangover_count--;
        }

        if (keep)
        {
            if (output_index + vad->frame_len <= max_output_len)
            {
                for (int j = 0; j < vad->frame_len; ++j)
                {
                    output[output_index++] = frame[j];
                }
            }
            else
            {
                break; // prevent overflow
            }
        }
    }

    return output_index; // total samples in output
}
