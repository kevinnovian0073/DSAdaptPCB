#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "driver/adc.h"
#include "esp_timer.h"
#include "VoiceActivityDetector.h"

// --- Pin Definitions ---
#define BUTTON_PIN 40
#define ADC_MIC_PIN 5

#if !defined(CONFIG_IDF_TARGET_ESP32)
#define VSPI FSPI
#endif

#define SD_MISO 12
#define SD_MOSI 11
#define SD_SCK 10
#define SD_CS 14

#define SAMPLE_RATE 16000
#define BUFFER_SIZE 512
#define TOTAL_SAMPLES 24000
#define DEBOUNCE_TIME 200
#define MAX_VAD_OUTPUT 24000

int16_t fullBuffer[TOTAL_SAMPLES];
int16_t vad_output[MAX_VAD_OUTPUT];

// HPF for 100Hz
float hpf_b[3] = {0.9726, -1.9452, 0.9726};
float hpf_a[3] = {1.0000, -1.9445, 0.9460};
float hpf_x[3] = {0, 0, 0}, hpf_y[3] = {0, 0, 0};

// BPF 300–3400Hz
float b[3] = {0.4107, 0, -0.4107};
float a[3] = {1.0000, -1.0739, 0.1786};
float x[3] = {0, 0, 0}, y[3] = {0, 0, 0};

volatile bool recording = false;
unsigned long lastButtonPressTime = 0;
bool lastButtonState = HIGH;

int16_t buffer[BUFFER_SIZE];
volatile size_t bufferIndex = 0;
volatile size_t totalSamplesRecorded = 0;
volatile bool bufferReadyToWrite = false;

File audioFile;
String filename;

SPIClass spiSD(VSPI);

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
esp_timer_handle_t periodic_timer;

bool isCalibrating = false;
VoiceActivityDetector vad;

void startRecording();
void stopRecording();
String getUniqueFilename();
void writeWavHeader(File file, uint32_t dataSize);
void onTimer(void *arg);

/* ========= INDEX.TXT FUNCTIONS ========= */

int getLastFileIndex()
{
    File indexFile = SD.open("/index.txt", FILE_READ);
    int index = 1;
    if (indexFile)
    {
        index = indexFile.parseInt();
        indexFile.close();
    }
    return index;
}

void saveLastFileIndex(int index)
{
    File indexFile = SD.open("/index.txt", FILE_WRITE);
    if (indexFile)
    {
        indexFile.println(index);
        indexFile.close();
    }
}

void onTimer(void *arg)
{
    int16_t raw = (2048 - adc1_get_raw(ADC1_CHANNEL_4)) * 16;

    hpf_x[0] = (float)raw;
    hpf_y[0] = hpf_b[0] * hpf_x[0] + hpf_b[1] * hpf_x[1] + hpf_b[2] * hpf_x[2] - hpf_a[1] * hpf_y[1] - hpf_a[2] * hpf_y[2];

    hpf_x[2] = hpf_x[1];
    hpf_x[1] = hpf_x[0];
    hpf_y[2] = hpf_y[1];
    hpf_y[1] = hpf_y[0];

    x[0] = hpf_y[0];
    y[0] = b[0] * x[0] + b[1] * x[1] + b[2] * x[2] - a[1] * y[1] - a[2] * y[2];

    x[2] = x[1];
    x[1] = x[0];
    y[2] = y[1];
    y[1] = y[0];

    int16_t filtered = (int16_t)y[0];

    if (isCalibrating)
    {
        vad_collect_calibration_sample(&vad, filtered);
        return;
    }

    if (!recording)
        return;

    portENTER_CRITICAL_ISR(&timerMux);
    buffer[bufferIndex++] = raw;

    if (totalSamplesRecorded < TOTAL_SAMPLES)
        fullBuffer[totalSamplesRecorded] = filtered;

    totalSamplesRecorded++;

    if (bufferIndex >= BUFFER_SIZE)
    {
        bufferReadyToWrite = true;
        bufferIndex = 0;
    }
    portEXIT_CRITICAL_ISR(&timerMux);

    if (totalSamplesRecorded >= TOTAL_SAMPLES)
    {
        esp_timer_stop(periodic_timer);
        recording = false;

        if (audioFile && bufferIndex > 0)
        {
            audioFile.write((uint8_t *)buffer, bufferIndex * sizeof(int16_t));
            bufferIndex = 0;
        }
        stopRecording();
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    if (!SD.begin(SD_CS, spiSD))
    {
        Serial.println("SD Card Failed!");
        while (1)
            ;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = &onTimer,
        .name = "sampling_timer"};
    esp_timer_create(&timer_args, &periodic_timer);

    vad_init(&vad, 0.020f, SAMPLE_RATE / 50, 5);
    vad_reset_calibration(&vad);

    Serial.println("Calibrating...");
    isCalibrating = true;
    esp_timer_start_periodic(periodic_timer, 1000000 / SAMPLE_RATE);

    delay(3000);

    vad_compute_calibration_threshold(&vad);
    esp_timer_stop(periodic_timer);
    isCalibrating = false;

    Serial.printf("Calibration done. Threshold: %.3f\n", vad.threshold);

    Serial.println("Ready to Record");
}

void loop()
{
    bool currentButtonState = digitalRead(BUTTON_PIN);
    if (currentButtonState == LOW &&
        lastButtonState == HIGH &&
        millis() - lastButtonPressTime > DEBOUNCE_TIME)
    {
        lastButtonPressTime = millis();
        if (!recording)
        {
            startRecording();
            totalSamplesRecorded = 0;
            bufferIndex = 0;
            bufferReadyToWrite = false;
            esp_timer_start_periodic(periodic_timer, 1000000 / SAMPLE_RATE);
        }
    }
    lastButtonState = currentButtonState;

    if (bufferReadyToWrite)
    {
        portENTER_CRITICAL(&timerMux);
        bufferReadyToWrite = false;
        portEXIT_CRITICAL(&timerMux);

        if (audioFile)
            audioFile.write((uint8_t *)buffer, BUFFER_SIZE * sizeof(int16_t));
    }
}

void startRecording()
{
    Serial.println("Recording started.");
    filename = getUniqueFilename();
    audioFile = SD.open(filename, FILE_WRITE);

    if (!audioFile)
    {
        Serial.println("Failed to open file!");
        recording = false;
        return;
    }

    recording = true;
    for (int i = 0; i < 44; i++)
        audioFile.write((uint8_t)0);
}

void stopRecording()
{
    Serial.println("Recording stopped.");

    if (audioFile)
    {
        uint32_t dataSize = audioFile.size() - 44;
        audioFile.seek(0);
        writeWavHeader(audioFile, dataSize);
        audioFile.close();
        Serial.println("Saved: " + filename);
    }

    int vad_sample_count = vad_extract(&vad, fullBuffer, TOTAL_SAMPLES,
                                       vad_output, MAX_VAD_OUTPUT);

    Serial.print("VAD Output Samples: ");
    Serial.println(vad_sample_count);

    if (vad_sample_count > 0)
    {
        String vad_filename = filename;
        vad_filename.replace(".wav", "_vad.wav");

        File vadFile = SD.open(vad_filename.c_str(), FILE_WRITE);
        if (vadFile)
        {
            writeWavHeader(vadFile, vad_sample_count * sizeof(int16_t));
            vadFile.write((uint8_t *)vad_output,
                          vad_sample_count * sizeof(int16_t));
            vadFile.close();

            Serial.println("Saved VAD file: " + vad_filename);
            vad_update_threshold_after_speech(&vad);
            Serial.printf("Updated Threshold: %.3f\n", vad.threshold);
        }
    }
    else
    {
        Serial.println("No voiced audio detected.");
    }

    Serial.print("Total samples: ");
    Serial.println(totalSamplesRecorded);
}

/* ========= UPDATED FILENAME FUNCTION ========= */

String getUniqueFilename()
{
    int fileIndex = getLastFileIndex();
    String name = "/Test050126Module_" + String(fileIndex) + ".wav";
    saveLastFileIndex(fileIndex + 1);
    return name;
}

/* ============================================ */

void writeWavHeader(File file, uint32_t dataSize)
{
    uint32_t fileSize = dataSize + 44 - 8;
    uint32_t fmtChunkSize = 16;
    uint16_t audioFormat = 1;
    uint16_t numChannels = 1;
    uint32_t sampleRate = SAMPLE_RATE;
    uint32_t byteRate = sampleRate * numChannels * 2;
    uint16_t blockAlign = numChannels * 2;
    uint16_t bitsPerSample = 16;

    file.write((const uint8_t *)"RIFF", 4);
    file.write((uint8_t *)&fileSize, 4);
    file.write((const uint8_t *)"WAVE", 4);
    file.write((const uint8_t *)"fmt ", 4);
    file.write((uint8_t *)&fmtChunkSize, 4);
    file.write((uint8_t *)&audioFormat, 2);
    file.write((uint8_t *)&numChannels, 2);
    file.write((uint8_t *)&sampleRate, 4);
    file.write((uint8_t *)&byteRate, 4);
    file.write((uint8_t *)&blockAlign, 2);
    file.write((uint8_t *)&bitsPerSample, 2);
    file.write((const uint8_t *)"data", 4);
    file.write((uint8_t *)&dataSize, 4);
    file.flush();
}
