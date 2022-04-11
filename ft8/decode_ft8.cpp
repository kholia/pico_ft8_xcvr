#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include "unpack.h"
#include "ldpc.h"
#include "decode.h"
#include "constants.h"
#include "crc.h"

#include "common/common.h"
#include "common/wave.h"
#include "common/debug.h"
#include "../fft/kiss_fftr.h"

#define LOG_LEVEL LOG_INFO

#include "decode_ft8.h"
#include "../util/rx_ft8.h"

#ifndef PC
#include "pico/multicore.h"
#endif

//#include "hardware/timer.h" //won't be needed once delay is replaced with interrupt

const float fft_norm = 2.0f / nfft;

// AA1GD - added array of structures to store info in decoded messages 8/22/2021
// Setup fft freq output power, which will be accessible by both cores
uint8_t mag_power[num_blocks * kFreq_osr * kTime_osr * num_bins] = {0};
waterfall_t power = {
  .num_blocks = num_blocks,
  .num_bins = num_bins,
  .time_osr = kTime_osr,
  .freq_osr = kFreq_osr,
  .mag = mag_power,
  .block_stride = (kTime_osr * kFreq_osr * num_bins),
  .protocol = PROTO_FT8
};

volatile int offset = 0;

kiss_fftr_cfg fft_cfg;

float window[nfft]; // I wonder if static will work here - AA1GD

void usage()
{
    fprintf(stderr, "Decode a 15-second (or slighly shorter) WAV file.\n");
}

static float hann_i(int i, int N)
{
    float x = sinf((float)M_PI * i / N);
    return x * x;
}

void make_window(void) {
  const int len_window = (int) (1.8f * block_size); // hand-picked and optimized
  for (int i = 0; i < nfft; ++i)
  {
    window[i] = (i < len_window) ? hann_i(i, len_window) : 0;
  }
}

static float max2(float a, float b)
{
  return (a >= b) ? a : b;
}

// Compute FFT magnitudes (log power) for each timeslot in the signal
void inc_extract_power(int16_t signal[])
{
  // Loop over two possible time offsets (0 and block_size/2)
  for (int time_sub = 0; time_sub < power.time_osr; ++time_sub)
  {
    kiss_fft_scalar timedata[nfft];
    kiss_fft_cpx freqdata[nfft / 2 + 1];
    float mag_db[nfft / 2 + 1];

    // Extract windowed signal block
    for (int pos = 0; pos < nfft; ++pos)
    {
      // Maybe I just need to convert back to a float here? added a divide by 32768.0 Sept 19 2021
      // Couldn't get kiss_fft to work with int16_t. Dividing by 2048 as ADC gives 12 bit readings Oct. 16 2021
      // Changing window changed... something. Trying without window (just 1)
      // timedata[pos] = window[pos] * signal[(time_sub * subblock_size) + pos] / 2048.0f;
      timedata[pos] = signal[(time_sub * subblock_size) + pos] / 2048.0f;
    }
    kiss_fftr(fft_cfg, timedata, freqdata);

    // Compute log magnitude in decibels
    for (int idx_bin = 0; idx_bin < nfft / 2 + 1; ++idx_bin)
    {
      float mag2 = (freqdata[idx_bin].i * freqdata[idx_bin].i) + (freqdata[idx_bin].r * freqdata[idx_bin].r);
      mag_db[idx_bin] = 10.0f * log10f(1E-12f + mag2 * fft_norm * fft_norm);
    }

    // Printf("Compute log magnitude\n");
    // Loop over two possible frequency bin offsets (for averaging)
    for (int freq_sub = 0; freq_sub < power.freq_osr; ++freq_sub)
    {
      for (int pos = 0; pos < power.num_bins; ++pos)
      {

        float db = mag_db[pos * power.freq_osr + freq_sub];
        // Scale decibels to unsigned 8-bit range and clamp the value
        // Range 0-240 covers -120..0 dB in 0.5 dB steps
        int scaled = (int)(2 * db + 240);
        power.mag[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
        power.mag[offset] = scaled;
        offset++;
      }
    }
  }
  return;
}

// The signal needs to be 3/2 (1.5) times as long as nfft
// Oct. 2, 2021 got time_osr = 1 incremental decoding working
void inc_collect_power() {
  size_t fft_work_size;
  kiss_fftr_alloc(nfft, 0, 0, &fft_work_size);

  printf("Sample rate %d Hz, %d blocks, %d bins\n", sample_rate_, num_blocks, num_bins);
  printf("This is size of array mag_power in bytes: %d\n", num_blocks * kFreq_osr * kTime_osr * num_bins);

  void *fft_work = malloc(fft_work_size);
  fft_cfg = kiss_fftr_alloc(nfft, 0, fft_work, &fft_work_size);
  printf("starting incremental collection\n");

  // PASS IDX_BLOCK THROUGH THE FIFO - this may help with the offset
  for (uint idx_block = 0; idx_block < num_blocks; idx_block++) {
#ifndef PC
    collect_adc();
    multicore_fifo_push_blocking(idx_block);
#endif
  }

  // May want to wait or get a message back from core 1 before memory is freed
#ifndef PC
  busy_wait_ms(160); // Waits a bit for the FFT to finish. May want to replace with a confirmation back from core 1.
#endif
  free(fft_work);
  printf("done collecting power\n");
  printf("resetting offset and max mag\n");
  offset = 0;
  return;
}

// This code is borrowed from ft8_lib's decode_ft8.c file
int decode_ft8(message_info message_list[])
{
  // Find top candidates by Costas sync score and localize them in time and frequency
  candidate_t candidate_list[kMax_candidates];
  int num_candidates = ft8_find_sync(&power, kMax_candidates, candidate_list, kMin_score);

  // Hash table for decoded messages (to check for duplicates)
  int num_decoded = 0;
  message_t decoded[kMax_decoded_messages];
  message_t *decoded_hashtable[kMax_decoded_messages];

  // Initialize hash table pointers
  for (int i = 0; i < kMax_decoded_messages; ++i)
  {
    decoded_hashtable[i] = NULL;
  }

  // Go over candidates and attempt to decode messages
  for (int idx = 0; idx < num_candidates; ++idx)
  {
    // AA1GD added to try correctly stop program when decoded>kMax_decoded_messages
    // printf("num decoded %d \n",num_decoded);
    if (num_decoded >= kMax_decoded_messages) {
      printf("decoded more than kMax_decoded_messages\n");
      printf("Decoded %d messages and force ended\n", num_decoded);
      return (num_decoded);
    }

    const candidate_t *cand = &candidate_list[idx];
    if (cand->score < kMin_score)
      continue;

    float freq_hz = (cand->freq_offset + (float)cand->freq_sub / kFreq_osr) * kFSK_dev;
    float time_sec = (cand->time_offset + (float)cand->time_sub / kTime_osr) / kFSK_dev;

    message_t message;
    decode_status_t status;

    if (!ft8_decode(&power, cand, &message, kLDPC_iterations, &status))
    {
      if (status.ldpc_errors > 0)
      {
        // printf("LDPC decode: %d errors\n", status.ldpc_errors);
      }
      else if (status.crc_calculated != status.crc_extracted)
      {
        // printf("CRC mismatch!\n");
      }
      else if (status.unpack_status != 0)
      {
        // printf("Error while unpacking!\n");
      }
      continue;
    }

    // printf("Checking hash table for %4.1fs / %4.1fHz [%d]...\n", time_sec, freq_hz, cand->score);
    int idx_hash = message.hash % kMax_decoded_messages;
    bool found_empty_slot = false;
    bool found_duplicate = false;
    do
    {
      if (decoded_hashtable[idx_hash] == NULL)
      {
        // printf("Found an empty slot\n");
        found_empty_slot = true;
      }
      else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text)))
      {
        // printf("Found a duplicate [%s]\n", message.text);
        found_duplicate = true;
      }
      else
      {
        // printf("Hash table clash!\n");
        // Move on to check the next entry in hash table
        idx_hash = (idx_hash + 1) % kMax_decoded_messages;
      }
    } while (!found_empty_slot && !found_duplicate);

    if (found_empty_slot)
    {
      // Fill the empty hashtable slot
      memcpy(&decoded[idx_hash], &message, sizeof(message));
      decoded_hashtable[idx_hash] = &decoded[idx_hash];

      int snr = cand->score;
      printf("%x   %3d  %+3.1f %4d ~  %s\n", num_decoded, snr, time_sec, (int) freq_hz, message.text);

      // AA1GD - add message info to the global variable message_list
      message_list[num_decoded].self_rx_snr = snr;
      message_list[num_decoded].af_frequency = (uint16_t) freq_hz;
      message_list[num_decoded].time_offset = time_sec;
      strcpy(message_list[num_decoded].full_text, message.text);

      ++num_decoded;
    }
  }
  printf("Decoded %d messages\n", num_decoded);

  return num_decoded;
}

static float hamming_i(int i, int N)
{
    const float a0 = (float)25 / 46;
    const float a1 = 1 - a0;

    float x1 = cosf(2 * (float)M_PI * i / N);
    return a0 - a1 * x1;
}

static float blackman_i(int i, int N)
{
    const float alpha = 0.16f; // or 2860/18608
    const float a0 = (1 - alpha) / 2;
    const float a1 = 1.0f / 2;
    const float a2 = alpha / 2;

    float x1 = cosf(2 * (float)M_PI * i / N);
    float x2 = 2 * x1 * x1 - 1; // Use double angle formula

    return a0 - a1 * x1 + a2 * x2;
}

void waterfall_init(waterfall_t* me, int max_blocks, int num_bins, int time_osr, int freq_osr)
{
    size_t mag_size = max_blocks * time_osr * freq_osr * num_bins * sizeof(me->mag[0]);
    me->max_blocks = max_blocks;
    me->num_blocks = 0;
    me->num_bins = num_bins;
    me->time_osr = time_osr;
    me->freq_osr = freq_osr;
    me->block_stride = (time_osr * freq_osr * num_bins);
    me->mag = (uint8_t  *)malloc(mag_size);
    LOG(LOG_DEBUG, "Waterfall size = %zu\n", mag_size);
}

void waterfall_free(waterfall_t* me)
{
    free(me->mag);
}

/// Configuration options for FT4/FT8 monitor
typedef struct
{
    float f_min;             ///< Lower frequency bound for analysis
    float f_max;             ///< Upper frequency bound for analysis
    int sample_rate;         ///< Sample rate in Hertz
    int time_osr;            ///< Number of time subdivisions
    int freq_osr;            ///< Number of frequency subdivisions
    ftx_protocol_t protocol; ///< Protocol: FT4 or FT8
} monitor_config_t;

/// FT4/FT8 monitor object that manages DSP processing of incoming audio data
/// and prepares a waterfall object
typedef struct
{
    float symbol_period; ///< FT4/FT8 symbol period in seconds
    int block_size;      ///< Number of samples per symbol (block)
    int subblock_size;   ///< Analysis shift size (number of samples)
    int nfft;            ///< FFT size
    float fft_norm;      ///< FFT normalization factor
    float* window;       ///< Window function for STFT analysis (nfft samples)
    float* last_frame;   ///< Current STFT analysis frame (nfft samples)
    waterfall_t wf;      ///< Waterfall object
    float max_mag;       ///< Maximum detected magnitude (debug stats)

    // KISS FFT housekeeping variables
    void* fft_work;        ///< Work area required by Kiss FFT
    kiss_fftr_cfg fft_cfg; ///< Kiss FFT housekeeping object
} monitor_t;

void monitor_init(monitor_t* me, const monitor_config_t* cfg)
{
    float slot_time = (cfg->protocol == PROTO_FT4) ? FT4_SLOT_TIME : FT8_SLOT_TIME;
    float symbol_period = (cfg->protocol == PROTO_FT4) ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    // Compute DSP parameters that depend on the sample rate
    me->block_size = (int)(cfg->sample_rate * symbol_period); // samples corresponding to one FSK symbol
    me->subblock_size = me->block_size / cfg->time_osr;
    me->nfft = me->block_size * cfg->freq_osr;
    me->fft_norm = 2.0f / me->nfft;
    // const int len_window = 1.8f * me->block_size; // hand-picked and optimized

    me->window = (float *)malloc(me->nfft * sizeof(me->window[0]));
    for (int i = 0; i < me->nfft; ++i)
    {
        // window[i] = 1;
        me->window[i] = hann_i(i, me->nfft);
        // me->window[i] = blackman_i(i, me->nfft);
        // me->window[i] = hamming_i(i, me->nfft);
        // me->window[i] = (i < len_window) ? hann_i(i, len_window) : 0;
    }
    me->last_frame = (float *)malloc(me->nfft * sizeof(me->last_frame[0]));

    size_t fft_work_size;
    kiss_fftr_alloc(me->nfft, 0, 0, &fft_work_size);

    LOG(LOG_INFO, "Block size = %d\n", me->block_size);
    LOG(LOG_INFO, "Subblock size = %d\n", me->subblock_size);
    LOG(LOG_INFO, "N_FFT = %d\n", me->nfft);
    LOG(LOG_DEBUG, "FFT work area = %zu\n", fft_work_size);

    me->fft_work = malloc(fft_work_size);
    me->fft_cfg = kiss_fftr_alloc(me->nfft, 0, me->fft_work, &fft_work_size);

    const int max_blocks = (int)(slot_time / symbol_period);
    const int num_bins = (int)(cfg->sample_rate * symbol_period / 2);
    waterfall_init(&me->wf, max_blocks, num_bins, cfg->time_osr, cfg->freq_osr);
    me->wf.protocol = cfg->protocol;
    me->symbol_period = symbol_period;

    me->max_mag = -120.0f;
}

void monitor_free(monitor_t* me)
{
    waterfall_free(&me->wf);
    free(me->fft_work);
    free(me->last_frame);
    free(me->window);
}

// Compute FFT magnitudes (log wf) for a frame in the signal and update waterfall data
void monitor_process(monitor_t* me, const float* frame)
{
    // Check if we can still store more waterfall data
    if (me->wf.num_blocks >= me->wf.max_blocks)
        return;

    int offset = me->wf.num_blocks * me->wf.block_stride;
    int frame_pos = 0;

    // Loop over block subdivisions
    for (int time_sub = 0; time_sub < me->wf.time_osr; ++time_sub)
    {
        kiss_fft_scalar timedata[me->nfft];
        kiss_fft_cpx freqdata[me->nfft / 2 + 1];

        // Shift the new data into analysis frame
        for (int pos = 0; pos < me->nfft - me->subblock_size; ++pos)
        {
            me->last_frame[pos] = me->last_frame[pos + me->subblock_size];
        }
        for (int pos = me->nfft - me->subblock_size; pos < me->nfft; ++pos)
        {
            me->last_frame[pos] = frame[frame_pos];
            ++frame_pos;
        }

        // Compute windowed analysis frame
        for (int pos = 0; pos < me->nfft; ++pos)
        {
            timedata[pos] = me->fft_norm * me->window[pos] * me->last_frame[pos];
        }

        kiss_fftr(me->fft_cfg, timedata, freqdata);

        // Loop over two possible frequency bin offsets (for averaging)
        for (int freq_sub = 0; freq_sub < me->wf.freq_osr; ++freq_sub)
        {
            for (int bin = 0; bin < me->wf.num_bins; ++bin)
            {
                int src_bin = (bin * me->wf.freq_osr) + freq_sub;
                float mag2 = (freqdata[src_bin].i * freqdata[src_bin].i) + (freqdata[src_bin].r * freqdata[src_bin].r);
                float db = 10.0f * log10f(1E-12f + mag2);
                // Scale decibels to unsigned 8-bit range and clamp the value
                // Range 0-240 covers -120..0 dB in 0.5 dB steps
                int scaled = (int)(2 * db + 240);

                me->wf.mag[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                ++offset;

                if (db > me->max_mag)
                    me->max_mag = db;
            }
        }
    }

    ++me->wf.num_blocks;
}

void monitor_reset(monitor_t* me)
{
    me->wf.num_blocks = 0;
    me->max_mag = 0;
}

#ifdef PC
int main(int argc, char** argv)
{
    // Accepted arguments
    const char* wav_path = NULL;
    bool is_ft8 = true;

    // Parse arguments one by one
    int arg_idx = 1;
    while (arg_idx < argc)
    {
        // Check if the current argument is an option (-xxx)
        if (argv[arg_idx][0] == '-')
        {
            // Check agaist valid options
            if (0 == strcmp(argv[arg_idx], "-ft4"))
            {
                is_ft8 = false;
            }
            else
            {
                usage();
                return -1;
            }
        }
        else
        {
            if (wav_path == NULL)
            {
                wav_path = argv[arg_idx];
            }
            else
            {
                usage();
                return -1;
            }
        }
        ++arg_idx;
    }
    // Check if all mandatory arguments have been received
    if (wav_path == NULL)
    {
        usage();
        return -1;
    }

    int sample_rate = 12000;
    int num_samples = 15 * sample_rate;
    float signal[num_samples];

    int rc = load_wav(signal, &num_samples, &sample_rate, wav_path);
    if (rc < 0)
    {
        return -1;
    }

    LOG(LOG_INFO, "Sample rate %d Hz, %d samples, %.3f seconds\n", sample_rate, num_samples, (double)num_samples / sample_rate);

    // Compute FFT over the whole signal and store it
    monitor_t mon;
    monitor_config_t mon_cfg = {
        .f_min = 100,
        .f_max = 3000,
        .sample_rate = sample_rate,
        .time_osr = kTime_osr,
        .freq_osr = kFreq_osr,
        .protocol = is_ft8 ? PROTO_FT8 : PROTO_FT4
    };
    monitor_init(&mon, &mon_cfg);
    LOG(LOG_DEBUG, "Waterfall allocated %d symbols\n", mon.wf.max_blocks);
    for (int frame_pos = 0; frame_pos + mon.block_size <= num_samples; frame_pos += mon.block_size)
    {
        // Process the waveform data frame by frame - you could have a live loop here with data from an audio device
        monitor_process(&mon, signal + frame_pos);
    }
    LOG(LOG_DEBUG, "Waterfall accumulated %d symbols\n", mon.wf.num_blocks);
    LOG(LOG_INFO, "Max magnitude: %.1f dB\n", mon.max_mag);

    // Find top candidates by Costas sync score and localize them in time and frequency
    candidate_t candidate_list[kMax_candidates];
    int num_candidates = ft8_find_sync(&mon.wf, kMax_candidates, candidate_list, kMin_score);

    // Hash table for decoded messages (to check for duplicates)
    int num_decoded = 0;
    message_t decoded[kMax_decoded_messages];
    message_t* decoded_hashtable[kMax_decoded_messages];

    // Initialize hash table pointers
    for (int i = 0; i < kMax_decoded_messages; ++i)
    {
        decoded_hashtable[i] = NULL;
    }

    // Go over candidates and attempt to decode messages
    for (int idx = 0; idx < num_candidates; ++idx)
    {
        const candidate_t* cand = &candidate_list[idx];
        if (cand->score < kMin_score)
            continue;

        float freq_hz = (cand->freq_offset + (float)cand->freq_sub / mon.wf.freq_osr) / mon.symbol_period;
        float time_sec = (cand->time_offset + (float)cand->time_sub / mon.wf.time_osr) * mon.symbol_period;

        message_t message;
        decode_status_t status;
        if (!ft8_decode(&mon.wf, cand, &message, kLDPC_iterations, &status))
        {
            // printf("000000 %3d %+4.2f %4.0f ~  ---\n", cand->score, time_sec, freq_hz);
            if (status.ldpc_errors > 0)
            {
                LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
            }
            else if (status.crc_calculated != status.crc_extracted)
            {
                LOG(LOG_DEBUG, "CRC mismatch!\n");
            }
            else if (status.unpack_status != 0)
            {
                LOG(LOG_DEBUG, "Error while unpacking!\n");
            }
            continue;
        }

        LOG(LOG_DEBUG, "Checking hash table for %4.1fs / %4.1fHz [%d]...\n", time_sec, freq_hz, cand->score);
        int idx_hash = message.hash % kMax_decoded_messages;
        bool found_empty_slot = false;
        bool found_duplicate = false;
        do
        {
            if (decoded_hashtable[idx_hash] == NULL)
            {
                LOG(LOG_DEBUG, "Found an empty slot\n");
                found_empty_slot = true;
            }
            else if ((decoded_hashtable[idx_hash]->hash == message.hash) && (0 == strcmp(decoded_hashtable[idx_hash]->text, message.text)))
            {
                LOG(LOG_DEBUG, "Found a duplicate [%s]\n", message.text);
                found_duplicate = true;
            }
            else
            {
                LOG(LOG_DEBUG, "Hash table clash!\n");
                // Move on to check the next entry in hash table
                idx_hash = (idx_hash + 1) % kMax_decoded_messages;
            }
        } while (!found_empty_slot && !found_duplicate);

        if (found_empty_slot)
        {
            // Fill the empty hashtable slot
            memcpy(&decoded[idx_hash], &message, sizeof(message));
            decoded_hashtable[idx_hash] = &decoded[idx_hash];
            ++num_decoded;

            // Fake WSJT-X-like output for now
            int snr = 0; // TODO: compute SNR
            printf("000000 %3d %+4.2f %4.0f ~  %s\n", cand->score, time_sec, freq_hz, message.text);
        }
    }
    LOG(LOG_INFO, "Decoded %d messages\n", num_decoded);

    monitor_free(&mon);

    return 0;
}
#endif
