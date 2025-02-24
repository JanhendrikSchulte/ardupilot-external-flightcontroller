/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper
 */

#include <AP_Math/AP_Math.h>
#include "AP_HAL.h"
#include "DSP.h"
#ifndef HAL_NO_UARTDRIVER
#include <GCS_MAVLink/GCS.h>
#endif
// TODO-TBU
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC
#include <assert.h>
#endif

#if HAL_WITH_DSP

using namespace AP_HAL;

extern const AP_HAL::HAL &hal;

#define SQRT_2_3 0.816496580927726f
#define SQRT_6   2.449489742783178f

DSP::FFTWindowState::FFTWindowState(uint16_t window_size, uint16_t sample_rate, uint8_t sliding_window_size) :
    _bin_resolution((float)sample_rate / (float)window_size),
    _bin_count(window_size / 2),
    _num_stored_freqs(window_size / 2 + 1),
    _window_size(window_size),
    _sliding_window_size(sliding_window_size),
    _current_slice(0)
{
    // includes DC ad Nyquist components and needs to be large enough for intermediate steps
    _freq_bins = (float*)hal.util->malloc_type(sizeof(float) * _window_size, DSP_MEM_REGION);
    _derivative_freq_bins = (float*)hal.util->malloc_type(sizeof(float) * _num_stored_freqs, DSP_MEM_REGION);
    _hanning_window = (float*)hal.util->malloc_type(sizeof(float) * _window_size, DSP_MEM_REGION);
    // allocate workspace, including Nyquist component
    _rfft_data = (float*)hal.util->malloc_type(sizeof(float) * (_window_size + 2), DSP_MEM_REGION);
    // sliding window of frequency bin frames
    if (_sliding_window_size > 0) {
        _sliding_window = (float*)hal.util->malloc_type(sizeof(float) * _num_stored_freqs * _sliding_window_size, DSP_MEM_REGION);
        _avg_freq_bins = (float*)hal.util->malloc_type(sizeof(float) * _num_stored_freqs, DSP_MEM_REGION);
        // we can still fallback to non-averaging if there is not enough memory
        if (_avg_freq_bins == nullptr) {
            hal.util->free_type(_sliding_window, sizeof(float) * _num_stored_freqs * _sliding_window_size, DSP_MEM_REGION);
            _sliding_window = nullptr;
        }
    }

    if (_freq_bins == nullptr || _hanning_window == nullptr || _rfft_data == nullptr || _derivative_freq_bins == nullptr) {
        free_data_structures();
        return;
    }

    // create the Hanning window
    // https://holometer.fnal.gov/GH_FFT.pdf - equation 19
    for (uint16_t i = 0; i < window_size; i++) {
        _hanning_window[i] = (0.5f - 0.5f * cosf(2.0f * M_PI * i / ((float)window_size - 1)));
        _window_scale += _hanning_window[i];
    }
    // Calculate the inverse of the Effective Noise Bandwidth - equation 24
    _window_scale = 2.0f / sq(_window_scale);
}

DSP::FFTWindowState::~FFTWindowState()
{
    free_data_structures();
}

void DSP::FFTWindowState::free_data_structures()
{
    hal.util->free_type(_freq_bins, sizeof(float) * _window_size * _sliding_window_size, DSP_MEM_REGION);
    _freq_bins = nullptr;
    hal.util->free_type(_derivative_freq_bins, sizeof(float) * _num_stored_freqs, DSP_MEM_REGION);
    _derivative_freq_bins = nullptr;
    hal.util->free_type(_hanning_window, sizeof(float) * (_window_size), DSP_MEM_REGION);
    _hanning_window = nullptr;
    hal.util->free_type(_rfft_data, sizeof(float) * (_window_size + 2), DSP_MEM_REGION);
    _rfft_data = nullptr;
    hal.util->free_type(_avg_freq_bins, sizeof(float) * _num_stored_freqs, DSP_MEM_REGION);
    _avg_freq_bins = nullptr;
    hal.util->free_type(_sliding_window, sizeof(float) * _num_stored_freqs * _sliding_window_size, DSP_MEM_REGION);
    _sliding_window = nullptr;
}

// step 3: find the magnitudes of the complex data
void DSP::step_cmplx_mag(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float noise_att_cutoff)
{
    // fft->_freq_bins is populated with the complex magnitude values of the fft data
    // find the maximum power in the range we are interested in
    // in order to see a peak in the last bin we need to allow all the way up to the nyquist
    const uint16_t smoothwidth = 1;
    float* freq_data = fft->_freq_bins;

    if (fft->_sliding_window != nullptr) {
        update_average_from_sliding_window(fft);
        freq_data = fft->_avg_freq_bins;
    } else {
        // scale the power to account for the input window
        vector_scale_float(fft->_freq_bins, fft->_window_scale, fft->_freq_bins, fft->_bin_count);
    }

    uint16_t bin_range = (MIN(end_bin + ((smoothwidth + 1) >> 1) + 2, fft->_bin_count) - start_bin) + 1;
    // find the three highest peaks using a zero crossing algorithm
    uint16_t peaks[MAX_TRACKED_PEAKS] {};
    memset(fft->_peak_data, 0, sizeof(fft->_peak_data));
    uint16_t numpeaks = find_peaks(&freq_data[start_bin], bin_range, fft->_derivative_freq_bins, peaks, MAX_TRACKED_PEAKS, 0.0f, -1.0f, smoothwidth, 2);
    //hal.console->printf("found %d peaks\n", numpeaks);

    for (uint16_t i = 0; i < MAX_TRACKED_PEAKS; i++) {
        fft->_peak_data[i]._bin = peaks[i] + start_bin;
    }

    uint16_t top = 0, bottom = 0;
    fft->_peak_data[CENTER]._noise_width_hz = find_noise_width(freq_data, start_bin, end_bin, fft->_peak_data[CENTER]._bin, noise_att_cutoff, fft->_bin_resolution, top, bottom);
    if (numpeaks > 1) {
        fft->_peak_data[LOWER_SHOULDER]._noise_width_hz = find_noise_width(freq_data, start_bin, end_bin, fft->_peak_data[LOWER_SHOULDER]._bin, noise_att_cutoff, fft->_bin_resolution, top, bottom);
    }
    if (numpeaks > 2) {
        fft->_peak_data[UPPER_SHOULDER]._noise_width_hz = find_noise_width(freq_data, start_bin, end_bin, fft->_peak_data[UPPER_SHOULDER]._bin, noise_att_cutoff, fft->_bin_resolution, top, bottom);
    }

    // average the FFT data
    if (fft->_averaging) {
        vector_add_float(fft->_avg_freq_bins, fft->_freq_bins, fft->_avg_freq_bins, fft->_bin_count);
        fft->_averaging_samples++;
    }
}

// calculate the noise width of a peak based on the input parameters
// freq_bins can be scaled or unscaled for power
float DSP::find_noise_width(float* freq_bins, uint16_t start_bin, uint16_t end_bin, uint16_t max_energy_bin, float cutoff, float bin_resolution, uint16_t& peak_top, uint16_t& peak_bottom) const
{
    // max_energy_bin is guaranteed to be between start_bin and end_bin
    peak_top = end_bin;
    peak_bottom = start_bin;

    // calculate the width of the peak
    float noise_width_hz = 1;

    // -attenuation/2 dB point above the center bin
    if (max_energy_bin < end_bin) {
        for (uint16_t b = max_energy_bin + 1; b <= end_bin; b++) {
            if (freq_bins[b] < freq_bins[max_energy_bin] * cutoff) {
                // we assume that the 3dB point is in the middle of the final shoulder bin
                noise_width_hz += (b - max_energy_bin - 0.5f);
                peak_top = b;
                break;
            }
        }
    }
    // -attenuation/2 dB point below the center bin
    if (max_energy_bin > start_bin) {
        for (uint16_t b = max_energy_bin - 1; b >= start_bin; b--) {
            if (freq_bins[b] < freq_bins[max_energy_bin] * cutoff) {
                // we assume that the 3dB point is in the middle of the final shoulder bin
                noise_width_hz += (max_energy_bin - b - 0.5f);
                peak_bottom = b;
                break;
            }
        }
    }
    noise_width_hz *= bin_resolution;

    return noise_width_hz;
}

// step 4: find the bin with the highest energy and interpolate the required frequency
uint16_t DSP::step_calc_frequencies(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin)
{
    fft->_peak_data[CENTER]._freq_hz = calc_frequency(fft, start_bin, fft->_peak_data[CENTER]._bin, end_bin);
    fft->_peak_data[UPPER_SHOULDER]._freq_hz = calc_frequency(fft, start_bin, fft->_peak_data[UPPER_SHOULDER]._bin, end_bin);
    fft->_peak_data[LOWER_SHOULDER]._freq_hz = calc_frequency(fft, start_bin, fft->_peak_data[LOWER_SHOULDER]._bin, end_bin);

    return fft->_peak_data[CENTER]._bin;
}

void DSP::update_average_from_sliding_window(FFTWindowState* fft)
{
// TODO-TBU
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC
#define ASSERT_MAX(v) assert((v)<(fft->_num_stored_freqs * fft->_sliding_window_size))
#else
#define ASSERT_MAX(v)
#endif

    // copy and scale the new slice
    const uint16_t slice_index = fft->_current_slice * fft->_num_stored_freqs;
    ASSERT_MAX(slice_index);
    float* slice = &fft->_sliding_window[slice_index];

    const uint16_t old_slice_index = ((fft->_current_slice + 1) % fft->_sliding_window_size) * fft->_num_stored_freqs;
    ASSERT_MAX(old_slice_index);
    float* old_slice = &fft->_sliding_window[old_slice_index];

    const float inv_ssize = 1.0f / fft->_sliding_window_size;

    for (uint16_t i = 0; i < fft->_bin_count; i++) {
        slice[i] = fft->_freq_bins[i] * fft->_window_scale * inv_ssize;
        fft->_avg_freq_bins[i] = fft->_avg_freq_bins[i] + slice[i] - old_slice[i];
    }

    // advance the current slice
    fft->_current_slice = (fft->_current_slice + 1) % fft->_sliding_window_size;
}

// calculate a single frequency
uint16_t DSP::calc_frequency(FFTWindowState* fft, uint16_t start_bin, uint16_t peak_bin, uint16_t end_bin)
{
    if (peak_bin == 0 || is_zero(fft->get_freq_bin(peak_bin))) {
        return start_bin * fft->_bin_resolution;
    }

    peak_bin = constrain_int16(peak_bin, start_bin, end_bin);

    // It turns out that Jain is pretty good and works with only magnitudes, but Candan is significantly better
    // if you have access to the complex values and Quinn is a little better still. Quinn is computationally
    // more expensive, but compared to the overall FFT cost seems worth it.
    if (fft->_sliding_window != nullptr) {
        return (peak_bin + calculate_jains_estimator(fft, fft->_avg_freq_bins, peak_bin)) * fft->_bin_resolution;
    } else {
        return (peak_bin + calculate_quinns_second_estimator(fft, fft->_rfft_data, peak_bin)) * fft->_bin_resolution;
    }
}

// Interpolate center frequency using https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
float DSP::calculate_quinns_second_estimator(const FFTWindowState* fft, const float* complex_fft, uint16_t k_max) const
{
    if (k_max <= 1 || k_max >= fft->_bin_count) {
        return 0.0f;
    }

    const uint16_t k_m1 = (k_max - 1) * 2;
    const uint16_t k_p1 = (k_max + 1) * 2;
    const uint16_t k = k_max * 2;

    const float divider = complex_fft[k] * complex_fft[k] + complex_fft[k+1] * complex_fft[k+1];
    const float ap = (complex_fft[k_p1] * complex_fft[k] + complex_fft[k_p1 + 1] * complex_fft[k+1]) / divider;
    const float am = (complex_fft[k_m1] * complex_fft[k] + complex_fft[k_m1 + 1] * complex_fft[k + 1]) / divider;

    // sanity check
    if (fabsf(1.0f - ap) < 0.01f || fabsf(1.0f - am) < 0.01f) {
        return 0.0f;
    }

    const float dp = -ap / (1.0f - ap);
    const float dm = am / (1.0f - am);

    float d = (dp + dm) * 0.5f + tau(dp * dp) - tau(dm * dm);

    // -0.5 < d < 0.5 which is the fraction of the sample spacing about the center element
    return constrain_float(d, -0.5f, 0.5f);
}

static const float TAU_FACTOR = SQRT_6 / 24.0f;

// Helper function used for Quinn's frequency estimation
float DSP::tau(const float x) const
{
    float p1 = logf(3.0f * sq(x) + 6.0f * x + 1.0f);
    float part1 = x + 1.0f - SQRT_2_3;
    float part2 = x + 1.0f + SQRT_2_3;
    float p2 = logf(part1 / part2);
    return (0.25f * p1 - TAU_FACTOR * p2);
}

// from https://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/
// Works on magnitudes only, which is useful when using averaged data
float DSP::calculate_jains_estimator(const FFTWindowState* fft, const float* real_fft, uint16_t k_max)
{
    if (k_max <= 1 || k_max >= fft->_bin_count) {
        return 0.0f;
    }

    float y1 = real_fft[k_max-1];
    float y2 = real_fft[k_max];
    float y3 = real_fft[k_max+1];

    if (is_zero(y2) || is_zero(y1)) {
        return 0.0f;
    }

    float d = 0.0f;

    if (y1 > y3) {
        float a = y2 / y1;
        d = a / (1 + a) - 1;
    } else {
        float a = y3 / y2;
        d = a  /  (1 + a);
    }
    return constrain_float(d, -0.5f, 0.5f);
}

// initialize averaging FFT windows as they are calculated
bool DSP::fft_init_average(FFTWindowState* fft)
{
    if (fft->_avg_freq_bins == nullptr) {
        fft->_avg_freq_bins = (float*)hal.util->malloc_type(sizeof(float) * fft->_num_stored_freqs, DSP_MEM_REGION);
        if (fft->_avg_freq_bins == nullptr) {
            return false;
        }
    }

    return true;
}

// start averaging FFT windows as they are calculated
bool DSP::fft_start_average(FFTWindowState* fft)
{
    if (fft->_averaging) {
        return false;
    }

    if (!fft_init_average(fft)) {
        return false;
    }

    fft->_averaging_samples = 0;
    fft->_averaging = true;
    return true;
}

// start averaging FFT windows as they are calculated
uint16_t DSP::fft_stop_average(FFTWindowState* fft, uint16_t start_bin, uint16_t end_bin, float* freqs)
{
    // ensure the window has been allocated even if we do nothing else
    if (!fft_init_average(fft)) {
        return 0;
    }

    if (!fft->_averaging) {
        return 0;
    }

    fft->_averaging = false;

    // scale by the number of samples
    vector_scale_float(fft->_avg_freq_bins, fft->_averaging_samples, fft->_avg_freq_bins, fft->_bin_count);

    const uint16_t smoothwidth = 1;
    uint16_t bin_range = (MIN(end_bin + ((smoothwidth + 1) >> 1) + 2, fft->_bin_count) - start_bin) + 1;

    // find the three highest peaks using a zero crossing algorithm
    // allocate the scratch space locally as we are in a different thread to the regular FFT
    float* scratch_space = (float*)hal.util->malloc_type(sizeof(float) * fft->_num_stored_freqs, DSP_MEM_REGION);
    if (scratch_space == nullptr) {
        return false;
    }
    uint16_t peaks[MAX_TRACKED_PEAKS] {};
    uint16_t numpeaks = find_peaks(&fft->_avg_freq_bins[start_bin], bin_range,
        scratch_space, peaks, MAX_TRACKED_PEAKS, 0.0f, -1.0f, smoothwidth, 2);
    hal.util->free_type(scratch_space, sizeof(float) * fft->_num_stored_freqs, DSP_MEM_REGION);

    numpeaks = MIN(numpeaks, uint16_t(MAX_TRACKED_PEAKS));

    // now try and find the lowest harmonic
    for (uint16_t i = 0; i < numpeaks; i++) {
        const uint16_t bin = peaks[i] + start_bin;
        float d = calculate_jains_estimator(fft, fft->_avg_freq_bins, bin);
        freqs[i] = (bin + d) * fft->_bin_resolution;
    }

    fft->_averaging_samples = 0;
    return numpeaks;
}

// find all the peaks in the fft window using https://terpconnect.umd.edu/~toh/spectrum/PeakFindingandMeasurement.htm
// in general peakgrup > 2 is only good for very broad noisy peaks, <= 2 better for spikey peaks, although 1 will miss
// a true spike 50% of the time
uint16_t DSP::find_peaks(const float* input, uint16_t length, float* d, uint16_t* peaks, uint16_t peaklen,
    float slopeThreshold, float ampThreshold, uint16_t smoothwidth, uint16_t peakgroup) const
{
    if (smoothwidth > 1) {
        derivative(input, d, length);
        fastsmooth(d, length, smoothwidth);
    } else {
        derivative(input, d, length);
    }

    uint16_t n = ((peakgroup + 1) >> 1) + 1;
    uint16_t halfw = (smoothwidth + 1) >> 1;
    uint16_t numpeaks = 0;
    uint16_t peakX = 0;
    float peakY = 0;
    uint16_t pindex;
    uint16_t xx[peakgroup];
    float yy[peakgroup];
    memset(xx, 0, peakgroup * sizeof(uint16_t));
    memset(yy, 0, peakgroup * sizeof(float));

    for (uint16_t j = (halfw << 1) - 2; j < length - smoothwidth - 1; j++) {
        if (d[j] >= 0 && d[j + 1] <= 0 && !is_equal(d[j], d[j + 1])) { // detect zero crossing
            if ((d[j] - d[j + 1]) > slopeThreshold) {
                for (uint16_t k = 0; k < peakgroup; k++) {
                    uint16_t groupIndex = j + k - n + 2;
                    groupIndex = constrain_int16(groupIndex, 0, length - 1);
                    xx[k] = groupIndex;
                    yy[k] = input[groupIndex];
                }
                if (peakgroup < 3) {
                    vector_max_float(yy, peakgroup, &peakY, &pindex);
                } else {
                    peakY = vector_mean_float(yy, peakgroup);
                    pindex = val2index(yy, peakgroup, peakY);
                }
                peakX = xx[pindex];
                //hal.console->printf("zero %d, gindex %d -> %d, index %d, val %f\n", j, j -n +2, j+peakgroup -1  - n +2, peakX, peakY);
                // see if we have a valid peak
                if (isfinite(peakY) && peakY >= ampThreshold) {
                    // record in amplitude order
                    for (int16_t i = 0; i < peaklen; i++) {
                        if (i >= numpeaks) {
                            peaks[i] = peakX;
                            break;
                        }
                        if (peakY > input[peaks[i]]) {
                            for (int16_t a = peaklen - 1; a > i; a--) {
                                peaks[a] = peaks[a - 1];
                            }
                            peaks[i] = peakX;
                            break;
                        }
                    }
                    numpeaks++;
                }
            }
        }
    }

    return numpeaks;
}

// Returns the index and the value of the element of a vector that is closest to val
uint16_t DSP::val2index(const float* vector, uint16_t n, float val) const
{
    float minval = FLT_MAX;
    uint16_t minidx = 0;
    for (uint16_t i = 0; i < n; i++) {
        float dif = fabsf(vector[i] - val);
        if (dif < minval) {
            minval = dif;
            minidx = i;
        }
    }
    return minidx;
}

// First derivative of vector using 2-point central difference.
void DSP::derivative(const float* input, float* output, uint16_t n) const
{
    output[0] = input[1] - input[0];
    output[n - 1] = input[n - 1] - input[n - 2];
    for (uint16_t i = 1; i < n - 1; i++) {
        output[i] = (input[i + 1] - input[i - 1]) / 2.0f;
    }
}

// smooth a vector in-place
void DSP::fastsmooth(float* input, uint16_t n, uint16_t smoothwidth) const
{
    float window[smoothwidth];
    memset(window, 0, smoothwidth * sizeof(float));
    float sumpoints = 0.0f;
    for (int i = 0; i < smoothwidth; i++) {
        sumpoints += input[i];
    }
    uint16_t halfw = (smoothwidth + 1) >> 1;
    for (int i = 0; i < n - smoothwidth; i++) {
        window[i % smoothwidth] = sumpoints;
        sumpoints -= input[i];
        sumpoints += input[i + smoothwidth];
        input[i] = window[(i + smoothwidth - 1) % smoothwidth] / smoothwidth;
    }
    uint16_t last = n - smoothwidth + halfw;
    input[last] = 0.0f;
    for (int i = last + 1; i < n; i++) {
        input[last] += input[i];
    }
    input[n - smoothwidth + halfw] /= smoothwidth;
    for (int i = last + 1; i < n; i++) {
        input[i] = 0.0f;
    }
}

#endif // HAL_WITH_DSP
