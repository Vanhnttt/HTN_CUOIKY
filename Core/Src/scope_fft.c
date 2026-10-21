#include "scope_fft.h"
#include "signal_calc.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float fft_re[FFT_N];
static float fft_im[FFT_N];
static float fft_win[FFT_N];
static float tw_re[FFT_N/2];
static float tw_im[FFT_N/2];

uint8_t  fft_mag[LCD_W];
uint32_t fft_peak_hz = 0;

void Scope_FFT_Init(void)
{
    for (int i = 0; i < FFT_N; i++)
        fft_win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(FFT_N - 1)));

    for (int k = 0; k < FFT_N / 2; k++)
    {
        float a = -2.0f * (float)M_PI * (float)k / (float)FFT_N;
        tw_re[k] = cosf(a);
        tw_im[k] = sinf(a);
    }
}

void Scope_FFT_Compute(const uint16_t *snap_data, uint32_t fs_hz)
{
    float sum = 0.0f;
    for (int i = 0; i < FFT_N; i++) sum += (float)snap_data[i];
    float mean = sum / (float)FFT_N;

    for (int i = 0; i < FFT_N; i++)
    {
        fft_re[i] = ((float)snap_data[i] - mean) * fft_win[i];
        fft_im[i] = 0.0f;
    }

    int j = 0;
    for (int i = 0; i < FFT_N - 1; i++)
    {
        if (i < j)
        {
            float t;
            t = fft_re[i]; fft_re[i] = fft_re[j]; fft_re[j] = t;
            t = fft_im[i]; fft_im[i] = fft_im[j]; fft_im[j] = t;
        }
        int k = FFT_N >> 1;
        while (k <= j) { j -= k; k >>= 1; }
        j += k;
    }

    for (int len = 2; len <= FFT_N; len <<= 1)
    {
        int half = len >> 1;
        int step = FFT_N / len;
        for (int i = 0; i < FFT_N; i += len)
        {
            for (int k = 0; k < half; k++)
            {
                float wr = tw_re[k * step];
                float wi = tw_im[k * step];
                int u = i + k, v = u + half;
                float tr = fft_re[v] * wr - fft_im[v] * wi;
                float ti = fft_re[v] * wi + fft_im[v] * wr;
                fft_re[v] = fft_re[u] - tr;
                fft_im[v] = fft_im[u] - ti;
                fft_re[u] += tr;
                fft_im[u] += ti;
            }
        }
    }

    float best = 0.0f;
    int   peak_bin = 1;
    for (int i = 1; i < FFT_N / 2; i++)
    {
        float m = sqrtf(fft_re[i] * fft_re[i] + fft_im[i] * fft_im[i]);
        fft_re[i] = m;
        if (m > best) { best = m; peak_bin = i; }
    }
    float peak = (best > FFT_FLOOR) ? best : FFT_FLOOR;

    for (int x = 0; x < LCD_W; x++)
    {
        int bin = 1 + x * (FFT_N / 2 - 2) / (LCD_W - 1);
        int h = (int)(fft_re[bin] / peak * (float)(SCO_H - 1));
        if (h < 0) h = 0;
        if (h >= SCO_H) h = SCO_H - 1;
        fft_mag[x] = (uint8_t)h;
    }

    fft_peak_hz = (best >= FFT_FLOOR / 4.0f)
                  ? (fs_hz * (uint32_t)peak_bin) / FFT_N : 0u;
}
