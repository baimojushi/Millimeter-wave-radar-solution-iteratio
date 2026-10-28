// Native adversarial tests for the *same* v8 core used by ESP32 wrapper.
// Compile: g++ -std=c++11 -O2 -Wall -Wextra -pedantic test_v8.cpp -o test_v8
#include "radar_resp_filter_v8.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <stdint.h>
using namespace radar_v8;

static uint32_t seed_ = 0x13579BDFu;
static float randu() {
    seed_ = seed_ * 1664525u + 1013904223u;
    return (float)((seed_ >> 8) & 0xFFFFFFu) / 16777216.0f;
}
static float noise(float scale) {
    float u = randu() + randu() + randu() + randu() + randu() + randu();
    return scale * (u - 3.0f) * 1.41421356f;
}
static float wrap_pm_pi(float x) {
    while (x > K_PI) x -= K_TAU;
    while (x < -K_PI) x += K_TAU;
    return x;
}
static int failcount = 0;
static void check(bool ok, const char* name, const Result& r) {
    std::printf("[%s] %-29s br=%6.2f status=%-13s conf=%5.3f amp=%5.3f cv=%5.3f fs=%5.2f\n",
        ok ? "PASS" : "FAIL", name, r.br, status_name(r.status),
        r.conf, r.amp, r.cv, r.fs);
    if (!ok) ++failcount;
}
static void stable(const char* name, float bpm, float amplitude,
                   float noise_amplitude, bool wrapped = false,
                   float second_harmonic = 0.0f) {
    Filter f;
    f.on_human(true);
    Result r = {};
    int good = 0, total = 0;
    float error_sum = 0.0f;
    for (int i = 0; i < 6000; ++i) {
        float t = i / FS;
        float raw = 1.1f + amplitude * std::sin(K_TAU * bpm / 60.0f * t)
                    + second_harmonic * std::sin(K_TAU * 2.0f * bpm / 60.0f * t)
                    + noise(noise_amplitude);
        if (wrapped) raw = wrap_pm_pi(raw + 1.9f);
        f.add(raw, (uint32_t)(i * 20));
        if (i > 4500 && i % 100 == 0) {
            r = f.result();
            ++total;
            if (r.status == OK) {
                ++good;
                error_sum += std::fabs(r.br - bpm);
            }
        }
    }
    r = f.result();
    float mae = good ? error_sum / good : 100;
    bool ok = good >= total * 0.70f && mae <= 1.15f &&
              r.status == OK && std::fabs(r.br - bpm) <= 1.3f;
    check(ok, name, r);
    std::printf("      window valid=%d/%d, MAE=%.2f bpm\n", good, total, mae);
}
int main() {
    stable("bpm=9 lower boundary", 9.0f, 0.75f, 0.03f);
    stable("bpm=12 noisy", 12.0f, 0.70f, 0.10f);
    stable("bpm=16 noisy", 16.0f, 0.70f, 0.10f);
    stable("bpm=20, no half-rate", 20.0f, 0.70f, 0.08f);
    stable("bpm=24, no half-rate", 24.0f, 0.70f, 0.08f);
    stable("bpm=30 upper boundary", 30.0f, 0.70f, 0.02f);
    stable("phase +/-pi wrapping", 16.0f, 0.65f, 0.04f, true);
    stable("weak second harmonic", 10.0f, 0.70f, 0.03f, false, 0.25f);

    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(0.02f * noise(1.0f) + 0.0005f * i, i * 20u);
        const Result r = f.result();
        check(r.br == 0 && r.status != OK, "flat drift/no breath", r);
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(noise(0.20f), i * 20u);
        const Result r = f.result();
        check(r.br == 0 && r.status != OK, "random noise only", r);
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(std::sin(K_TAU * 16.0f / 60.0f * i / FS), i * 20u);
        f.on_human(false);
        const Result r = f.result();
        check(r.status == NO_HUMAN && r.br == 0, "human disappeared", r);
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(std::sin(K_TAU * 16.0f / 60.0f * i / FS), i * 20u);
        f.add(std::numeric_limits<float>::quiet_NaN(), 80100);
        const Result r = f.result();
        check(r.status == BAD_INPUT && r.br == 0 && r.count == 0,
              "NaN packet sanitization", r);
        f.add(std::numeric_limits<float>::infinity(), 80120);
        check(f.result().status == BAD_INPUT, "Inf packet sanitization", f.result());
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(std::sin(K_TAU * 16.0f / 60.0f * i / FS), i * 20u);
        f.tick(83000);
        check(f.result().status == STALE && f.result().br == 0,
              "UART silence timeout", f.result());
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(std::sin(K_TAU * 16.0f / 60.0f * i / FS), i * 25u);
        check(f.result().status == TIMEBASE && f.result().br == 0,
              "wrong sampling clock 40Hz", f.result());
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i)
            f.add(std::sin(K_TAU * 16.0f / 60.0f * i / FS), i * 20u);
        f.add(2.5f, 80000);  // one abrupt phase spike (nonwrap)
        check(f.result().status == MOTION && f.result().br == 0,
              "large step / motion gate", f.result());
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 7000; ++i) {
            const float t = i / FS;
            const float phase = K_TAU * (i < 3200
                 ? 12.0f/60.0f*t
                 : 12.0f/60.0f*(3200/FS) + 22.0f/60.0f*(t-3200/FS));
            f.add(0.8f * std::sin(phase) + noise(0.04f), i * 20u);
        }
        check(f.result().status == OK && std::fabs(f.result().br - 22.0f) < 2.0f,
              "step change 12 -> 22", f.result());
    }
    {
        Filter f; f.on_human(true);
        for (int i = 0; i < 4000; ++i) {
            float t = i / FS;
            float signal = 0.7f * std::sin(K_TAU * 10.0f / 60.0f*t)
                         + 1.0f * std::sin(K_TAU * 20.0f / 60.0f*t);
            f.add(signal + noise(0.02f), i * 20u);
        }
        const Result r = f.result();
        check((r.status != OK || std::fabs(r.br - 10.0f) <= 1.5f) &&
              !(r.status == OK && std::fabs(r.br - 20.0f) <= 1.5f),
              "strong second harmonic (10+20)", r);
    }
    std::printf("\nRESULT: %d failed adversarial tests\n", failcount);
    return failcount ? 1 : 0;
}
