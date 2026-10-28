#ifndef RADAR_RESP_FILTER_V8_H
#define RADAR_RESP_FILTER_V8_H
// Portable ESP32/desktop respiratory-rate core. C++11; no heap allocations.
// v8: phase unwrap, detrended normalized ACF, quadrature spectral validation,
// motion/timebase/no-person gates, harmonic ambiguity rejection.
// Non-medical research prototype; RADAR_FS must match actual radar phase packet rate.
#include <stdint.h>
#include <math.h>

namespace radar_v8 {
static constexpr float FS = 50.0f;
static constexpr float K_PI = 3.14159265358979323846f;
static constexpr float K_TAU = 6.28318530717958647692f;
static constexpr int WINDOW = 1000;       // 20 s at 50 Hz
static constexpr int UPDATE_STRIDE = 100; // 2 s
static constexpr int LAG_MIN = 100;       // 30 bpm
static constexpr int LAG_MAX = 333;       // ~9 bpm
static constexpr uint32_t STALE_MS = 700;
static constexpr float STEP_LIMIT_RAD = 3.0f; // ~34 cm/s radial-speed motion gate (was 0.90 -> ~4.5 cm/s; 2.0 still reset on seated micro-motion)
// All thresholds are starting values pending real-person ground-truth validation.
static constexpr float MIN_RMS_RAD = 0.035f;
static constexpr float MIN_AMP_RAD = 0.020f; // 放宽 0.045 -> 0.020，适配弱呼吸相位幅度
static constexpr float MIN_ACF = 0.25f;      // 放宽 0.35 -> 0.25，适配较弱的周期性峰值
static constexpr float MIN_PURITY = 0.30f;
static constexpr float MAX_CV = 0.60f;

enum Status : uint8_t {
    WARMUP = 0,
    OK = 1,
    NO_HUMAN = 2,
    STALE = 3,
    BAD_INPUT = 4,
    MOTION = 5,
    TIMEBASE = 6,
    LOW_SIGNAL = 7,
    LOW_QUALITY = 8,
    AMBIGUOUS = 9,
    TRANSITION = 10
};

inline const char* status_name(Status s) {
    switch (s) {
      case WARMUP: return "warmup";
      case OK: return "ok";
      case NO_HUMAN: return "no_human";
      case STALE: return "stale";
      case BAD_INPUT: return "bad_input";
      case MOTION: return "motion";
      case TIMEBASE: return "timebase";
      case LOW_SIGNAL: return "low_signal";
      case LOW_QUALITY: return "low_quality";
      case AMBIGUOUS: return "ambiguous";
      case TRANSITION: return "transition";
      default: return "unknown";
    }
}

struct Result {
    float raw;
    float lp;
    float hil_ph;   // v8: fitted respiratory phase (NOT v7 Hilbert-FIR phase)
    float br;       // 0 means "not validated", not apnea
    float conf;     // lagged Pearson correlation, 0..1
    float amp;      // fitted fundamental amplitude, radians
    float cv;       // between-half-window fitted-amplitude CV
    float score;    // dimensionless confidence 0..1 (changed from v7)
    float fs;       // observed effective samples/s
    float lag;      // raw selected fundamental lag, samples
    uint32_t last_sample_ms;
    uint16_t count;
    Status status;
    int8_t human;   // -1 unknown, 0 absent, 1 present
};

class Filter {
public:
    Filter() : human_known_(false), human_present_(false) {
        reset(WARMUP);
    }

    void on_human(bool present) {
        bool changed = !human_known_ || (human_present_ != present);
        human_known_ = true;
        human_present_ = present;
        if (changed) reset(present ? WARMUP : NO_HUMAN);
        result_.human = present ? 1 : 0;
    }

    // Call once per actual phase packet (not per JSON output frame).
    void add(float raw, uint32_t now_ms) {
        if (human_known_ && !human_present_) {
            result_.status = NO_HUMAN;
            result_.br = 0.0f;
            return;
        }
        if (!isfinite(raw) || fabsf(raw) > 1000000.0f) {
            reset(BAD_INPUT);
            return;
        }
        if (have_raw_ && (uint32_t)(now_ms - last_ms_) > STALE_MS) {
            reset(STALE);
        }
        result_.raw = raw;
        result_.last_sample_ms = now_ms;
        if (!have_raw_) {
            have_raw_ = true;
            prev_raw_ = raw;
            phase_ = raw;
            lp_ = raw;
            last_ms_ = now_ms;
            result_.lp = lp_;
            return;
        }

        float step = raw - prev_raw_;
        // Auto-handle common [-pi,+pi] and [0,2pi] wrapped phase domains.
        const bool signed_phase = fabsf(raw) <= K_PI + 0.10f &&
                                  fabsf(prev_raw_) <= K_PI + 0.10f;
        const bool unsigned_phase = raw >= -0.10f && prev_raw_ >= -0.10f &&
                                    raw <= K_TAU + 0.10f &&
                                    prev_raw_ <= K_TAU + 0.10f;
        if ((signed_phase || unsigned_phase) && fabsf(step) > K_PI) {
            step -= (step > 0.0f ? K_TAU : -K_TAU);
        }
        prev_raw_ = raw;
        last_ms_ = now_ms;

        if (fabsf(step) > STEP_LIMIT_RAD || !isfinite(step)) {
            reset(MOTION);
            // Keep the new anchor so the next sample can warm up safely.
            have_raw_ = true;
            prev_raw_ = raw;
            phase_ = raw;
            lp_ = raw;
            last_ms_ = now_ms;
            result_.raw = raw;
            result_.lp = raw;
            result_.last_sample_ms = now_ms;
            return;
        }

        phase_ += step;
        // Smooth high-frequency phase jitter. The long-window linear trend is
        // removed inside analyze() to retain ~0.15-0.5 Hz respiration.
        lp_ += 0.20f * (phase_ - lp_);
        result_.lp = lp_;
        buffer_[wr_] = lp_;
        times_[wr_] = now_ms;
        wr_ = (wr_ + 1) % WINDOW;
        if (count_ < WINDOW) ++count_;
        result_.count = (uint16_t)count_;
        if (count_ == WINDOW && ++since_ >= UPDATE_STRIDE) {
            since_ = 0;
            analyze();
        }
    }

    void tick(uint32_t now_ms) {
        if (have_raw_ && (uint32_t)(now_ms - last_ms_) > STALE_MS) {
            reset(STALE);
        }
        result_.human = human_known_ ? (human_present_ ? 1 : 0) : -1;
    }

    const Result& result() const { return result_; }

    void reset(Status reason = WARMUP) {
        wr_ = 0; count_ = 0; since_ = 0;
        have_raw_ = false;
        prev_raw_ = phase_ = lp_ = 0.0f;
        last_ms_ = 0;
        stable_bpm_ = 0.0f;
        proposed_bpm_ = 0.0f;
        proposal_count_ = 0;
        Result zero = {};
        result_ = zero;
        result_.status = reason;
        result_.human = human_known_ ? (human_present_ ? 1 : 0) : -1;
    }

private:
    float buffer_[WINDOW] = {};
    uint32_t times_[WINDOW] = {};
    float detrended_[WINDOW] = {};
    float acf_[LAG_MAX + 1] = {};
    int wr_ = 0, count_ = 0, since_ = 0;
    bool have_raw_ = false, human_known_, human_present_;
    float prev_raw_ = 0, phase_ = 0, lp_ = 0;
    uint32_t last_ms_ = 0;
    float stable_bpm_ = 0, proposed_bpm_ = 0;
    uint8_t proposal_count_ = 0;
    Result result_ = {};

    static float clamp01(float x) {
        return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
    }

    // Quadrature projection is an explicit frequency-domain check independent
    // of the ACF peak. It also yields a stable amplitude and fitted phase.
    // At most a few dozen ACF local maxima are evaluated per 2-second update.
    static float project(const float* x, int start, int length,
                         float period, float* amplitude = nullptr,
                         float* phase = nullptr) {
        const float omega = K_TAU / period;
        const float cw = cosf(omega), sw = sinf(omega);
        float c = 1.0f, s = 0.0f;
        float re = 0.0f, im = 0.0f;
        for (int i = 0; i < length; ++i) {
            const float v = x[start + i];
            re += v * c;
            im -= v * s;
            const float nc = c * cw - s * sw;
            s = s * cw + c * sw;
            c = nc;
        }
        const float mag2 = re * re + im * im;
        if (amplitude) *amplitude = 2.0f * sqrtf(fmaxf(0.0f, mag2)) / length;
        if (phase) *phase = atan2f(im, re); // estimated x(t) = A*cos(w*t + phase)
        return mag2;
    }

    float purity(int lag, float total_energy) const {
        const float energy = project(detrended_, 0, WINDOW, (float)lag);
        return clamp01(2.0f * energy / (WINDOW * total_energy + 1e-12f));
    }

    void invalidate(Status reason) {
        result_.status = reason;
        result_.br = 0.0f;
        stable_bpm_ = 0.0f;
        proposed_bpm_ = 0.0f;
        proposal_count_ = 0;
    }

    void analyze() {
        const int start = wr_;  // oldest ring element after a full-window write
        const uint32_t elapsed = (uint32_t)(
            times_[(start + WINDOW - 1) % WINDOW] - times_[start]);
        if (elapsed == 0) {
            result_.fs = 0.0f;
            invalidate(TIMEBASE);
            return;
        }
        const float measured_fs = 1000.0f * (WINDOW - 1) / (float)elapsed;
        result_.fs = measured_fs;
        // A phase packet rate other than 50 Hz invalidates fixed lag->bpm mapping.
        if (measured_fs < 47.0f || measured_fs > 53.0f) {
            invalidate(TIMEBASE);
            return;
        }

        // OLS remove DC and slow linear drift; a fixed high-pass corner is avoided.
        float sx = 0.0f, six = 0.0f;
        for (int i = 0; i < WINDOW; ++i) {
            const float val = buffer_[(start + i) % WINDOW];
            sx += val;
            six += (i - (WINDOW - 1) * 0.5f) * val;
        }
        const float mean = sx / WINDOW;
        const float i2 = (float)WINDOW * (WINDOW * WINDOW - 1) / 12.0f;
        const float slope = six / i2;
        float energy = 0.0f;
        for (int i = 0; i < WINDOW; ++i) {
            float x = buffer_[(start + i) % WINDOW]
                    - mean - slope * (i - (WINDOW - 1) * 0.5f);
            detrended_[i] = x;
            energy += x * x;
        }
        if (!isfinite(energy) || energy < WINDOW * MIN_RMS_RAD * MIN_RMS_RAD) {
            result_.conf = result_.amp = result_.cv = result_.score = 0.0f;
            invalidate(LOW_SIGNAL);
            return;
        }

        // Per-lag energy normalization avoids the k-dependent overlap bias of
        // v7's R[k]/R[0]. This is a normalized autocorrelation-like coefficient.
        for (int k = LAG_MIN; k <= LAG_MAX; ++k) {
            float cross = 0.0f, ea = 0.0f, eb = 0.0f;
            for (int i = 0; i < WINDOW - k; ++i) {
                float a = detrended_[i], b = detrended_[i + k];
                cross += a * b;
                ea += a * a;
                eb += b * b;
            }
            acf_[k] = cross / (sqrtf(ea * eb) + 1e-12f);
        }

        int winner = 0;
        float winner_quality = -1.0f, winner_power = 0.0f, winner_corr = 0.0f;
        for (int k = LAG_MIN; k <= LAG_MAX; ++k) {
            // ACF peaks and valid boundary peaks; no forced lag doubling.
            if (acf_[k] < 0.20f) continue;
            if (k > LAG_MIN && acf_[k] < acf_[k - 1]) continue;
            if (k < LAG_MAX && acf_[k] < acf_[k + 1]) continue;
            float power = purity(k, energy);
            float q = 0.30f * clamp01(acf_[k]) + 0.70f * power;
            if (q > winner_quality) {
                winner = k; winner_quality = q;
                winner_power = power; winner_corr = acf_[k];
            }
        }
        if (!winner) {
            result_.conf = result_.amp = result_.cv = result_.score = 0.0f;
            invalidate(LOW_QUALITY);
            return;
        }

        // A real waveform can have two peaks per breath. Check the subharmonic
        // using independent frequency energy. Reject unresolved ambiguity.
        int twice = 2 * winner;
        if (twice <= LAG_MAX) {
            float low_power = purity(twice, energy);
            float low_corr = acf_[twice];
            if (low_corr > 0.55f && low_power > 0.18f &&
                low_power > winner_power * 0.30f) {
                if (low_power >= winner_power * 0.55f) {
                    winner = twice;
                    winner_power = low_power;
                    winner_corr = low_corr;
                } else {
                    result_.conf = winner_corr;
                    result_.amp = result_.cv = 0.0f;
                    result_.score = 0.0f;
                    invalidate(AMBIGUOUS);
                    return;
                }
            }
        }

        // Sub-sample parabolic peak interpolation; use only for a real concave peak.
        float lag = (float)winner;
        if (winner > LAG_MIN && winner < LAG_MAX) {
            const float a = acf_[winner - 1], b = acf_[winner], c = acf_[winner + 1];
            const float den = a - 2.0f * b + c;
            if (den < -1e-6f) {
                float d = 0.5f * (a - c) / den;
                if (d > 1.0f) d = 1.0f;
                if (d < -1.0f) d = -1.0f;
                lag += d;
            }
        }

        float amp = 0.0f, phase = 0.0f;
        project(detrended_, 0, WINDOW, lag, &amp, &phase);
        float a0 = 0.0f, a1 = 0.0f;
        project(detrended_, 0, WINDOW / 2, lag, &a0);
        project(detrended_, WINDOW / 2, WINDOW / 2, lag, &a1);
        const float cv = fabsf(a0 - a1) / (a0 + a1 + 1e-6f);
        const float score = clamp01(sqrtf(fmaxf(0.0f, winner_corr) *
                                            fmaxf(0.0f, winner_power)) *
                                      (1.0f - 0.25f * cv));
        result_.conf = clamp01(winner_corr);
        result_.amp = amp;
        result_.cv = cv;
        result_.score = score;
        result_.lag = lag;
        result_.hil_ph = atan2f(sinf(phase + K_TAU * (WINDOW - 1) / lag),
                                cosf(phase + K_TAU * (WINDOW - 1) / lag));

        if (winner_corr < MIN_ACF || winner_power < MIN_PURITY ||
            amp < MIN_AMP_RAD || cv > MAX_CV) {
            invalidate(LOW_QUALITY);
            return;
        }

        const float bpm = FS * 60.0f / lag;
        if (!isfinite(bpm) || bpm < 8.9f || bpm > 30.1f) {
            invalidate(LOW_QUALITY);
            return;
        }
        // Large jumps require two consecutive compatible observations.
        if (stable_bpm_ > 0.0f && fabsf(bpm - stable_bpm_) > 5.0f) {
            if (proposal_count_ == 0 ||
                fabsf(bpm - proposed_bpm_) > 2.5f) {
                proposed_bpm_ = bpm;
                proposal_count_ = 1;
                result_.br = 0.0f;
                result_.status = TRANSITION;
                return;
            }
            ++proposal_count_;
            if (proposal_count_ < 2) {
                result_.br = 0.0f;
                result_.status = TRANSITION;
                return;
            }
            stable_bpm_ = bpm;
        } else if (stable_bpm_ <= 0.0f) {
            stable_bpm_ = bpm;
        } else {
            stable_bpm_ = stable_bpm_ * 0.45f + bpm * 0.55f;
        }
        proposal_count_ = 0;
        proposed_bpm_ = 0.0f;
        result_.br = stable_bpm_;
        result_.status = OK;
    }
};

} // namespace radar_v8
#endif
