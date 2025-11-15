#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

namespace bno {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct GestureSample {
    double t;      // czas w sekundach (steady_clock od startu)
    Vec3  accel;   // przyspieszenie w układzie (tutaj: WORLD, patrz add_sample)
    Quat  quat;    // orientacja sensora (Game Rotation Vector)
};

struct GestureResult {
    double t_center;      // czas środka okna gestu
    double duration;      // czas trwania okna (s)
    Vec3   delta_v_world; // zintegrowane a_dyn w układzie świata
    Vec3   baseline_world;// bazowy wektor grawitacji
    char   axis;          // 'X', 'Y', 'Z'
    char   sign;          // '+' lub '-'
    std::string label;    // "UP"/"DOWN"/etc.
};

// Obrót wektora przez kwaternion (q * v * q^{-1})
inline Vec3 rotate_vector_by_quat(const Vec3& v, const Quat& q)
{
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;

    // t = 2 * (q_vec x v)
    const double tx = 2.0 * (qy * v.z - qz * v.y);
    const double ty = 2.0 * (qz * v.x - qx * v.z);
    const double tz = 2.0 * (qx * v.y - qy * v.x);

    // v' = v + w * t + (q_vec x t)
    Vec3 out;
    out.x = v.x + qw * tx + (qy * tz - qz * ty);
    out.y = v.y + qw * ty + (qz * tx - qx * tz);
    out.z = v.z + qw * tz + (qx * ty - qy * tx);
    return out;
}

inline double norm(const Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

class GestureDirectionDetector {
public:
    struct Config {
        double baseline_window_s     = 0.2;  // ile s na estymację grawitacji
        double half_window_s         = 0.3;  // pół okna gestu (pełne ok. 0.6 s)
        double min_dyn_threshold     = 0.5;  // m/s^2 – próg dynamiki (odcina szum)
        double min_peak_magnitude    = 1.5;  // m/s^2 – min. norma a_dyn uznana za gest
        double min_gesture_interval  = 0.8;  // s – minimalny odstęp między gestami
    };

    // UWAGA: bez domyślnego argumentu (= Config()), to powodowało błąd.
    explicit GestureDirectionDetector(const Config& cfg)
        : cfg_(cfg)
    {}

    void add_sample(double t, const Vec3& accel_sensor, const Quat& quat)
    {
        // sensor -> world
        Vec3 accel_world = rotate_vector_by_quat(accel_sensor, quat);

        GestureSample sample{t, accel_world, quat};
        buffer_.push_back(sample);

        const double max_buffer_span = 2.5 * cfg_.half_window_s;
        while (!buffer_.empty() && (t - buffer_.front().t) > max_buffer_span) {
            buffer_.pop_front();
        }

        if (!baseline_computed_) {
            compute_baseline_if_ready();
        }

        if (baseline_computed_) {
            maybe_detect_gesture();
        }
    }

    std::optional<GestureResult> poll_result()
    {
        if (!pending_result_) {
            return std::nullopt;
        }
        auto out = pending_result_;
        pending_result_.reset();
        return out;
    }

    const Vec3& baseline_world() const { return a0_world_; }
    bool has_baseline() const { return baseline_computed_; }

private:
    Config cfg_;
    std::deque<GestureSample> buffer_;
    Vec3 a0_world_{0.0, 0.0, 0.0};
    bool baseline_computed_{false};
    double t_baseline_end_{0.0};
    double last_gesture_time_{-1e9};
    std::optional<GestureResult> pending_result_;

    void compute_baseline_if_ready()
    {
        if (buffer_.empty()) {
            return;
        }

        const double t0 = buffer_.front().t;
        const double window_s = cfg_.baseline_window_s;

        double sumx = 0.0, sumy = 0.0, sumz = 0.0;
        std::size_t count = 0;

        for (const auto& s : buffer_) {
            if ((s.t - t0) > window_s) {
                break;
            }
            sumx += s.accel.x;
            sumy += s.accel.y;
            sumz += s.accel.z;
            ++count;
        }

        if (count < 3) {
            return;
        }

        a0_world_.x = sumx / static_cast<double>(count);
        a0_world_.y = sumy / static_cast<double>(count);
        a0_world_.z = sumz / static_cast<double>(count);
        baseline_computed_ = true;
        t_baseline_end_ = t0 + window_s;
    }

    void maybe_detect_gesture()
    {
        if (buffer_.size() < 3) {
            return;
        }

        const double t_now = buffer_.back().t;
        if ((t_now - last_gesture_time_) < cfg_.min_gesture_interval) {
            return;
        }

        // 1) peak |a_dyn|
        double max_mag = -1.0;
        std::size_t i_peak = 0;

        for (std::size_t i = 0; i < buffer_.size(); ++i) {
            const auto& s = buffer_[i];
            if (s.t < t_baseline_end_) {
                continue;
            }
            Vec3 dyn{
                s.accel.x - a0_world_.x,
                s.accel.y - a0_world_.y,
                s.accel.z - a0_world_.z,
            };
            const double mag = norm(dyn);
            if (mag > max_mag) {
                max_mag = mag;
                i_peak = i;
            }
        }

        if (max_mag < cfg_.min_peak_magnitude) {
            return;
        }

        const double t_peak = buffer_[i_peak].t;
        const double t_start = t_peak - cfg_.half_window_s;
        const double t_end   = t_peak + cfg_.half_window_s;

        // 2) indeksy okna
        std::size_t start_idx = 0;
        while (start_idx < buffer_.size() && buffer_[start_idx].t < t_start) {
            ++start_idx;
        }
        std::size_t end_idx = start_idx;
        while (end_idx < buffer_.size() && buffer_[end_idx].t <= t_end) {
            ++end_idx;
        }

        if (end_idx <= start_idx + 2) {
            return;
        }

        // 3) integracja a_dyn
        Vec3 dv{0.0, 0.0, 0.0};
        const double duration = buffer_[end_idx - 1].t - buffer_[start_idx].t;

        for (std::size_t i = start_idx + 1; i < end_idx; ++i) {
            const auto& prev = buffer_[i - 1];
            const auto& curr = buffer_[i];
            const double dt = curr.t - prev.t;
            if (dt <= 0.0) {
                continue;
            }

            Vec3 dyn{
                curr.accel.x - a0_world_.x,
                curr.accel.y - a0_world_.y,
                curr.accel.z - a0_world_.z,
            };

            const double mag = norm(dyn);
            if (mag < cfg_.min_dyn_threshold) {
                continue;
            }

            dv.x += dyn.x * dt;
            dv.y += dyn.y * dt;
            dv.z += dyn.z * dt;
        }

        const double absx = std::fabs(dv.x);
        const double absy = std::fabs(dv.y);
        const double absz = std::fabs(dv.z);

        char axis;
        double axis_val;
        if (absx >= absy && absx >= absz) {
            axis = 'X';
            axis_val = dv.x;
        } else if (absy >= absx && absy >= absz) {
            axis = 'Y';
            axis_val = dv.y;
        } else {
            axis = 'Z';
            axis_val = dv.z;
        }

        const char sign = (axis_val >= 0.0) ? '+' : '-';

        const double mag_axis = std::max(absx, std::max(absy, absz));
        if (mag_axis < 0.5) {
            return;
        }

        GestureResult res;
        res.t_center       = t_peak;
        res.duration       = duration;
        res.delta_v_world  = dv;
        res.baseline_world = a0_world_;
        res.axis           = axis;
        res.sign           = sign;
        res.label          = axis_sign_to_label(axis, sign);

        pending_result_    = res;
        last_gesture_time_ = t_now;
    }

    static std::string axis_sign_to_label(char axis, char sign)
    {
        switch (axis) {
        case 'X':
            return (sign == '+') ? "UP" : "DOWN";
        case 'Z':
            return (sign == '+') ? "RIGHT" : "LEFT";
        case 'Y':
            return (sign == '+') ? "FORWARD" : "BACKWARD";
        default:
            return "UNKNOWN";
        }
    }
};

} // namespace bno
