#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

// One fixed horizontal plane per mapping session, measured along gravity.
// Filtering exported maps must not remove observations from the LIO estimator.
class MapCeilingFilter
{
public:
    void configure(bool enabled, double height)
    {
        if (!std::isfinite(height) || height <= 0.0 || height > 50.0)
            throw std::invalid_argument("map_filter.max_height must be in (0, 50] metres");
        enabled_ = enabled;
        height_ = height;
        ready_ = false;
    }

    bool enabled() const { return enabled_; }
    bool ready() const { return ready_; }

    bool initialize(const std::array<double, 3> &lidar, const std::array<double, 3> &gravity)
    {
        if (ready_) return true;
        double norm = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(lidar[i]) || !std::isfinite(gravity[i])) return false;
            norm += gravity[i] * gravity[i];
        }
        norm = std::sqrt(norm);
        if (!std::isfinite(norm) || norm < 1e-6) return false;
        anchor_ = lidar;
        for (int i = 0; i < 3; ++i) up_[i] = -gravity[i] / norm;
        ready_ = true;
        return true;
    }

    template <typename Point>
    bool keep(const Point &point) const
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) return false;
        if (!enabled_) return true;
        if (!ready_) return false;
        const double height = up_[0] * (point.x - anchor_[0]) +
                              up_[1] * (point.y - anchor_[1]) +
                              up_[2] * (point.z - anchor_[2]);
        return height <= height_;
    }

    template <typename Cloud>
    void filter(Cloud &cloud) const
    {
        if (!enabled_) return;
        cloud.erase(std::remove_if(cloud.begin(), cloud.end(),
                    [this](const typename Cloud::value_type &p) { return !keep(p); }), cloud.end());
    }

private:
    bool enabled_ = false;
    bool ready_ = false;
    double height_ = 1.5;
    std::array<double, 3> anchor_{{0., 0., 0.}};
    std::array<double, 3> up_{{0., 0., 1.}};
};
