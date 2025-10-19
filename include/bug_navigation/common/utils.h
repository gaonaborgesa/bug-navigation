#ifndef BUG_NAVIGATION_COMMON_UTILS_H
#define BUG_NAVIGATION_COMMON_UTILS_H

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#define HIGH 1e6


/**
 * Gets the current time in microseconds.
 */
static inline int getTimeMicro()
{
    auto now = std::chrono::system_clock::now();
    return now.time_since_epoch().count();
}

/**
 * Sleeps for the provided number of seconds.
 */
static inline void sleepFor(const float secs)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(secs * 1000)));
}

/**
 * Wraps angle to range [-PI, PI].
 * @param  angle Angle to normalize.
 * @return       Wrapped angle.
 */
static inline float wrapAngle(const float angle)
{
    const float result = fmod(angle + M_PI, 2.0 * M_PI);
    if(result <= 0) return result + M_PI;
    return result - M_PI;
}

/**
 * Performs a cross product between two 3D vectors.
 * @param  v1 First vector.
 * @param  v2 Second vector.
 * @return    Cross product between v1 and v2.
 */
std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2);

/**
 * Finds the index of the minimum value in the given vector.
 * @param  ranges Vector of floats.
 * @return        The index (location) of the smallest value in v.
 */
int findMinDist(const std::vector<float>& ranges);

/**
 * Finds the index of the minimum range in the given lidar scan, in a given angle range.
 * @param  ranges       Vector of ray ranges.
 * @param  thetas       Vector of ray angles.
 * @param  target_angle The angle the slice is centered around.
 * @param  slice_size   The size of the slice.
 * @return   The index (location) of the smallest value in ranges within the slice.
 */
int findMinDistInSlice(const std::vector<float>& ranges, const std::vector<float>& thetas,
                       float target_angle, float slice_size);

#endif  // BUG_NAVIGATION_COMMON_UTILS_H
