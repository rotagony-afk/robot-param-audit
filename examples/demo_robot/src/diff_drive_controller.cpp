// demo_robot differential drive kinematics.
// The hard-coded constants below are the third place where the robot geometry
// is written down, and they disagree with the URDF.
#include <cmath>
#include <cstdint>

namespace demo_robot
{

constexpr double WHEEL_RADIUS = 0.05;
constexpr double WHEEL_SEPARATION = 0.30;
constexpr double CONTROL_RATE = 50.0;
constexpr std::int32_t TICKS_PER_REV = 1024;

double distancePerTick()
{
  return 2.0 * M_PI * WHEEL_RADIUS / static_cast<double>(TICKS_PER_REV);
}

double odomDistance(std::int32_t ticks)
{
  return static_cast<double>(ticks) * distancePerTick();
}

double wheelSpeedFromCmd(double linear, double angular, bool left)
{
  const double side = left ? -1.0 : 1.0;
  return linear + side * angular * WHEEL_SEPARATION / 2.0;
}

}  // namespace demo_robot
