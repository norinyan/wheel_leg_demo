#pragma once
#include <vector>

namespace wheel_leg_demo {
struct Pose2 { double x=0, y=0, yaw=0; };
struct Obstacle { double x, y, length, width; };
struct Bounds { double xmin, xmax, ymin, ymax; };
struct TebConfig {
  double max_v=.3, max_w=.8, max_a=.3, max_alpha=1.;
  double length=.6, width=.78, margin=.05, horizon=2.2, spacing=.15;
  int iterations=30;
};
struct Band { std::vector<Pose2> poses; std::vector<double> dt; double cost=0; bool valid=false; };
double wrap(double a);
bool footprintFree(Pose2 p, Bounds b, const std::vector<Obstacle>& obstacles, const TebConfig& c);
bool sweptFree(Pose2 a, Pose2 b, Bounds bounds, const std::vector<Obstacle>& obstacles, const TebConfig& c);
// Discrete timed elastic band: SE(2) poses and dt are jointly optimized.
Band optimizeBand(const std::vector<Pose2>& seed, double v0, double w0, Bounds bounds,
                  const std::vector<Obstacle>& obstacles, const TebConfig& config);
}
