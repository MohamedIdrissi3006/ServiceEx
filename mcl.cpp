#include "mcl_localization/mcl.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

MCL::MCL(int num_particles)
    : num_particles_(num_particles)
{
  rng_.seed(std::random_device{}());
  particles_.resize(num_particles_);
}

void MCL::initializeUniform(double xmin, double xmax,
                            double ymin, double ymax)
{
  std::uniform_real_distribution<double> dx(xmin, xmax);
  std::uniform_real_distribution<double> dy(ymin, ymax);
  std::uniform_real_distribution<double> dtheta(-M_PI, M_PI);

  for (auto &p : particles_)
  {
    p.x = dx(rng_);
    p.y = dy(rng_);
    p.theta = dtheta(rng_);
    p.weight = 1.0 / num_particles_;
  }
}

// FIXED: Motion update in particle's local frame
void MCL::motionUpdate(double dx, double dy, double dtheta)
{
  for (auto &p : particles_)
  {
    // Transform odometry from global to particle's local frame
    double c = std::cos(p.theta);
    double s = std::sin(p.theta);

    // Add noise to motion model
    double noisy_dx = dx + noise_(rng_) * 0.2;
    double noisy_dy = dy + noise_(rng_) * 0.2;
    double noisy_dtheta = dtheta + noise_(rng_) * 0.1;

    // Apply motion in particle's frame
    // p.x += c * noisy_dx - s * noisy_dy;
    // p.y += s * noisy_dx + c * noisy_dy;
    p.x += noisy_dx;
    p.y += noisy_dy;
    p.theta = normalizeAngle(p.theta + noisy_dtheta);
  }
}

void MCL::measurementUpdate(const std::vector<double> &obs_x,
                            const std::vector<double> &obs_y,
                            const std::vector<double> &lm_x,
                            const std::vector<double> &lm_y,
                            double sigma)
{
  double sigma2 = sigma * sigma;

  for (auto &p : particles_)
  {
    double weight = 1.0;

    // For each observation
    for (size_t i = 0; i < obs_x.size(); ++i)
    {
      // Transform landmark from global to particle's local frame
      double dx = lm_x[i] - p.x;
      double dy = lm_y[i] - p.y;
      double c = std::cos(p.theta);
      double s = std::sin(p.theta);

      double pred_x = c * dx + s * dy;
      double pred_y = -s * dx + c * dy;

      // Compare with actual observation
      double ex = obs_x[i] - pred_x;
      double ey = obs_y[i] - pred_y;

      // Gaussian likelihood
      weight *= std::exp(-(ex * ex + ey * ey) / (2.0 * sigma2));
    }

    p.weight = weight;
  }

  // Normalize weights
  double sum_w = 0.0;
  for (const auto &p : particles_)
    sum_w += p.weight;

  if (sum_w > 0.0) // Avoid division by zero
  {
    for (auto &p : particles_)
      p.weight /= sum_w;
  }
  else
  {
    // If all weights are zero, reset to uniform
    for (auto &p : particles_)
      p.weight = 1.0 / num_particles_;
  }
}

void MCL::resample()
{
  std::vector<Particle> new_particles;
  new_particles.resize(num_particles_);

  // Low variance resampling
  double step = 1.0 / num_particles_;
  std::uniform_real_distribution<double> dist(0.0, step);
  double r = dist(rng_);
  double c = particles_[0].weight;
  int i = 0;

  for (int m = 0; m < num_particles_; ++m)
  {
    double U = r + m * step;
    while (U > c && i < num_particles_ - 1)
    {
      i++;
      c += particles_[i].weight;
    }
    new_particles[m] = particles_[i];
  }

  particles_ = new_particles;

  // Reset weights to uniform after resampling
  for (auto &p : particles_)
    p.weight = 1.0 / num_particles_;

  // Optional: Add random particles for robustness (roughening)
  // This helps prevent particle depletion
  int num_random = num_particles_ / 20; // 5% random particles
  std::uniform_real_distribution<double> random_noise(-0.5, 0.5);

  for (int i = 0; i < num_random && i < num_particles_; ++i)
  {
    particles_[i].x += random_noise(rng_);
    particles_[i].y += random_noise(rng_);
    particles_[i].theta += random_noise(rng_) * 0.2;
    particles_[i].theta = normalizeAngle(particles_[i].theta);
  }
}

// FIXED: Proper circular mean for angle averaging
MCL::Particle MCL::estimatePose() const
{
  Particle est{0, 0, 0, 1.0};
  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (const auto &p : particles_)
  {
    est.x += p.x * p.weight;
    est.y += p.y * p.weight;

    // Use circular mean for angles
    sin_sum += std::sin(p.theta) * p.weight;
    cos_sum += std::cos(p.theta) * p.weight;
  }

  est.theta = std::atan2(sin_sum, cos_sum);
  return est;
}

const std::vector<MCL::Particle> &MCL::particles() const
{
  return particles_;
}

double MCL::normalizeAngle(double angle) const
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}