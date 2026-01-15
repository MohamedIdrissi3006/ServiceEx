#ifndef MCL_LOCALIZATION__MCL_HPP_
#define MCL_LOCALIZATION__MCL_HPP_

#include <vector>
#include <random>

class MCL
{
public:
  struct Particle
  {
    double x;
    double y;
    double theta;
    double weight;
  };

  MCL(int num_particles);

  void initializeUniform(double xmin, double xmax,
                         double ymin, double ymax);

  void motionUpdate(double dx, double dy, double dtheta);
  void measurementUpdate(const std::vector<double>& obs_x,
                         const std::vector<double>& obs_y,
                         const std::vector<double>& lm_x,
                         const std::vector<double>& lm_y,
                         double sigma);

  void resample();
  Particle estimatePose() const;

  const std::vector<Particle>& particles() const;
  void setNumParticles(int num_particles){
      num_particles_ = num_particles;
      // particles_.resize(num_particles_);
  };

private:
  int num_particles_;
  std::vector<Particle> particles_;

  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};

  double normalizeAngle(double angle) const;
};

#endif
