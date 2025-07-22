#pragma once
#include <cmath>
#include <numbers>
#include <vector>
#include "units/length.h"
#include "units/angle.h"
#include "pros/rtos.hpp"
#include <random>
#include <chrono>
#include "Eigen/Core"
#include "api/odometry/odometry.hpp"
#include "api/devices/distance.hpp"

using namespace units;
using namespace units::literals;

namespace aekulib
{

    struct Particle
    {
        inches<> positionX;
        inches<> positionY;
        radians<> direction;
        double weight;
    };

    class mcl
    {
      public:
        Eigen::Vector2<inches<>> getEstimatedPosition() const;
        radians<> getEstimatedOrientation() const;

        explicit mcl(const std::shared_ptr<Odometry> &iodometry, std::uint8_t distance_sensor_port);

      private:
        void initialize();
        void updateParticle();
        void updateWeights(inches<> actualSensorReading);
        void resample();

        aekulib::Distance distance_sensor;

        std::default_random_engine generator;
        std::normal_distribution<double> pos_noise_dist_mm;
        std::normal_distribution<double> dir_noise_dist_deg;

        std::shared_ptr<Odometry> odometry = nullptr;

        // arrays for particles
        std::vector<Particle> particles;
        int numParticles = 50;

        double expectedDistance(inches<> pX, inches<> pY, radians<> pDirection);

        radians<> normalizeAngle(radians<> angle);

        pros::Task mcl_update_task;

        void mclTaskLoop();
    };
}