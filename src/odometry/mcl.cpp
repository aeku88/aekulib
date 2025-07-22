#include "api/odometry/mcl.hpp"
#include <algorithm>
#include <numeric>

namespace aekulib
{

    mcl::mcl(const std::shared_ptr<Odometry> &iodometry, std::uint8_t distance_sensor_port)
        : odometry(iodometry), numParticles(100), // Initialize numParticles (you can change this value)
          particles(numParticles), // Initialize vector with numParticles default-constructed elements
          generator(std::chrono::system_clock::now().time_since_epoch().count()), // Seed random generator
          pos_noise_dist_mm(0.0, 1.5),  // Mean 0, std dev 5mm (TUNE THIS)
          dir_noise_dist_deg(0.0, 2.0), // Mean 0, std dev 2 degrees (TUNE THIS)
          distance_sensor(distance_sensor_port),
          mcl_update_task([this]() { this->mclTaskLoop(); }, "MCL Update Task") // Start the MCL update task
    {
        initialize();
    }

    void mcl::mclTaskLoop()
    {
        while(true)
        {
            updateParticle();
            updateWeights(distance_sensor.get());
            resample();
            pros::delay(10);
        }
    }

    void mcl::initialize()
    {
        for(int i = 0; i < numParticles; i++)
        {
            particles[i].positionX = 0_in;
            particles[i].positionY = 0_in;
            particles[i].direction = M_PI / 2 * 1_rad;
            particles[i].weight = 1.0 / numParticles;
        }
    }

    void mcl::updateParticle()
    {
        for(int i = 0; i < numParticles; i++)
        {
            particles[i].positionX
              += odometry->getPositionChangeX() + millimeters(pos_noise_dist_mm(generator));
            particles[i].positionY
              += odometry->getPositionChangeY() + millimeters(pos_noise_dist_mm(generator));
            particles[i].direction
              += odometry->getOrientationChange() + degrees(dir_noise_dist_deg(generator));

            const double BOX_HALF_SIZE = 72.0;
            particles[i].positionX
              = std::clamp(particles[i].positionX, -BOX_HALF_SIZE * 1_in, BOX_HALF_SIZE * 1_in);
            particles[i].positionY
              = std::clamp(particles[i].positionY, -BOX_HALF_SIZE * 1_in, BOX_HALF_SIZE * 1_in);

            particles[i].direction = normalizeAngle(particles[i].direction);
        }
    }

    void mcl::updateWeights(inches<> actualSensorReading)
    {
        double totalWeight = 0.0;
        for(int i = 0; i < numParticles; i++)
        {
            double expectedSensorReading
              = expectedDistance(particles[i].positionX, particles[i].positionY, particles[i].direction);

            const double SENSOR_NOISE_STD_DEV_INCHES = 2.0;
            double diff = (actualSensorReading - inches(expectedSensorReading)).value();

            particles[i].weight
              *= std::exp(-(diff * diff) / (2 * SENSOR_NOISE_STD_DEV_INCHES * SENSOR_NOISE_STD_DEV_INCHES));

            totalWeight += particles[i].weight;
        }

        if(totalWeight > 0)
        {
            for(int i = 0; i < numParticles; i++)
            {
                particles[i].weight /= totalWeight;
            }
        }
        else
        {
            // remember to do something about this, I have to recreate all the particles
            initialize();
        }
    }

    void mcl::resample()
    {
        std::vector<Particle> newParticles(numParticles);

        double c = particles[0].weight;
        int i = 0;

        std::uniform_real_distribution<double> dist_0_1(0.0, 1.0);

        for(int m = 0; m < numParticles; m++)
        {
            double u = (double(m) / numParticles) + (dist_0_1(generator) / numParticles);

            while(u > c && i < numParticles - 1)
            {
                i++;
                c += particles[i].weight;
            }

            newParticles[m] = particles[i];
        }

        for(int m = 0; m < numParticles; m++)
        {
            newParticles[m].weight = 1.0 / numParticles;
        }

        particles = newParticles;
    }

    double mcl::expectedDistance(inches<> pX, inches<> pY, radians<> pDirection)
    {
        const double X_MIN = -72.0;
        const double X_MAX = 72.0;
        const double Y_MIN = -72.0;
        const double Y_MAX = 72.0;

        double cosTheta = std::cos(pDirection.value());
        double sinTheta = std::sin(pDirection.value());

        double minDistance = std::numeric_limits<double>::max();
        const double EPSILON = 1e-9;

        if(pX.value() > X_MIN && cosTheta < -EPSILON)
        {
            double dist = (X_MIN - pX.value()) / cosTheta;
            double intersectionY = pY.value() + dist * sinTheta;
            if(dist > 0 && intersectionY >= Y_MIN && intersectionY <= Y_MAX)
            {
                minDistance = std::min(minDistance, dist);
            }
        }

        if(pX.value() < X_MAX && cosTheta > EPSILON)
        {
            double dist = (X_MAX - pX.value()) / cosTheta;
            double intersectionY = pY.value() + dist * sinTheta;
            if(dist > 0 && intersectionY >= Y_MIN && intersectionY <= Y_MAX)
            {
                minDistance = std::min(minDistance, dist);
            }
        }

        if(pY.value() > Y_MIN && sinTheta < -EPSILON)
        {
            double dist = (Y_MIN - pY.value()) / sinTheta;
            double intersectionX = pX.value() + dist * cosTheta;
            if(dist > 0 && intersectionX >= X_MIN && intersectionX <= X_MAX)
            {
                minDistance = std::min(minDistance, dist);
            }
        }

        if(pY.value() < Y_MAX && sinTheta > EPSILON)
        {
            double dist = (Y_MAX - pY.value()) / sinTheta;
            double intersectionX = pX.value() + dist * cosTheta;
            if(dist > 0 && intersectionX >= X_MIN && intersectionX <= X_MAX)
            {
                minDistance = std::min(minDistance, dist);
            }
        }

        if(minDistance == std::numeric_limits<double>::max())
        {
            return 9999;
        }

        return minDistance;
    }

    radians<> mcl::normalizeAngle(radians<> angle)
    {
        double val = angle.value();
        val = std::fmod(val, 2.0 * M_PI);
        if(val < 0)
        {
            val += 2.0 * M_PI;
        }
        if(val > M_PI)
        {
            val -= 2.0 * M_PI;
        }
        return radians(val);
    }

    Eigen::Vector2<inches<>> mcl::getEstimatedPosition() const
    {
        double estimatedX_val = 0.0;
        double estimatedY_val = 0.0;

        for(int i = 0; i < numParticles; ++i)
        {
            estimatedX_val += particles[i].positionX.value() * particles[i].weight;
            estimatedY_val += particles[i].positionY.value() * particles[i].weight;
        }
        return Eigen::Vector2<inches<>>(inches(estimatedX_val), inches(estimatedY_val));
    }

    radians<> mcl::getEstimatedOrientation() const
    {
        double x_comp_sum = 0.0;
        double y_comp_sum = 0.0;

        for(int i = 0; i < numParticles; ++i)
        {
            x_comp_sum += std::cos(particles[i].direction.value()) * particles[i].weight;
            y_comp_sum += std::sin(particles[i].direction.value()) * particles[i].weight;
        }

        if(std::abs(x_comp_sum) < 1e-9 && std::abs(y_comp_sum) < 1e-9)
        {
            return 0_rad;
        }

        return radians(std::atan2(y_comp_sum, x_comp_sum));
    }

}