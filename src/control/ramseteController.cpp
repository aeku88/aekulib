#include "api/control/ramseteController.hpp"
#include <memory>

namespace aekulib
{
    RamseteController::RamseteController(
      const std::shared_ptr<RamseteGains> &iramseteGains,
      const std::shared_ptr<Odometry> &iodometry)
    {}

    void RamseteController::step() {}
}