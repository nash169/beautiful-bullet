#ifndef BEAUTIFULBULLET_CONTROL_ABSTRACTCONTROL_HPP
#define BEAUTIFULBULLET_CONTROL_ABSTRACTCONTROL_HPP

#include <Eigen/Core>
#include <iostream>

namespace beautiful_bullet {
    namespace control {
        template <typename Agent>
        class AbstractControl {
        public:
            virtual Eigen::VectorXd action(const Agent& agent) const = 0;
        };
    } // namespace control

} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_CONTROL_ABSTRACTCONTROL_HPP