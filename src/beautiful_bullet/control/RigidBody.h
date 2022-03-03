#ifndef BEAUTIFULBULLET_CONTROL_RIGIDBODY_H
#define BEAUTIFULBULLET_CONTROL_RIGIDBODY_H

#include "beautiful_bullet/control/AbstractControl.hpp"

namespace beautiful_bullet {
    namespace bodies {
        class RigidBody;
    } // namespace bodies

    namespace control {
        class RigidBodyCtr : public AbstractControl<bodies::RigidBody> {
        };
    } // namespace control

} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_CONTROL_RIGIDBODY_H