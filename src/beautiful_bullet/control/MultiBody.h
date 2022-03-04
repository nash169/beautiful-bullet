#ifndef BEAUTIFULBULLET_CONTROL_MULTIBODY_H
#define BEAUTIFULBULLET_CONTROL_MULTIBODY_H

#include "beautiful_bullet/control/AbstractControl.hpp"

namespace beautiful_bullet {
    enum class ControlMode : unsigned int {
        OPERATIONSPACE = 1 << 0,
        CONFIGURATIONSPACE = 1 << 1
    };

    namespace bodies {
        class MultiBody;
    } // namespace bodies

    namespace control {
        class MultiBodyCtr : public AbstractControl<bodies::MultiBody> {
        public:
            /* Constructor */
            MultiBodyCtr(const ControlMode& mode = ControlMode::CONFIGURATIONSPACE) : _mode(mode) {}

            /* Get control mode */
            const ControlMode& mode() const { return _mode; }

            /* Get reference link */
            const std::string& frame() const { return _frame; }

            /* Set control mode */
            MultiBodyCtr& setMode(const ControlMode& mode)
            {
                _mode = mode;
                return *this;
            }

            /* Set control frame */
            MultiBodyCtr& setFrame(const std::string& frame)
            {
                _frame = frame;
                return *this;
            }

        protected:
            // Control mode
            ControlMode _mode;

            // Operational space reference
            std::string _frame;
        };
    } // namespace control

} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_CONTROL_MULTIBODY_H