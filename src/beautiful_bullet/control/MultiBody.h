/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

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

            /* Destroyer */
            virtual ~MultiBodyCtr() = default;

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