#ifndef ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

#include <magnum_dynamics/MagnumApp.hpp>

#include "robot_bullet/Simulator.hpp"
#include "robot_bullet/graphics/AbstractGraphics.hpp"

namespace robot_bullet {
    namespace graphics {
        using namespace magnum_dynamics;

        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics(double desiredFPS = 25);

            ~MagnumGraphics();

            void init(Simulator* sim) override
            {
            }

            bool done() override;

            bool pause() override;

            void refresh() override;

        protected:
        };
    } // namespace ogre
} // namespace gui
} // namespace robot_raisim

#endif // ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP