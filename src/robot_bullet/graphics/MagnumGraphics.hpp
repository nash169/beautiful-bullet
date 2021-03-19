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
                for (auto& agent : sim->getAgents()) {
                    if (agent->getType() & AgentType::BOX) {
                    }
                    else if (agent->getType() & AgentType::SPHERE) {
                    }
                }

                for (size_t i = 0; i < sim->getWorld()->getNumMultibodies(); i++) {
                    // sim->getWorld()->getMultiBody(i)
                }
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