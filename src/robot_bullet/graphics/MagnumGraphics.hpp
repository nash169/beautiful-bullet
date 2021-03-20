#ifndef ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP
#define ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP

#include <iostream>
#include <memory>

#include <Magnum/BulletIntegration/MotionState.h>
#include <magnum_dynamics/MagnumApp.hpp>

#include "robot_bullet/Simulator.hpp"
#include "robot_bullet/graphics/AbstractGraphics.hpp"

namespace robot_bullet {
    namespace graphics {
        using namespace magnum_dynamics;

        class MagnumGraphics : public AbstractGraphics {
        public:
            MagnumGraphics() : _done(false), _pause(false)
            {
            }

            ~MagnumGraphics()
            {
                delete _app;
            }

            void init(Simulator& sim) override
            {
                int argc = 0;
                char** argv = NULL;

                _app = new MagnumApp({argc, argv});

                // Add non-static rigid body
                for (size_t i = 0; i < sim.getWorld()->getNonStaticRigidBodies().size(); i++) {
                    std::string shape_type(sim.getWorld()->getNonStaticRigidBodies()[i]->getCollisionShape()->getName());

                    if (!shape_type.compare("Box")) {

                        btVector3 dimension = static_cast<btBoxShape*>(sim.getWorld()->getNonStaticRigidBodies()[i]->getCollisionShape())->getHalfExtentsWithMargin();

                        _app->add("cube", "", Matrix4::scaling(Vector3{0.5f}), 0xff0000_rgbf);
                    }
                    else if (!shape_type.compare("Sphere")) {
                    }

                    auto motionState = new BulletIntegration::MotionState{*(_app->getObjects().back())};
                    sim.getWorld()->getNonStaticRigidBodies()[i]->setMotionState(&motionState->btMotionState());
                }

                for (size_t i = 0; i < sim.getWorld()->getNumMultibodies(); i++) {
                    // sim->getWorld()->getMultiBody(i)
                }
            }

            bool done() override
            {
                return _done;
            }

            bool pause() override
            {
                return _pause;
            }

            bool refresh() override
            {
                return _app->mainLoopIteration();
            }

        protected:
            bool _done, _pause;

            MagnumApp* _app;
        };
    } // namespace graphics
} // namespace robot_bullet

#endif // ROBOT_BULLET_GRAPHICS_MAGNUM_GRAPHICS_HPP