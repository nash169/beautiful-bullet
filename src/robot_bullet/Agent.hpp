#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

#include <iostream>
#include <unordered_map>

// Corrade
#include <Corrade/Containers/EnumSet.h>

#include <LinearMath/btVector3.h>

#include <BulletCollision/btBulletCollisionCommon.h>

#include "robot_bullet/utils/BulletLoader.hpp"

namespace robot_bullet {
    // class Simulator; // for access to the world; this will be changed in order not to have to pass the world to the agent
    class Agent {
    public:
        // Constructor
        Agent(const std::string& model)
        {
            // Create loader
            _loader = std::make_shared<utils::BulletLoader>();

            // Get multibody
            _body = _loader->parseMultiBody(model);

            // Update model
            update();
        }

        // ~Agent() { delete _body; }

        // Get body pointer
        btMultiBody* body() { return _body; }

        // Update model
        void update() {}

        // Loader
        std::shared_ptr<utils::BulletLoader> loader() { return _loader; }

    protected:
        // Bullet MultiBody Object
        btMultiBody* _body = nullptr;

        // Loader
        std::shared_ptr<utils::BulletLoader> _loader;
    }; // namespace robot_raisim
} // namespace robot_bullet

#endif // ROBOT_BULLET_AGENT_HPP