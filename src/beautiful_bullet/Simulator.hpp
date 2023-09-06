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

#ifndef BEAUTIFULBULLET_SIMULATOR_HPP
#define BEAUTIFULBULLET_SIMULATOR_HPP

#include <cmath>
#include <memory>

#include <btBulletDynamicsCommon.h>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>

#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>

#include "beautiful_bullet/bodies/MultiBody.hpp"
#include "beautiful_bullet/bodies/RigidBody.hpp"
#include "beautiful_bullet/graphics/AbstractGraphics.hpp"

namespace beautiful_bullet {
    enum MyFilterModes {
        FILTER_GROUPAMASKB_AND_GROUPBMASKA2 = 0,
        FILTER_GROUPAMASKB_OR_GROUPBMASKA2
    };

    struct MyOverlapFilterCallback2 : public btOverlapFilterCallback {
        int m_filterMode;

        MyOverlapFilterCallback2()
            : m_filterMode(FILTER_GROUPAMASKB_AND_GROUPBMASKA2)
        {
        }

        virtual ~MyOverlapFilterCallback2()
        {
        }
        // return true when pairs need collision
        virtual bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
        {
            if (m_filterMode == FILTER_GROUPAMASKB_AND_GROUPBMASKA2) {
                bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
                collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
                return collides;
            }

            if (m_filterMode == FILTER_GROUPAMASKB_OR_GROUPBMASKA2) {
                bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
                collides = collides || (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
                return collides;
            }
            return false;
        }
    };

    class Simulator {
    public:
        // Constructor
        Simulator();

        /* Cleanup in the reverse order of creation/initialization */
        ~Simulator();

        /* Get DynamicsWorld object */
        btMultiBodyDynamicsWorld* world();

        /* Get agents */
        std::vector<std::shared_ptr<bodies::MultiBody>>& multiBodies();

        /* Get objects */
        std::vector<std::shared_ptr<bodies::RigidBody>>& rigidBodies();

        /* Get graphics handle */
        graphics::AbstractGraphics& graphics();

        /* Set graphics */
        Simulator& setGraphics(std::unique_ptr<graphics::AbstractGraphics> graphics);

        /* Init graphics */
        Simulator& initGraphics();

        /* Add ground into the simulation */
        Simulator& addGround();

        /* Add bodies into the simulation */
        template <typename Body, typename... Args>
        Simulator& add(Body body, Args... args)
        {
            if constexpr (std::is_same_v<Body, std::shared_ptr<bodies::RigidBody>>) {
                // Move object
                _rigidBody.push_back(body);

                // Add rigid body
                addRigidBody(body->body());
            }
            else if constexpr (std::is_same_v<Body, std::shared_ptr<bodies::MultiBody>>) {
                // Move agent inside simulator
                _multiBody.push_back(body);

                // Add multi body
                addMultiBody(body->body());
            }

            if constexpr (sizeof...(args) > 0)
                add(args...);

            return *this;
        }

        /* One step simulation */
        inline bool step(const size_t& time = 1)
        {
            // Update objects
            for (auto& object : _rigidBody)
                object->update();

            // Update agents
            for (auto& agent : _multiBody)
                agent->update();

            // Simulation step
            _world->stepSimulation(_timeStep, 0);

            // Refresh graphics
            return time % _graphics->desiredFPS() == 0 ? _graphics->refresh() : true;
        }

        /* Run simulation */
        void run(double runTime = -1);

    protected:
        /* Collision Configuration */
        btDefaultCollisionConfiguration* _collisionConfiguration;

        /* Collision Dispatcher */
        btCollisionDispatcher* _dispatcher;

        /* Broad Phase Interface */
        MyOverlapFilterCallback2* _filterCallback;
        btOverlappingPairCache* _pairCache;
        btBroadphaseInterface* _broadphase;

        /* Multibody Constraints Solver */
        // btMLCPSolverInterface* _mlcp;
        // btMultiBodyMLCPConstraintSolver* _solver;
        btMultiBodyConstraintSolver* _solver; // btSequentialImpulseConstraintSolver

        /* Multibody Dynamic World */
        btMultiBodyDynamicsWorld* _world; // btDiscreteDynamicsWorld

        /* Collision Shapes */
        btAlignedObjectArray<btCollisionShape*> _collisionShapes;

        /* Graphics */
        std::unique_ptr<graphics::AbstractGraphics> _graphics;

        /* Simulation params */
        size_t _clock;
        double _timeStep;

        /* Ground */
        bool _ground;

        /* Shared Pointers Bodies */
        std::vector<std::shared_ptr<bodies::RigidBody>> _rigidBody;
        std::vector<std::shared_ptr<bodies::MultiBody>> _multiBody;

        /* Add RigidBody to World */
        void addRigidBody(btRigidBody* body);

        /* Add MultiBody to World */
        void addMultiBody(btMultiBody* body);
    };
} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_SIMULATOR_HPP