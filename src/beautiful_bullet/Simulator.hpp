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
        Simulator() : _timeStep(1e-3), _ground(false)
        {
            // collision configuration contains default setup for memory, collision setup
            _collisionConfiguration = new btDefaultCollisionConfiguration();
            // _collisionConfiguration->setConvexConvexMultipointIterations();

            // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            _dispatcher = new btCollisionDispatcher(_collisionConfiguration);

            // broadphase
            _filterCallback = new MyOverlapFilterCallback2();
            _pairCache = new btHashedOverlappingPairCache();
            _pairCache->setOverlapFilterCallback(_filterCallback);
            _broadphase = new btDbvtBroadphase(_pairCache); // btSimpleBroadphase();

            // solver
            // _mlcp = new btLemkeSolver();
            // _solver = new btMultiBodyMLCPConstraintSolver(_mlcp);
            _solver = new btMultiBodyConstraintSolver;

            // create dynamics world
            _world = new btMultiBodyDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfiguration);

            // set world gravity
            _world->setGravity(btVector3(0, 0, -9.81));
        }

        /* Cleanup in the reverse order of creation/initialization */
        ~Simulator()
        {
            // Remove constaints
            for (int i = _world->getNumConstraints() - 1; i >= 0; i--)
                _world->removeConstraint(_world->getConstraint(i));

            // Remove and delete MultiBody constraints
            for (int i = _world->getNumMultiBodyConstraints() - 1; i >= 0; i--) {
                btMultiBodyConstraint* mbc = _world->getMultiBodyConstraint(i);
                _world->removeMultiBodyConstraint(mbc);
                delete mbc;
            }

            // Remove and delete MultiBodies
            for (int i = _world->getNumMultibodies() - 1; i >= 0; i--) {
                btMultiBody* mb = _world->getMultiBody(i);
                _world->removeMultiBody(mb);
                delete mb;
            }

            // Remove and delete the rigid bodies
            for (int i = _world->getNumCollisionObjects() - 1; i >= 0; i--) {
                btCollisionObject* obj = _world->getCollisionObjectArray()[i];
                btRigidBody* body = btRigidBody::upcast(obj);
                if (body && body->getMotionState()) {
                    delete body->getMotionState();
                }
                _world->removeCollisionObject(obj);
                delete obj;
            }

            // delete collision shapes
            for (int j = 0; j < _collisionShapes.size(); j++) {
                btCollisionShape* shape = _collisionShapes[j];
                delete shape;
            }
            _collisionShapes.clear();

            // Clears agents
            _multiBody.clear();
            _multiBody.shrink_to_fit();

            // Clear object
            _rigidBody.clear();
            _rigidBody.shrink_to_fit();

            // Delete MultibodyDynamicsWorld
            delete _world;
            _world = nullptr;

            // Delete MultiBodyConstraintSolver
            delete _solver;
            _solver = nullptr;

            // Delete DbvtBroadphase
            delete _broadphase;
            _broadphase = nullptr;

            // Delete CollisionDispatcher
            delete _dispatcher;
            _dispatcher = nullptr;

            // Delete HashedOverlappingPairCache
            delete _pairCache;
            _pairCache = nullptr;

            // Delete OverlapFilterCallback
            delete _filterCallback;
            _filterCallback = nullptr;

            // Delete DefaultCollisionConfiguration
            delete _collisionConfiguration;
            _collisionConfiguration = nullptr;
        }

        /* Get DynamicsWorld object */
        btMultiBodyDynamicsWorld* world() { return _world; }

        /* Get agents */
        std::vector<bodies::MultiBody>& agents() { return _multiBody; }

        /* Get objects */
        std::vector<bodies::RigidBody>& objects() { return _rigidBody; }

        /* Set graphics */
        Simulator& setGraphics(std::unique_ptr<graphics::AbstractGraphics> graphics)
        {
            _graphics = std::move(graphics);

            return *this;
        }

        /* Add ground into the simulation */
        Simulator& addGround()
        {
            // Ground parameters
            bodies::BoxParams params;
            params.setSize(4, 4, 0.5)
                .setMass(0)
                .setFriction(0.5);

            return add(bodies::RigidBody("box", params).setPosition(0, 0, -0.5));
        }

        /* Add bodies into the simulation */
        template <typename Body, typename... Args>
        Simulator& add(Body& body, Args&... args) // move the agent
        {
            if constexpr (std::is_same_v<Body, bodies::RigidBody>) {
                // Move object
                _rigidBody.push_back(std::move(body));

                // Add rigid body
                _world->addRigidBody(_rigidBody.back().body());

                // Add collision shape
                _collisionShapes.push_back(_rigidBody.back().body()->getCollisionShape());
            }
            else if constexpr (std::is_same_v<Body, bodies::MultiBody>) {
                // Move agent inside simulator
                _multiBody.push_back(std::move(body)); // it does not seem to be working

                for (int i = -1; i < _multiBody.back().body()->getNumLinks(); i++) {
                    if (i >= 0) {
                        // Add joint constraints to the world
                        if (_multiBody.back().body()->getLink(i).m_jointType == btMultibodyLink::eRevolute || _multiBody.back().body()->getLink(i).m_jointType == btMultibodyLink::ePrismatic)
                            if (_multiBody.back().body()->getLink(i).m_jointLowerLimit <= _multiBody.back().body()->getLink(i).m_jointUpperLimit) {
                                btMultiBodyConstraint* constraint = new btMultiBodyJointLimitConstraint(_multiBody.back().body(), i, _multiBody.back().body()->getLink(i).m_jointLowerLimit, _multiBody.back().body()->getLink(i).m_jointUpperLimit);
                                _world->addMultiBodyConstraint(constraint);
                            }

                        // Add collision object to the world
                        bool isDynamic = (i == -1 && _multiBody.back().body()->hasFixedBase()) ? false : true;
                        int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                            collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                        _world->addCollisionObject(_multiBody.back().body()->getLinkCollider(i), collisionFilterGroup, collisionFilterMask);

                        // Add collision shape
                        _collisionShapes.push_back(_multiBody.back().body()->getLinkCollider(i)->getCollisionShape());
                    }
                    else {
                        // Add collision object to the world
                        bool isDynamic = (i == -1 && _multiBody.back().body()->hasFixedBase()) ? false : true;
                        int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                            collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                        _world->addCollisionObject(_multiBody.back().body()->getBaseCollider(), collisionFilterGroup, collisionFilterMask);

                        // Add collision shape
                        _collisionShapes.push_back(_multiBody.back().body()->getBaseCollider()->getCollisionShape());
                    }
                }

                _world->addMultiBody(_multiBody.back().body());
            }

            if constexpr (sizeof...(args) > 0)
                add(args...);

            return *this;
        }

        inline void step(const size_t& time = 1)
        {
            // Update objects
            for (auto& object : _rigidBody)
                object.update();

            // Update agents
            for (auto& agent : _multiBody)
                agent.update();

            // Simulation step
            _world->stepSimulation(_timeStep, 0);

            // Refresh graphics
            if (time % _graphics->desiredFPS() == 0)
                _graphics->refresh();
        }

        /* Run simulation */
        void run(double runTime = -1)
        {
            // Reset clock
            _clock = 0;

            // Init graphics
            if (!_graphics)
                _graphics = std::make_unique<graphics::AbstractGraphics>();

            _graphics->init(*this);

            while (runTime < 0 || _clock * _timeStep <= runTime) {
                // Update objects
                for (auto& object : _rigidBody)
                    object.update();

                // Update agents
                for (auto& agent : _multiBody)
                    agent.update();

                // Simulation step
                _world->stepSimulation(_timeStep, 0);

                // Refresh graphics
                if (_clock % _graphics->desiredFPS() == 0)
                    if (!_graphics->refresh())
                        break;

                // Increment clock
                _clock++;
            }
        }

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

        /* Bodies */
        std::vector<bodies::RigidBody> _rigidBody;
        std::vector<bodies::MultiBody> _multiBody;
    };
} // namespace beautiful_bullet

#endif // BEAUTIFULBULLET_SIMULATOR_HPP