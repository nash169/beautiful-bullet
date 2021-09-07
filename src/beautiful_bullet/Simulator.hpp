#ifndef BEAUTIFUL_BULLET_SIMULATOR_HPP
#define BEAUTIFUL_BULLET_SIMULATOR_HPP

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

#include "beautiful_bullet/Agent.hpp"
#include "beautiful_bullet/Object.hpp"
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
            // remove the rigidbodies from the dynamics world and delete them
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

            delete _collisionConfiguration;
            delete _dispatcher;
            delete _filterCallback;
            delete _pairCache;
            delete _broadphase;
            delete _solver;
            delete _world;
        }

        /* Get DynamicsWorld object */
        btMultiBodyDynamicsWorld* world() { return _world; }

        /* Get agents */
        std::vector<Agent>& agents() { return _agents; }

        /* Get objects */
        std::vector<Object>& objects() { return _objects; }

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
            BoxParams params;
            params
                .setSize(4, 4, 0.5)
                .setMass(0)
                .setFriction(0.5);
            // .setPose(Eigen::Vector3d(0, 0, -0.5));

            _ground = true;

            return addObjects(Object("box", params).setPosition(0, 0, -0.5));
        }

        /* Add (rigidbody) object into the simulation */
        template <typename... Args>
        Simulator& addObjects(const Object& object, Args... args) // move the agent
        {
            // Move object
            _objects.push_back(std::move(object));

            // Add rigid body
            _world->addRigidBody(_objects.back().body());

            // Add collision shape
            _collisionShapes.push_back(_objects.back().body()->getCollisionShape());

            // Check if the object collides with the ground (if present) and move it if necessary
            // Maybe in the future check for the collisions in all the axis
            // Not correct yet without considering the shape of the object
            // if (_ground) {
            //     // ground assumed to be the first object (maybe store the index of the ground)
            //     double dist = _objects.back().body()->getCenterOfMassPosition().z() - _objects[0].body()->getCenterOfMassPosition().z();

            //     if (dist < 0)
            //         _objects.back().setPose(_objects.back().body()->getCenterOfMassPosition().x(),
            //             _objects.back().body()->getCenterOfMassPosition().y(),
            //             _objects.back().body()->getCenterOfMassPosition().z() + dist);
            // }

            if constexpr (sizeof...(args) > 0)
                addObjects(args...);

            return *this;
        }

        /* Add (multibody) agent into the simulation */
        template <typename... Args>
        Simulator& addAgents(const Agent& agent, Args... args) // move the agent
        {
            // Move agent inside simulator
            _agents.push_back(std::move(agent)); // it does not seem to be working

            for (int i = -1; i < _agents.back().body()->getNumLinks(); i++) {
                if (i >= 0) {
                    // Add joint constraints to the world
                    if (_agents.back().body()->getLink(i).m_jointType == btMultibodyLink::eRevolute || _agents.back().body()->getLink(i).m_jointType == btMultibodyLink::ePrismatic)
                        if (_agents.back().body()->getLink(i).m_jointLowerLimit <= _agents.back().body()->getLink(i).m_jointUpperLimit) {
                            btMultiBodyConstraint* constraint = new btMultiBodyJointLimitConstraint(_agents.back().body(), i, _agents.back().body()->getLink(i).m_jointLowerLimit, _agents.back().body()->getLink(i).m_jointUpperLimit);
                            _world->addMultiBodyConstraint(constraint);
                        }

                    // Add collision object to the world
                    bool isDynamic = (i == -1 && _agents.back().body()->hasFixedBase()) ? false : true;
                    int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                        collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                    _world->addCollisionObject(_agents.back().body()->getLinkCollider(i), collisionFilterGroup, collisionFilterMask);
                }
                else {
                    // Add collision object to the world
                    bool isDynamic = (i == -1 && _agents.back().body()->hasFixedBase()) ? false : true;
                    int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                        collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                    _world->addCollisionObject(_agents.back().body()->getBaseCollider(), collisionFilterGroup, collisionFilterMask);
                }
            }

            _world->addMultiBody(_agents.back().body());

            if constexpr (sizeof...(args) > 0)
                addAgents(args...);

            return *this;
        }

        /* Run simulation */
        void run(double runTime = -1)
        {
            // Reset clock
            _clock = 0;

            // Init graphics
            _graphics->init(*this);

            while (runTime < 0 || _clock * _timeStep <= runTime) {
                // Update objects
                for (auto& object : _objects)
                    object.update();

                // Update agents
                for (auto& agent : _agents)
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

        /* Objects */
        std::vector<Object> _objects;

        /* Agents */
        std::vector<Agent> _agents;
    };
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_SIMULATOR_HPP