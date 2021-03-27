#ifndef ROBOT_BULLET_SIMULATOR_HPP
#define ROBOT_BULLET_SIMULATOR_HPP

#include <cmath>
#include <memory>

#include <btBulletDynamicsCommon.h>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>

#include "robot_bullet/Agent.hpp"
#include "robot_bullet/graphics/AbstractGraphics.hpp"

namespace robot_bullet {
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
        Simulator()
        {
            // collision configuration contains default setup for memory, collision setup
            _collisionConfiguration = new btDefaultCollisionConfiguration(); // _collisionConfiguration->setConvexConvexMultipointIterations();

            // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            _dispatcher = new btCollisionDispatcher(_collisionConfiguration);

            // broadphase
            _filterCallback = new MyOverlapFilterCallback2();
            _pairCache = new btHashedOverlappingPairCache();
            _pairCache->setOverlapFilterCallback(_filterCallback);
            _broadphase = new btDbvtBroadphase(_pairCache); // btSimpleBroadphase();

            // solver
            _solver = new btMultiBodyConstraintSolver;

            // create dynamics world
            _dynamicsWorld = new btMultiBodyDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfiguration);

            // set world gravity
            _dynamicsWorld->setGravity(btVector3(0, 0, -9.81));

            // time-step
            _time_step = 0.001;
        }

        ~Simulator() {}

        btMultiBodyDynamicsWorld* getWorld()
        {
            return _dynamicsWorld;
        }

        btAlignedObjectArray<btCollisionShape*>& getCollisionShapes()
        {
            return _collisionShapes;
        }

        void addGround()
        {
            ///create a ground 4.0f, 0.5f, 4.0f - btVector3(btScalar(150.), btScalar(25.), btScalar(150.))
            btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(4.), btScalar(4.), btScalar(0.5)));

            _collisionShapes.push_back(groundShape);

            btTransform groundTransform;
            groundTransform.setIdentity();
            groundTransform.setOrigin(btVector3(0, 0, -0.5));
            // groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
            //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
            btScalar mass(0.);

            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);

            btVector3 localInertia(0, 0, 0);
            if (isDynamic)
                groundShape->calculateLocalInertia(mass, localInertia);

            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
            btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
            _ground = new btRigidBody(rbInfo);
            _ground->setFriction(0.5);

            //add the ground to the dynamics world
            _dynamicsWorld->addRigidBody(_ground, 1, 1 + 2);
        }

        void addAgent(Agent* agent)
        {
            _agents.push_back(agent);
        }

        std::vector<Agent*> getAgents()
        {
            return _agents;
        }

        btRigidBody* getGround()
        {
            return _ground;
        }

        void setGraphics(std::unique_ptr<graphics::AbstractGraphics> graphics)
        {
            _graphics = std::move(graphics);
        }

        void run(double run_time = -1)
        {
            // Reset clock
            _clock = 0;

            _graphics->init(*this);

            uint8_t step = 0;
            uint8_t g_step = std::ceil(40. / _time_step); // 40Hz graphics
            while (true) {
                _dynamicsWorld->stepSimulation(_time_step);
                if (step % g_step == 0)
                    if (!_graphics->refresh())
                        break;
                step++;
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
        btMultiBodyConstraintSolver* _solver; // btSequentialImpulseConstraintSolver

        /* Multibody Dynamic World */
        btMultiBodyDynamicsWorld* _dynamicsWorld; // btDiscreteDynamicsWorld

        /* Collision Shapes */
        btAlignedObjectArray<btCollisionShape*> _collisionShapes;

        /* Graphics */
        std::unique_ptr<graphics::AbstractGraphics> _graphics;

        /* Simulation params */
        double _time_step, _clock, _run_time;

        /* Ground */
        btRigidBody* _ground = nullptr;

        /* Agents */
        std::vector<Agent*> _agents;
    };
} // namespace robot_bullet

#endif // ROBOT_BULLET_SIMULATOR_HPP