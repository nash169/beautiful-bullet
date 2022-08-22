#include "beautiful_bullet/Simulator.hpp"

namespace beautiful_bullet {
    Simulator::Simulator() : _timeStep(1e-3), _ground(false)
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

    Simulator::~Simulator()
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

    btMultiBodyDynamicsWorld* Simulator::world()
    {
        return _world;
    }

    std::vector<bodies::MultiBodyPtr>& Simulator::multiBodies()
    {
        return _multiBody;
    }

    std::vector<bodies::RigidBodyPtr>& Simulator::rigidBodies()
    {
        return _rigidBody;
    }

    Simulator& Simulator::setGraphics(std::unique_ptr<graphics::AbstractGraphics> graphics)
    {
        _graphics = std::move(graphics);

        return *this;
    }

    Simulator& Simulator::initGraphics()
    {
        if (!_graphics)
            _graphics = std::make_unique<graphics::AbstractGraphics>();

        _graphics->init(*this);

        return *this;
    }

    Simulator& Simulator::addGround()
    {
        // Ground parameters
        bodies::BoxParams params;
        params.setSize(4, 4, 0.5)
            .setMass(0)
            .setFriction(0.5);

        bodies::RigidBodyPtr ground = std::make_shared<bodies::RigidBody>("box", params);
        ground->setPosition(0, 0, -0.5);

        return add(ground);
    }

    void Simulator::run(double runTime)
    {
        // Reset clock
        _clock = 0;

        // Init graphics
        initGraphics();

        while (runTime < 0 || _clock * _timeStep <= runTime) {
            // Update objects
            for (auto body : _rigidBody)
                body->update();

            // Update agents
            for (auto body : _multiBody)
                body->update();

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

    void Simulator::addRigidBody(btRigidBody* body)
    {
        // Add rigid body
        _world->addRigidBody(body);

        // Add collision shape
        _collisionShapes.push_back(body->getCollisionShape());
    }

    void Simulator::addMultiBody(btMultiBody* body)
    {
        for (int i = -1; i < body->getNumLinks(); i++) {
            if (i >= 0) {
                // Add joint constraints to the world
                if (body->getLink(i).m_jointType == btMultibodyLink::eRevolute || body->getLink(i).m_jointType == btMultibodyLink::ePrismatic)
                    if (body->getLink(i).m_jointLowerLimit <= body->getLink(i).m_jointUpperLimit) {
                        btMultiBodyConstraint* constraint = new btMultiBodyJointLimitConstraint(body, i, body->getLink(i).m_jointLowerLimit, body->getLink(i).m_jointUpperLimit);
                        _world->addMultiBodyConstraint(constraint);
                    }

                // Add collision object to the world
                bool isDynamic = (i == -1 && body->hasFixedBase()) ? false : true;
                int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                    collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                _world->addCollisionObject(body->getLinkCollider(i), collisionFilterGroup, collisionFilterMask);

                // Add collision shape
                _collisionShapes.push_back(body->getLinkCollider(i)->getCollisionShape());
            }
            else {
                // Add collision object to the world
                bool isDynamic = (i == -1 && body->hasFixedBase()) ? false : true;
                int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter),
                    collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

                _world->addCollisionObject(body->getBaseCollider(), collisionFilterGroup, collisionFilterMask);

                // Add collision shape
                _collisionShapes.push_back(body->getBaseCollider()->getCollisionShape());
            }
        }

        _world->addMultiBody(body);
    }

} // namespace beautiful_bullet
