#ifndef ROBOT_BULLET_SIMULATOR_HPP
#define ROBOT_BULLET_SIMULATOR_HPP

#include <memory>

#include <btBulletDynamicsCommon.h>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>

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

            _filterCallback = new MyOverlapFilterCallback2();

            // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            _dispatcher = new btCollisionDispatcher(_collisionConfiguration);

            _pairCache = new btHashedOverlappingPairCache();

            _pairCache->setOverlapFilterCallback(_filterCallback);

            _broadphase = new btDbvtBroadphase(_pairCache); // btSimpleBroadphase();

            _solver = new btMultiBodyConstraintSolver;

            _dynamicsWorld = new btMultiBodyDynamicsWorld(_dispatcher, _broadphase, _solver, _collisionConfiguration);

            _dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
        }

        ~Simulator();

    protected:
        btAlignedObjectArray<btCollisionShape*> _collisionShapes;
        MyOverlapFilterCallback2* _filterCallback;
        btOverlappingPairCache* _pairCache;
        btBroadphaseInterface* _broadphase;
        btCollisionDispatcher* _dispatcher;
        btMultiBodyConstraintSolver* _solver;
        btDefaultCollisionConfiguration* _collisionConfiguration;
        btMultiBodyDynamicsWorld* _dynamicsWorld;
    };
} // namespace robot_bullet

#endif // ROBOT_BULLET_SIMULATOR_HPP