#ifndef ROBOT_BULLET_MULTIBODY_CREATOR_HPP
#define ROBOT_BULLET_MULTIBODY_CREATOR_HPP

#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <btBulletDynamicsCommon.h>

#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btHashMap.h>
#include <LinearMath/btTransform.h>

#include "robot_bullet/importers/URDFJointTypes.hpp"

namespace robot_bullet {
    namespace utils {
        struct GenericConstraintUserInfo {
            int m_urdfIndex;
            int m_urdfJointType;
            btVector3 m_jointAxisInJointSpace;
            int m_jointAxisIndex;
            btScalar m_lowerJointLimit;
            btScalar m_upperJointLimit;
        };

        class MultiBodyCreator {
        public:
            btAlignedObjectArray<int> m_mb2urdfLink;

            MultiBodyCreator();

            virtual ~MultiBodyCreator() {}

            virtual class btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints, btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep);

            virtual class btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape);

            virtual class btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder = 0);

            virtual class btGeneric6DofSpring2Constraint* createPrismaticJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
                const btVector3& jointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit);

            virtual class btGeneric6DofSpring2Constraint* createRevoluteJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
                const btVector3& jointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit);

            virtual class btGeneric6DofSpring2Constraint* createFixedJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB);

            virtual class btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int urdfLinkIndex, int mbLinkIndex, btMultiBody* body);

            virtual void addLinkMapping(int urdfLinkIndex, int mbLinkIndex);

            btMultiBody* getBulletMultiBody();

            btRigidBody* getRigidBody()
            {
                return m_rigidBody;
            }

            int getNum6DofConstraints() const
            {
                return m_6DofConstraints.size();
            }

            btGeneric6DofSpring2Constraint* get6DofConstraint(int index)
            {
                return m_6DofConstraints[index];
            }

        protected:
            btMultiBody* m_bulletMultiBody;

            btRigidBody* m_rigidBody;

            btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_6DofConstraints;
        };
    } // namespace utils
} // namespace robot_bullet

#endif // ROBOT_BULLET_MULTIBODY_CREATOR_HPP