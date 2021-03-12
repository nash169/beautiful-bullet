#include "robot_bullet/utils/MultiBodyCreator.hpp"

namespace robot_bullet {
    namespace utils {
        MultiBodyCreator::MultiBodyCreator()
            : m_bulletMultiBody(0),
              m_rigidBody(0)
        {
        }

        class btMultiBody* MultiBodyCreator::allocateMultiBody(int /* urdfLinkIndex */, int totalNumJoints, btScalar mass, const btVector3& localInertiaDiagonal, bool isFixedBase, bool canSleep)
        {
            //	m_urdf2mbLink.resize(totalNumJoints+1,-2);
            m_mb2urdfLink.resize(totalNumJoints + 1, -2);

            m_bulletMultiBody = new btMultiBody(totalNumJoints, mass, localInertiaDiagonal, isFixedBase, canSleep);
            //if (canSleep)
            //	m_bulletMultiBody->goToSleep();
            return m_bulletMultiBody;
        }

        class btRigidBody* MultiBodyCreator::allocateRigidBody(int urdfLinkIndex, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, class btCollisionShape* colShape)
        {
            btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
            rbci.m_startWorldTransform = initialWorldTrans;
            btRigidBody* body = new btRigidBody(rbci);
            if (m_rigidBody == 0) {
                //only store the root of the multi body
                m_rigidBody = body;
            }
            return body;
        }

        class btMultiBodyLinkCollider* MultiBodyCreator::allocateMultiBodyLinkCollider(int /*urdfLinkIndex*/, int mbLinkIndex, btMultiBody* multiBody)
        {
            btMultiBodyLinkCollider* mbCol = new btMultiBodyLinkCollider(multiBody, mbLinkIndex);
            return mbCol;
        }

        class btGeneric6DofSpring2Constraint* MultiBodyCreator::allocateGeneric6DofSpring2Constraint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB, int rotateOrder)
        {
            btGeneric6DofSpring2Constraint* c = new btGeneric6DofSpring2Constraint(rbA, rbB, offsetInA, offsetInB, (RotateOrder)rotateOrder);

            return c;
        }

        class btGeneric6DofSpring2Constraint* MultiBodyCreator::createPrismaticJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
            const btVector3& jointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit)
        {
            int rotateOrder = 0;
            btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, rotateOrder);
            //todo(erwincoumans) for now, we only support principle axis along X, Y or Z
            int principleAxis = jointAxisInJointSpace.closestAxis();

            GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
            userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
            userInfo->m_jointAxisIndex = principleAxis;

            userInfo->m_urdfJointType = URDFPrismaticJoint;
            userInfo->m_lowerJointLimit = jointLowerLimit;
            userInfo->m_upperJointLimit = jointUpperLimit;
            userInfo->m_urdfIndex = urdfLinkIndex;
            dof6->setUserConstraintPtr(userInfo);

            switch (principleAxis) {
            case 0: {
                dof6->setLinearLowerLimit(btVector3(jointLowerLimit, 0, 0));
                dof6->setLinearUpperLimit(btVector3(jointUpperLimit, 0, 0));
                break;
            }
            case 1: {
                dof6->setLinearLowerLimit(btVector3(0, jointLowerLimit, 0));
                dof6->setLinearUpperLimit(btVector3(0, jointUpperLimit, 0));
                break;
            }
            case 2:
            default: {
                dof6->setLinearLowerLimit(btVector3(0, 0, jointLowerLimit));
                dof6->setLinearUpperLimit(btVector3(0, 0, jointUpperLimit));
            }
            };

            dof6->setAngularLowerLimit(btVector3(0, 0, 0));
            dof6->setAngularUpperLimit(btVector3(0, 0, 0));
            m_6DofConstraints.push_back(dof6);
            return dof6;
        }

        class btGeneric6DofSpring2Constraint* MultiBodyCreator::createRevoluteJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB,
            const btVector3& jointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit)
        {
            btGeneric6DofSpring2Constraint* dof6 = 0;

            //only handle principle axis at the moment,
            //@todo(erwincoumans) orient the constraint for non-principal axis
            int principleAxis = jointAxisInJointSpace.closestAxis();
            switch (principleAxis) {
            case 0: {
                dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_ZYX);
                dof6->setLinearLowerLimit(btVector3(0, 0, 0));
                dof6->setLinearUpperLimit(btVector3(0, 0, 0));

                dof6->setAngularLowerLimit(btVector3(jointLowerLimit, 0, 0));
                dof6->setAngularUpperLimit(btVector3(jointUpperLimit, 0, 0));

                break;
            }
            case 1: {
                dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_XZY);
                dof6->setLinearLowerLimit(btVector3(0, 0, 0));
                dof6->setLinearUpperLimit(btVector3(0, 0, 0));

                dof6->setAngularLowerLimit(btVector3(0, jointLowerLimit, 0));
                dof6->setAngularUpperLimit(btVector3(0, jointUpperLimit, 0));
                break;
            }
            case 2:
            default: {
                dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_XYZ);
                dof6->setLinearLowerLimit(btVector3(0, 0, 0));
                dof6->setLinearUpperLimit(btVector3(0, 0, 0));

                dof6->setAngularLowerLimit(btVector3(0, 0, jointLowerLimit));
                dof6->setAngularUpperLimit(btVector3(0, 0, jointUpperLimit));
            }
            };

            GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
            userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
            userInfo->m_jointAxisIndex = 3 + principleAxis;

            if (jointLowerLimit > jointUpperLimit) {
                userInfo->m_urdfJointType = URDFContinuousJoint;
            }
            else {
                userInfo->m_urdfJointType = URDFRevoluteJoint;
                userInfo->m_lowerJointLimit = jointLowerLimit;
                userInfo->m_upperJointLimit = jointUpperLimit;
            }
            userInfo->m_urdfIndex = urdfLinkIndex;
            dof6->setUserConstraintPtr(userInfo);
            m_6DofConstraints.push_back(dof6);
            return dof6;
        }

        class btGeneric6DofSpring2Constraint* MultiBodyCreator::createFixedJoint(int urdfLinkIndex, btRigidBody& rbA /*parent*/, btRigidBody& rbB, const btTransform& offsetInA, const btTransform& offsetInB)
        {
            btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB);

            GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
            userInfo->m_urdfIndex = urdfLinkIndex;
            userInfo->m_urdfJointType = URDFFixedJoint;

            dof6->setUserConstraintPtr(userInfo);

            dof6->setLinearLowerLimit(btVector3(0, 0, 0));
            dof6->setLinearUpperLimit(btVector3(0, 0, 0));

            dof6->setAngularLowerLimit(btVector3(0, 0, 0));
            dof6->setAngularUpperLimit(btVector3(0, 0, 0));
            m_6DofConstraints.push_back(dof6);
            return dof6;
        }

        void MultiBodyCreator::addLinkMapping(int urdfLinkIndex, int mbLinkIndex)
        {
            if (m_mb2urdfLink.size() < (mbLinkIndex + 1)) {
                m_mb2urdfLink.resize((mbLinkIndex + 1), -2);
            }
            //    m_urdf2mbLink[urdfLinkIndex] = mbLinkIndex;
            m_mb2urdfLink[mbLinkIndex] = urdfLinkIndex;
        }

        btMultiBody* MultiBodyCreator::getBulletMultiBody()
        {
            return m_bulletMultiBody;
        }
    } // namespace utils
} // namespace robot_bullet
