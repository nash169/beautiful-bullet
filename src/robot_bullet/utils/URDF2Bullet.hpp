#ifndef ROBOT_BULLET_UTILS_URDF_2_BULLET_HPP
#define ROBOT_BULLET_UTILS_URDF_2_BULLET_HPP

#include <stdio.h>
#include <string>

#include "LinearMath/btThreads.h"
#include "LinearMath/btTransform.h"
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>

#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include <Bullet3Common/b3Logging.h>

#include "robot_bullet/importers/ImporterURDF.hpp"
#include "robot_bullet/importers/URDFJointTypes.hpp" // for UrdfMaterialColor cache
#include "robot_bullet/utils/MultiBodyCreator.hpp"

namespace robot_bullet {
    namespace utils {
        struct UrdfVisualShapeCache {
            btAlignedObjectArray<UrdfMaterialColor> m_cachedUrdfLinkColors;
            btAlignedObjectArray<int> m_cachedUrdfLinkVisualShapeIndices;
        };

        void ConvertURDF2Bullet(const importers::ImporterURDF& u2b,
            MultiBodyCreator& creationCallback,
            const btTransform& rootTransformInWorldSpace,
            btMultiBodyDynamicsWorld* world,
            bool createMultiBody,
            const char* pathPrefix,
            int flags = 0,
            UrdfVisualShapeCache* cachedLinkGraphicsShapes = 0);
    } // namespace utils
} // namespace robot_bullet

#endif // ROBOT_BULLET_UTILS_URDF_2_BULLET_HPP
