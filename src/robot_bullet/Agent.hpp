#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

#include <iostream>
// Corrade
#include <Corrade/Containers/EnumSet.h>

#include <LinearMath/btVector3.h>

#include "BulletInverseDynamics/IDConfig.hpp"
#include <BulletCollision/btBulletCollisionCommon.h>
#include <InverseDynamics/btMultiBodyTreeCreator.hpp>

#include <robot_bullet/importers/ImporterURDF.hpp>
#include <robot_bullet/utils/MultiBodyCreator.hpp>
#include <robot_bullet/utils/URDF2Bullet.hpp>

namespace robot_bullet {
    class Simulator; // for access to the world; this will be changed in order not to have to pass the world to the agent

    // check here if it useful to depend on corrade
    enum class AgentType : unsigned int {
        MULTIBODY = 1 << 0,
        SPHERE = 1 << 1,
        CAPSULE = 1 << 2,
        CYLINDER = 1 << 3,
        BOX = 1 << 4,
        RIGIDBODY = SPHERE | CAPSULE | CYLINDER | BOX
    };

    using AgentTypes = Corrade::Containers::EnumSet<AgentType>;
    CORRADE_ENUMSET_OPERATORS(AgentTypes)

    struct AgentParams {
        AgentParams(const btScalar& pMass = 1.f, const std::string& mat = "red", const btVector3& origin = btVector3(0.f, 0.f, 0.f), const btVector3& pBox = btVector3(0.5f, 0.5f, 0.5f), const btScalar& pSphere = 1.f)
            : mass(pMass), box(pBox), sphere(pSphere), material(mat)
        {
            transform.setIdentity();
            transform.setOrigin(origin);
        }

        btScalar mass;
        // Box params
        btVector3 box;
        // Sphere params
        btScalar sphere;
        // Origin
        btTransform transform;
        // Material
        std::string material;
    };

    class Agent {
    public:
        // Constructor
        Agent(Simulator& simulator, const std::string& model, const AgentParams& params = AgentParams());

        // Destructor
        ~Agent();

        btRigidBody* getBody();

        AgentParams& getParams();

        AgentTypes& getType();

        std::vector<importers::LinkVisual> getVisual()
        {
            return _links_visual;
        }

        btMultiBody* getMultiBody()
        {
            return _multiBody;
        }

        btInverseDynamics::MultiBodyTree* getInverseModel()
        {
            return _inverseModel;
        }

        std::vector<btTransform> getLinkPos()
        {
            std::vector<btTransform> transformations(_multiBody->getNumLinks());

            transformations[0] = _multiBody->getBaseWorldTransform();

            btTransform tr = transformations[0];
            btVector3 origin = _multiBody->getBasePos();

            for (size_t i = 1; i < transformations.size(); i++) {
                transformations[i].setIdentity();
                transformations[i].setOrigin(_multiBody->localDirToWorld(i - 1, {0, 0, 0}));
                // transformations[i].setRotation(_multiBody->getParentToLocalRot(i - 1));
                // transformations[i] = transformations[i] * tr;
                // tr = transformations[i];
            }

            return transformations;
        }

        std::vector<btTransform> getLinkPos2()
        {
            update();

            std::vector<btTransform> transformations(_multiBody->getNumLinks());

            // for (size_t i = 0; i < transformations.size(); i++) {
            //     btInverseDynamicsBullet3::mat33* world_T_body = new btInverseDynamicsBullet3::mat33();
            //     btInverseDynamicsBullet3::vec3* world_origin = new btInverseDynamicsBullet3::vec3();

            //     _inverseModel->getBodyOrigin(i, world_origin);
            //     _inverseModel->getBodyTransform(i, world_T_body);

            //     transformations[i].setOrigin(*static_cast<btVector3*>(world_origin));

            //     btQuaternion q;
            //     (*static_cast<btMatrix3x3*>(world_T_body)).getRotation(q);
            //     transformations[i].setRotation(q);
            // }

            for (size_t i = 0; i < transformations.size(); i++) {
                btInverseDynamicsBullet3::mat33* world_T_body = new btInverseDynamicsBullet3::mat33();
                btInverseDynamicsBullet3::vec3* world_origin = new btInverseDynamicsBullet3::vec3();

                _inverseModel->getBodyOrigin(i, world_origin);
                _inverseModel->getBodyTransform(i, world_T_body);

                btTransform tr(*static_cast<btMatrix3x3*>(world_T_body));
                tr.setOrigin(*static_cast<btVector3*>(world_origin));
                transformations[i] = tr;
            }

            return transformations;
        }

        void update()
        {
            if (_multiBody) {
                const int num_dofs = _multiBody->getNumDofs();
                if (_inverseModel) {
                    btInverseDynamics::vecx qdot(num_dofs), q(num_dofs);

                    for (int dof = 0; dof < num_dofs; dof++) {
                        q(dof) = _multiBody->getJointPos(dof);
                        qdot(dof) = _multiBody->getJointVel(dof);
                    }

                    _inverseModel->calculatePositionAndVelocityKinematics(q, qdot);
                }
            }
        }

    protected:
        // Bullet MultiBody Object
        btMultiBody* _multiBody = nullptr;
        btInverseDynamics::MultiBodyTree* _inverseModel = nullptr;
        std::vector<importers::LinkVisual> _links_visual;

        // Bullet Rigid Body Object
        btRigidBody* _rigidBody = nullptr;

        // Agent name
        std::string _name;

        // Agent type
        AgentTypes _type;

        // Agent parameters
        AgentParams _params;

        // Create Rigid Body
        btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape);
    }; // namespace robot_raisim
} // namespace robot_bullet

#endif // ROBOT_BULLET_AGENT_HPP