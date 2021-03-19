#include <robot_bullet/Agent.hpp>

#include "robot_bullet/Simulator.hpp"

namespace robot_bullet {
    Agent::Agent(Simulator& simulator, const std::string& model, const AgentParams& params) : _params(params)
    {
        // Check if we are loading an URDF model
        if (model.size() > 5 && !model.compare(model.size() - 5, 5, ".urdf")) {
            std::cout << "Hello" << std::endl;

            importers::ImporterURDF importer;

            if (importer.loadURDF(model.c_str())) {
                int rootLinkIndex = importer.getRootLinkIndex();

                b3Printf("urdf root link index = %d\n", rootLinkIndex);

                utils::MultiBodyCreator mb_creator;

                btTransform identityTrans;

                identityTrans.setIdentity();

                utils::ConvertURDF2Bullet(importer, mb_creator, identityTrans, simulator.getWorld(), true, importer.getPathPrefix());

                for (int i = 0; i < importer.getNumAllocatedCollisionShapes(); i++) {
                    simulator.getCollisionShapes().push_back(importer.getAllocatedCollisionShape(i));
                }

                _multiBody = mb_creator.getBulletMultiBody();

                if (_multiBody) {
                    //kuka without joint control/constraints will gain energy explode soon due to timestep/integrator
                    //temporarily set some extreme damping factors until we have some joint control or constraints
                    _multiBody->setAngularDamping(0 * 0.99);
                    _multiBody->setLinearDamping(0 * 0.99);
                    b3Printf("Root link name = %s", importer.getLinkName(importer.getRootLinkIndex()).c_str());
                }
            }
        }
        else {
            if (!model.compare("box")) {
                /* Set box type */
                _type = AgentType::BOX;
                /* Create box shape */
                btBoxShape* box_shape = new btBoxShape(_params.box);
                simulator.getCollisionShapes().push_back(box_shape);
                /* Create box rigid body */
                _rigidBody = createRigidBody(_params.mass, _params.transform, box_shape);
                /* don't know yet */
                _rigidBody->forceActivationState(DISABLE_DEACTIVATION);
            }
            else if (!model.compare("sphere")) {
                /* Set sphere type */
                _type = AgentType::SPHERE;
                /* Create sphere shape */
                btSphereShape sphere_shape(_params.sphere);
                // simulator.getCollisionShapes().push_back(sphere_shape);
                /* Create box rigid body */
                // _rigidBody = createRigidBody(_params.mass, _params.transform, sphere_shape);
                /* don't know yet */
                // _rigidBody->forceActivationState(DISABLE_DEACTIVATION);
            }
            else {
                b3Warning("Basic shape not found.");
                return;
            }

            // Add rigid body to the world simulation
            simulator.getWorld()->addRigidBody(_rigidBody);

            // Pass agent to simulator
            simulator.addAgent(this);
        }
    }

    // Destructor
    Agent::~Agent()
    {
    }

    btRigidBody* Agent::getBody()
    {
        return _rigidBody;
    }

    AgentParams& Agent::getParams()
    {
        return _params;
    }

    AgentTypes& Agent::getType()
    {
        return _type;
    }

    btRigidBody* Agent::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            shape->calculateLocalInertia(mass, localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

        btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

        btRigidBody* body = new btRigidBody(cInfo);

        body->setUserIndex(-1);

        // _rigidBody->forceActivationState(DISABLE_DEACTIVATION);

        return body;
    }
} // namespace robot_bullet