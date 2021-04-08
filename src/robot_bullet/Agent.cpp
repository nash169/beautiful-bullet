#include <robot_bullet/Agent.hpp>

#include "robot_bullet/Simulator.hpp"

namespace robot_bullet {
    Agent::Agent(Simulator& simulator, const std::string& model, const AgentParams& params) : _params(params)
    {
        // Check if we are loading an URDF model
        if (model.size() > 5 && !model.compare(model.size() - 5, 5, ".urdf")) {
            /* Set multibody type */
            _type = AgentType::MULTIBODY;

            importers::ImporterURDF importer;

            if (importer.loadURDF(model.c_str())) {
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
                }

                // Store visual information
                for (size_t i = 0; i < _multiBody->getNumLinks(); i++) {
                    _linkVisual.push_back(importer.getLinkVisual(i));
                    _linkCollision.push_back(importer.getLinkCollision(i));
                    _link.push_back(importer.getLink(i));
                }

                // Pass agent to simulator
                simulator.addAgent(this);

                // Update model
                update();
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

    btRigidBody& Agent::getRigidBody()
    {
        return *_rigidBody;
    }

    AgentParams& Agent::getParams()
    {
        return _params;
    }

    AgentTypes& Agent::getType()
    {
        return _type;
    }

    btArray<importers::UrdfVisual>& Agent::getLinkVisual(size_t index)
    {
        return _linkVisual[index];
    }

    btArray<importers::UrdfCollision>& Agent::getLinkCollision(size_t index)
    {
        return _linkCollision[index];
    }

    importers::UrdfLink& Agent::getLink(size_t index)
    {
        return _link[index];
    }

    btMultiBody& Agent::getMultiBody()
    {
        return *_multiBody;
    }

    void Agent::update()
    {
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

        // body->setUserIndex(-1);

        // _rigidBody->forceActivationState(DISABLE_DEACTIVATION);

        return body;
    }
} // namespace robot_bullet