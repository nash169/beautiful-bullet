#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

#include <iostream>
// Corrade
#include <Corrade/Containers/EnumSet.h>

#include <LinearMath/btVector3.h>

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
        AgentParams(const btScalar& pMass = 1.f, const btVector3& origin = btVector3(0.f, 0.f, 0.f), const btVector3& pBox = btVector3(1.f, 1.f, 1.f), const btScalar& pSphere = 1.f)
            : mass(pMass), box(pBox), sphere(pSphere), material("default")
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
        Agent(Simulator& simulator, const std::string& model, const AgentParams& params = AgentParams())
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
        ~Agent()
        {
        }

    protected:
        // Bullet MultiBody Object
        btMultiBody* _multiBody;

        // Bullet Rigid Body Object
        btRigidBody* _rigidBody;

        // Agent name
        std::string _name;

        // Agent type
        AgentTypes _type;

        // Agent parameters
        AgentParams _params;

        // Create Rigid Body
        btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape)
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
            // m_dynamicsWorld->addRigidBody(body);
            // _rigidBody->forceActivationState(DISABLE_DEACTIVATION);
            return body;
        }
    }; // namespace robot_raisim
} // namespace robot_bullet

#endif // ROBOT_BULLET_AGENT_HPP