#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

// Corrade
#include <Corrade/Containers/EnumSet.h>

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
        double mass = 1;
        std::string material = "default";
        float radius = 1.0f; // Sphere, cylinder, cone, capsule
        float height = 1.0f; // Cylinder, cone, capsule
        float xLength = 1.0f; // Box
        float yLength = 1.0f; // Box
        float zLength = 1.0f; // Box, ground
    };

    class Agent {
    public:
        // Constructor
        Agent(Simulator& simulator, const std::string& model, const std::string& name = "agent", const AgentParams& params = AgentParams())
        {
            // Check if we are loading an URDF model
            if (model.size() > 5 && !model.compare(model.size() - 5, 5, ".urdf")) {
                importers::ImporterURDF importer;

                if (importer.loadURDF(model.c_str())) {
                    int rootLinkIndex = importer.getRootLinkIndex();

                    b3Printf("urdf root link index = %d\n", rootLinkIndex);

                    utils::MultiBodyCreator mb_creator;

                    btTransform identityTrans;

                    identityTrans.setIdentity();

                    utils::ConvertURDF2Bullet(importer, mb_creator, identityTrans, simulator.getWorld(), true, importer.getPathPrefix());

                    _multiBody = mb_creator.getBulletMultiBody();

                    if (_multiBody) {
                        //kuka without joint control/constraints will gain energy explode soon due to timestep/integrator
                        //temporarily set some extreme damping factors until we have some joint control or constraints
                        _multiBody->setAngularDamping(0 * 0.99);
                        _multiBody->setLinearDamping(0 * 0.99);
                        b3Printf("Root link name = %s", importer.getLinkName(importer.getRootLinkIndex()).c_str());
                    }
                }

                // construct inverse model
                btInverseDynamics::btMultiBodyTreeCreator id_creator;
                if (-1 == id_creator.createFromBtMultiBody(_multiBody, false)) {
                    b3Error("error creating tree\n");
                }
                else {
                    _inverseModel = btInverseDynamics::CreateMultiBodyTree(id_creator);
                }
            }
            else {
                if (!model.compare("box")) {
                    _type = AgentType::BOX;
                    _box = new btBoxShape{{_params.xLength, _params.yLength, _params.zLength}};
                }

                if (!model.compare("sphere")) {
                    _type = AgentType::SPHERE;
                    _sphere = new btSphereShape{_params.radius};
                }
            }
        }

        // Destructor
        ~Agent()
        {
            delete _inverseModel;
        }

    protected:
        // Bullet MuliBody Object
        btMultiBody* _multiBody;

        // Basic shapes
        btBoxShape* _box;
        btSphereShape* _sphere;

        // Agent name
        std::string _name;

        // Agent type
        AgentTypes _type;

        // Agent parameters
        AgentParams _params;

        // Inverse Dynamics Model (Pinocchio will handle this)
        btInverseDynamics::MultiBodyTree* _inverseModel;
    }; // namespace robot_raisim
} // namespace robot_bullet

#endif // ROBOT_BULLET_AGENT_HPP