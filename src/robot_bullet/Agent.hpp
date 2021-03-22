#ifndef ROBOT_BULLET_AGENT_HPP
#define ROBOT_BULLET_AGENT_HPP

#include <iostream>
// Corrade
#include <Corrade/Containers/EnumSet.h>

#include <LinearMath/btVector3.h>

#include <BulletCollision/btBulletCollisionCommon.h>
// #include <InverseDynamics/btMultiBodyTreeCreator.hpp>

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

    protected:
        // Bullet MultiBody Object
        btMultiBody* _multiBody;
        std::vector<std::vector<std::string>> _visual_meshes;

        // Bullet Rigid Body Object
        btRigidBody* _rigidBody;

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