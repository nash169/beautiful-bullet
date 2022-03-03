#include <beautiful_bullet/bodies/MultiBody.hpp>
#include <beautiful_bullet/bodies/RigidBody.hpp>

using namespace beautiful_bullet;

class Control1 : public control::RigidBodyCtr {
public:
    Control1()
    {
        std::cout << "Hello 1" << std::endl;
    }

    Eigen::VectorXd action(const bodies::RigidBody& body) const override
    {
        Eigen::VectorXd u;
        return u;
    }
};

class Control2 : public control::RigidBodyCtr {
public:
    Control2()
    {
        std::cout << "Hello 2" << std::endl;
    }

    Eigen::VectorXd action(const bodies::RigidBody& body) const override
    {
        Eigen::VectorXd u;
        return u;
    }
};

class Control3 : public control::MultiBodyCtr {
public:
    Control3()
    {
        std::cout << "Hello 3" << std::endl;
    }

    Eigen::VectorXd action(const bodies::MultiBody& body) const override
    {
        Eigen::VectorXd u;
        return u;
    }
};

class Control4 : public control::MultiBodyCtr {
public:
    Control4()
    {
        std::cout << "Hello 4" << std::endl;
    }

    Eigen::VectorXd action(const bodies::MultiBody& body) const override
    {
        Eigen::VectorXd u;
        return u;
    }
};

int main(int argc, char const* argv[])
{
    std::vector<std::unique_ptr<control::RigidBodyCtr>> controllers1;
    controllers1.push_back(std::make_unique<Control1>());
    controllers1.push_back(std::make_unique<Control2>());

    std::vector<std::unique_ptr<control::MultiBodyCtr>> controllers2;
    controllers2.push_back(std::make_unique<Control3>());
    controllers2.push_back(std::make_unique<Control4>());

    return 0;
}
