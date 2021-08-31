#ifndef BEAUTIFUL_BULLET_CONTROL_HPP
#define BEAUTIFUL_BULLET_CONTROL_HPP

#include <Eigen/Core>

namespace beautiful_bullet {
    enum class ControlMode : unsigned int {
        OPERATIONSPACE = 1 << 0,
        CONFIGURATIONSPACE = 1 << 1
    };

    class Control {
    public:
        Control(ControlMode mode = ControlMode::CONFIGURATIONSPACE, int linkRef = -1) : _controlMode(mode), _controlRef(linkRef) {}

        // Control() = default;

        virtual ~Control() {}

        /* Get control mode */
        ControlMode controlMode() const { return _controlMode; }

        /* Get reference link */
        int controlRef() const { return _controlRef; }

        /* Set control mode */
        Control& setControlMode(ControlMode mode, int linkRef = -1)
        {
            _controlMode = mode;
            _controlRef = linkRef;

            return *this;
        }

        /* Get control command */
        template <typename... Args>
        Eigen::VectorXd control(const Args&... args)
        {
            size_t vec_size = 0;
            std::vector<Eigen::VectorXd> arguments = {args...};

            for (auto& arg : arguments)
                vec_size += arg.size();

            Eigen::VectorXd state(vec_size);

            int curr_index = 0;
            for (auto& arg : arguments) {
                state.segment(curr_index, arg.size()) = arg;
                curr_index += arg.size();
            }

            return update(state);
        }

        /* Init controller */
        virtual void init() = 0;

        /* Update controller */
        virtual Eigen::VectorXd update(const Eigen::VectorXd& state) = 0;

    protected:
        // Control mode
        ControlMode _controlMode;

        // Operational space reference
        int _controlRef;
    };

} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_CONTROL_HPP