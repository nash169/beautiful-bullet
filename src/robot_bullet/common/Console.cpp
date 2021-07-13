#include "robot_bullet/common/Console.hpp"

#include <iostream>

namespace robot_bullet {
    namespace common {

        std::ostream& colorMsg(const std::string& _msg, int _color)
        {
            std::cout << "\033[1;" << _color << "m" << _msg << "\033[0m ";
            return std::cout;
        }

        std::ostream& colorErr(
            const std::string& _msg,
            const std::string& _file,
            unsigned int _line,
            int _color)
        {
            int index = _file.find_last_of("/") + 1;

            std::cerr << "\033[1;" << _color << "m" << _msg << " ["
                      << _file.substr(index, _file.size() - index) << ":" << _line
                      << "]\033[0m ";

            return std::cerr;
        }

    } // namespace common
} // namespace robot_bullet
