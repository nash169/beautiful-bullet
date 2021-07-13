#include "robot_bullet/common/Resource.hpp"

#include "robot_bullet/common/Console.hpp"
#include <exception>
#include <string>

namespace robot_bullet {
    namespace common {

        //==============================================================================
        std::string Resource::readAll()
        {
            std::string content;
            content.resize(getSize());
            const auto result = read(&content.front(), content.size(), 1);
            // Safe because std::string is guaranteed to be contiguous in C++11.

            if (result != 1)
                throw std::runtime_error("Failed reading data from a resource.");

            return content;
        }

    } // namespace common
} // namespace robot_bullet
