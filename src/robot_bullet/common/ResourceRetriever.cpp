#include "robot_bullet/common/ResourceRetriever.hpp"

#include "robot_bullet/common/Console.hpp"
#include <sstream>

namespace robot_bullet {
    namespace common {

        //==============================================================================
        std::string ResourceRetriever::readAll(const Uri& uri)
        {
            auto resource = retrieve(uri);

            if (!resource) {
                std::stringstream ss;
                ss << "Failed retrieving a resource from '" << uri.toString() << "'.";
                throw std::runtime_error(ss.str());
            }

            return resource->readAll();
        }

        //==============================================================================
        std::string ResourceRetriever::getFilePath(const Uri& /*uri*/)
        {
            return "";
        }

    } // namespace common
} // namespace robot_bullet
