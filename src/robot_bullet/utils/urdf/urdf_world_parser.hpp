#pragma once

#include <string>

#include <urdf_model/model.h>
#include <urdf_model/pose.h>
#include <urdf_model/twist.h>
#include <urdf_world/world.h>

#include "robot_bullet/common/ResourceRetriever.hpp"
#include "robot_bullet/common/Uri.hpp"

namespace robot_bullet {
    namespace utils {
        namespace urdf_parsing {

            /// We need a customized version of the Entity class, because we need to keep
            /// track of a Skeleton's uri in order to correctly handle relative file paths.
            class Entity {
            public:
                Entity() = default;

                /// Copy over a standard urdfEntity
                Entity(const urdf::Entity& urdfEntity);

                std::shared_ptr<urdf::ModelInterface> model;
                urdf::Pose origin;
                urdf::Twist twist;

                robot_bullet::common::Uri uri;
            };

            class World {
            public:
                std::string name;
                std::vector<Entity> models;
            };

            std::shared_ptr<World> parseWorldURDF(
                const std::string& xml_string,
                const robot_bullet::common::Uri& _baseUri,
                const common::ResourceRetrieverPtr& retriever);

        } // namespace urdf_parsing
    } // namespace utils
} // namespace robot_bullet
