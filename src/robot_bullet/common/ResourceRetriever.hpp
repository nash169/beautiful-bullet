#ifndef ROBOT_BULLET_COMMON_RESOURCERETRIEVER_HPP_
#define ROBOT_BULLET_COMMON_RESOURCERETRIEVER_HPP_

#include "robot_bullet/common/Resource.hpp"
#include "robot_bullet/common/Uri.hpp"
#include <memory>
#include <string>

namespace robot_bullet {
    namespace common {

        /// ResourceRetriever provides methods for testing for the existance of and
        /// accessing the content of a resource specified by URI.
        class ResourceRetriever {
        public:
            virtual ~ResourceRetriever() = default;

            /// Returns whether the resource specified by a URI exists.
            virtual bool exists(const Uri& uri) = 0;

            /// Returns the resource specified by a URI or nullptr on failure.
            virtual ResourcePtr retrieve(const Uri& uri) = 0;

            /// Reads all data from the resource of uri, and returns it as a string.
            ///
            /// \param[in] uri URI to the resource to be retrieved.
            /// \return The string retrieved from the resource.
            /// \throw std::runtime_error when failed to read sucessfully.
            virtual std::string readAll(const Uri& uri);

            /// Returns absolute file path to \c uri; an empty string if unavailable.
            ///
            /// This base class returns an empty string by default.
            virtual std::string getFilePath(const Uri& uri);

            // We don't const-qualify for exists, retrieve, readAll, and getFilePath here.
            // Derived classes of ResourceRetriever will be interacting with external
            // resources that you don't necessarily have control over so we cannot
            // guarantee that you get the same result every time with the same input Uri.
            // Indeed, const-qualification for those functions is pointless.
        };

        using ResourceRetrieverPtr = std::shared_ptr<ResourceRetriever>;

    } // namespace common
} // namespace robot_bullet

#endif // ifndef ROBOT_BULLET_COMMON_RESOURCERETRIEVER_HPP_
