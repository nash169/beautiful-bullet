#ifndef ROBOT_BULLET_COMMON_RESOURCE_HPP_
#define ROBOT_BULLET_COMMON_RESOURCE_HPP_

#include <cstddef>
#include <memory>
#include <string>

namespace robot_bullet {
    namespace common {

        /// \brief Resource provides file-like access to a resource loaded from URI.
        ///
        /// It is expected that each \a ResourceRetriever will provide a concrete /
        /// instantiation of the Resource class. This interface exposes an similar API
        /// to that of the the standard C file manipulation functions.
        class Resource {
        public:
            /// \brief Position to seek relative to.
            enum SeekType {
                SEEKTYPE_CUR, ///< Current position.
                SEEKTYPE_END, ///< End of file.
                SEEKTYPE_SET ///< Begining of file.
            };

            virtual ~Resource() = default;

            /// \brief Return the size of the resource, in bytes.
            virtual std::size_t getSize() = 0;

            /// \brief Return the current value of the position indicator.
            /// \note This method has the same API as the standard ftell function.
            virtual std::size_t tell() = 0;

            /// \brief Set the position indicator to a new position.
            /// \param[in] _offset Offset, in bytes, relative to _origin.
            /// \param[in] _origin Position used as the reference of _offset.
            /// \note This method has the same API as the standard fseek function.
            virtual bool seek(ptrdiff_t _offset, SeekType _origin) = 0;

            /// \brief Read _count element, each of size _size, into _buffer.
            /// \param[out] _buffer Pointer to a block of memory with a size of at least
            ///                     (_size * _count) bytes.
            /// \param[in] _size Size, in bytes, of each element.
            /// \param[in] _count Number of elements, each of _size bytes.
            /// \note This method has the same API as the standard fread function.
            virtual std::size_t read(void* _buffer, std::size_t _size, std::size_t _count)
                = 0;

            /// Reads all data from this resource, and returns it as a string.
            ///
            /// \return The string retrieved from the resource.
            /// \throw std::runtime_error when failed to read sucessfully.
            virtual std::string readAll();
        };

        using ResourcePtr = std::shared_ptr<Resource>;

    } // namespace common
} // namespace robot_bullet

#endif // ifndef ROBOT_BULLET_COMMON_RESOURCE_HPP_
