#ifndef ROBOT_BULLET_COMMON_URI_HPP_
#define ROBOT_BULLET_COMMON_URI_HPP_

#include <string>

namespace robot_bullet {
    namespace common {

        class UriComponent final {
        public:
            using value_type = std::string;
            using reference_type = value_type&;
            using reference_const_type = const value_type&;
            using pointer_type = value_type*;
            using pointer_const_type = const value_type*;

            UriComponent();
            UriComponent(reference_const_type _value);

            operator bool() const;

            bool operator!() const;

            UriComponent& operator=(reference_const_type _value);

            reference_type operator*();
            reference_const_type operator*() const;

            pointer_type operator->();
            pointer_const_type operator->() const;

            void assign(reference_const_type _value);
            void reset();

            reference_type get();
            reference_const_type get() const;

            reference_type get_value_or(reference_type _default);
            reference_const_type get_value_or(reference_const_type _default) const;

        private:
            bool mExists;
            std::string mValue;
        };

        /// The Uri struct provides URI parsing and merging functionality based on RFC
        /// 3986.
        ///
        /// We have Uri as a struct rather than class to expose member variables. Many
        /// ResourceRetreiver classes rewrite URIs to other types of URIs (e.g, resolve
        /// 'package://' URIs to 'file://' URIs), which is easier to implement if you
        /// have direct access to the URI components.
        struct Uri final {
            /// Scheme, e.g. 'http', 'file', 'package'
            UriComponent mScheme;

            /// Authority, e.g. 'google.com', 'en.wikipedia.org'
            UriComponent mAuthority;

            /// Path, e.g. '/index.html', '/foo/bar.txt'
            UriComponent mPath;

            /// Query string, i.e. the part of the URI after the ?
            UriComponent mQuery;

            /// Fragment, e.g. the part of the URI after the #
            UriComponent mFragment;

            /// Constructor
            Uri() = default;

            /// Constructor that takes a URI or local path. Internally, this is equivalent
            /// to calling fromStringOrPath(_input) after default constructor.
            ///
            /// We don't declare this constructor as explicit in order to allow implicit
            /// conversion from string so that you can pass in string parameter to a
            /// function that takes Uri.
            Uri(const std::string& _input);

            /// Constructor that takes a URI or local path as const char*. The behavior is
            /// identical to Uri(const std::string&).
            Uri(const char* _input);

            /// Clear the URI by reset()ing all components.
            void clear();

            /// Parse a URI from a string; return success. All the components will be
            /// cleared on failure.
            bool fromString(const std::string& _input);

            /// Parse a local path (i.e. URI with no schema) from a string; return
            /// success. Note that the input path should be absolute path. All the
            /// components will be cleared on failure.
            bool fromPath(const std::string& _path);

            /// Parse a URI or local path (i.e. URI with no schema) from a string; return
            /// success. We assume that any string without a scheme is a path. All the
            /// components will be cleared on failure.
            bool fromStringOrPath(const std::string& _input);

            /// Resolve a relative path reference; return success. All the components will
            /// be cleared on failure.
            bool fromRelativeUri(
                const std::string& _base,
                const std::string& _relative,
                bool _strict = false);

            /// Resolve a relative path reference; return success. All the components will
            /// be cleared on failure.
            bool fromRelativeUri(
                const char* _base, const char* _relative, bool _strict = false);

            /// Resolve a relative path reference; return success. All the components will
            /// be cleared on failure.
            bool fromRelativeUri(
                const Uri& _base, const std::string& _relative, bool _strict = false);

            /// Resolve a relative path reference; return success. All the components will
            /// be cleared on failure.
            bool fromRelativeUri(
                const Uri& _base, const char* _relative, bool _strict = false);

            /// Resolve a relative path reference; return success. All the components will
            /// be cleared on failure.
            bool fromRelativeUri(
                const Uri& _base, const Uri& _relative, bool _strict = false);

            /// Combine the parts of the URI into a string.
            std::string toString() const;

            /// Create URI from a string; return an empty URI on failure.
            static Uri createFromString(const std::string& _input);

            /// Create file URI from a string; return an empty URI on failure.
            static Uri createFromPath(const std::string& _path);

            /// Create general URI or file URI from a string; return an empty URI on
            /// failure.
            static Uri createFromStringOrPath(const std::string& _input);

            /// Create URI resolving a relative path reference; return an empty URI on
            /// failure.
            static Uri createFromRelativeUri(
                const std::string& _base,
                const std::string& _relative,
                bool _strict = false);

            /// Create URI resolving a relative path reference; return an empty URI on
            /// failure.
            static Uri createFromRelativeUri(
                const Uri& _base, const std::string& _relative, bool _strict = false);

            /// Create URI resolving a relative path reference; return an empty URI on
            /// failure.
            static Uri createFromRelativeUri(
                const Uri& _base, const Uri& _relative, bool _strict = false);

            /// Parse a URI from a string; return an empty string on failure.
            static std::string getUri(const std::string& _input);

            /// Resolve a relative path reference; return an empty string on failure.
            static std::string getRelativeUri(
                const std::string& _base,
                const std::string& _relative,
                bool _strict = false);

            /// Resolve a relative path reference; return an empty string on failure.
            static std::string getRelativeUri(
                const Uri& _base, const std::string& _relative, bool _strict = false);

            /// Resolve a relative path reference; return an empty string on failure.
            static std::string getRelativeUri(
                const Uri& _base, const Uri& _relative, bool _strict = false);

            /// Get the path component of the URI as a string.
            std::string getPath() const;

            /// Get the path in the local filesystem as a string. You should use this
            /// function rather than getPath() if you are trying to access a local file.
            /// Note that this function is identical to getPath() for Unix systems, but
            /// differ by the leading '/' on Windows.
            std::string getFilesystemPath() const;

        private:
            /// Implement section 5.2.3 of RFC 3986.
            static std::string mergePaths(const Uri& _base, const Uri& _relative);

            /// Implement section 5.2.4 of RFC 3986.
            static std::string removeDotSegments(const std::string& _path);
        };

    } // namespace common
} // namespace robot_bullet

#endif // ifndef ROBOT_BULLET_COMMON_URI_HPP_
