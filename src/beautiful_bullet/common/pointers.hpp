#ifndef BEAUTIFULBULLET_COMMON_POINTERS_HPP
#define BEAUTIFULBULLET_COMMON_POINTERS_HPP

#include <memory>

// Define a typedef for const and non-const version of shared_ptr for the class X
#define COMMON_DECLARE_SHARED(X)       \
    class X;                           \
    using X##Ptr = std::shared_ptr<X>; \
    using Const##X##Ptr = std::shared_ptr<const X>;

// Define a typedef for const and non-const version of weak_ptr for the class X
#define COMMON_DECLARE_WEAK(X)             \
    class X;                               \
    using Weak##X##Ptr = std::weak_ptr<X>; \
    using WeakConst##X##Ptr = std::weak_ptr<const X>;

// Define a typedef for const and non-const version of unique_ptr for the class X
#define COMMON_DECLARE_UNIQUE(X)               \
    class X;                                   \
    using Unique##X##Ptr = std::unique_ptr<X>; \
    using UniqueConst##X##Ptr = std::unique_ptr<const X>;

#endif // BEAUTIFULBULLET_COMMON_POINTERS_HPP