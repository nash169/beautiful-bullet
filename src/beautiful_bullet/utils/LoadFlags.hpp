/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef BEAUTIFUL_BULLET_LOAD_FLAGS_HPP
#define BEAUTIFUL_BULLET_LOAD_FLAGS_HPP

namespace beautiful_bullet {
    enum LoadFlags {
        /*  */
        CUF_USE_SDF = 1,

        /* Use inertia values in URDF instead of recomputing them from collision shape. */
        CUF_USE_URDF_INERTIA = 2,

        /*  */
        CUF_USE_MJCF = 4,

        /*  */
        CUF_USE_SELF_COLLISION = 8,

        /*  */
        CUF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16,

        /*  */
        CUF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32,

        /*  */
        CUF_RESERVED = 64,

        /*  */
        CUF_USE_IMPLICIT_CYLINDER = 128,

        /*  */
        CUF_GLOBAL_VELOCITIES_MB = 256,

        /*  */
        CUF_MJCF_COLORS_FROM_FILE = 512,

        /*  */
        CUF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024,

        /*  */
        CUF_ENABLE_SLEEPING = 2048,

        /*  */
        CUF_INITIALIZE_SAT_FEATURES = 4096,

        /*  */
        CUF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192,

        /*  */
        CUF_PARSE_SENSORS = 16384,

        /*  */
        CUF_USE_MATERIAL_COLORS_FROM_MTL = 32768,

        /*  */
        CUF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536,

        /*  */
        CUF_MAINTAIN_LINK_ORDER = 131072,

        /*  */
        CUF_ENABLE_WAKEUP = 1 << 18,

        /*  */
        CUF_MERGE_FIXED_LINKS = 1 << 19,

        /*  */
        CUF_IGNORE_VISUAL_SHAPES = 1 << 20,

        /*  */
        CUF_IGNORE_COLLISION_SHAPES = 1 << 21,

        /*  */
        CUF_PRINT_URDF_INFO = 1 << 22,

        /*  */
        CUF_GOOGLEY_UNDEFINED_COLORS = 1 << 23,
    };
} // namespace beautiful_bullet

#endif // BEAUTIFUL_BULLET_LOAD_FLAGS_HPP