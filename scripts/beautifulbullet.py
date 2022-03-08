#!/usr/bin/env python
# encoding: utf-8
#
#    This file is part of beautiful-bullet.
#
#    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>
#
#    Permission is hereby granted, free of charge, to any person obtaining a copy
#    of this software and associated documentation files (the "Software"), to deal
#    in the Software without restriction, including without limitation the rights
#    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#    copies of the Software, and to permit persons to whom the Software is
#    furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in all
#    copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#    SOFTWARE.

from waflib.Configure import conf
from utils import check_include, check_lib


def options(opt):
    # Required package options
    opt.load("eigen bullet", tooldir="waf_tools")

    # Options
    opt.add_option("--bb-path", type="string",
                   help="Path to Beautiful Bullet.", dest="bb_path")

    opt.add_option("--bb-pinocchio", action="store_true",
                   help="Activate Pinocchio support.", dest="bb_pinocchio")


@conf
def check_beautifulbullet(ctx):
    # Set the search path
    if ctx.options.bb_path is None:
        path_check = ["/usr/local", "/usr"]
    else:
        path_check = [ctx.options.bb_path]

    # beautiful-bullet includes
    check_include(ctx, "BEAUTIFULBULLET", ["beautiful_bullet"], [
                  "Simulator.hpp"], path_check)

    # beautiful-bullet libs
    check_lib(ctx, "BEAUTIFULBULLET", "", ["libBeautifulBullet"], path_check)

    if ctx.env.LIB_BEAUTIFULBULLET or ctx.env.STLIB_BEAUTIFULBULLET:
        # Add dependencies to require libraries
        ctx.get_env()["requires"] = ctx.get_env()[
            "requires"] + ["EIGEN", "BULLET"]

        # Check for dependencies
        ctx.load("eigen bullet", tooldir="waf_tools")

        # Add library
        ctx.get_env()["libs"] = ctx.get_env()["libs"] + ["BEAUTIFULBULLET"]


def configure(cfg):
    if not cfg.env.LIB_BEAUTIFULBULLET and not cfg.env.STLIB_BEAUTIFULBULLET:
        cfg.check_beautifulbullet()
