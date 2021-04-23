#! /usr/bin/env python
# encoding: utf-8

from waflib.Configure import conf
from utils import check_include, check_lib


def options(opt):
    # Required package options
    opt.load("eigen bullet", tooldir="waf_tools")

    # Options
    opt.add_option(
        "--robotbullet-path", type="string", help="path to robot-bullet", dest="robotbullet_path"
    )


@conf
def check_robotbullet(ctx):
    # Set the search path
    if ctx.options.robotbullet_path is None:
        path_check = ["/usr/local", "/usr"]
    else:
        path_check = [ctx.options.robotbullet_path]

    # robot-bullet includes
    check_include(ctx, "ROBOTBULLET", ["robot_bullet"], [
                  "Simulator.hpp"], path_check)

    # robot-bullet libs
    check_lib(ctx, "ROBOTBULLET", "", ["libRobotBullet"], path_check)

    if ctx.env.LIB_ROBOTBULLET or ctx.env.STLIB_ROBOTBULLET:
        # Add dependencies to require libraries
        ctx.get_env()["requires"] = ctx.get_env()[
            "requires"] + ["EIGEN", "BULLET"]

        # Check for dependencies
        ctx.load("eigen bullet", tooldir="waf_tools")

        # Add library
        ctx.get_env()["libs"] = ctx.get_env()["libs"] + ["ROBOTBULLET"]


def configure(cfg):
    if not cfg.env.LIB_ROBOTBULLET and not cfg.env.STLIB_ROBOTBULLET:
        cfg.check_robotbullet()
