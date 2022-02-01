#! /usr/bin/env python
# encoding: utf-8

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
