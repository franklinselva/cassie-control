load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

cmake(
    name = "mujoco",
    cache_entries = {
        "CMAKE_BUILD_TYPE": "Release",
    },
    lib_source = "@mujoco//:all_srcs",
    out_static_libs = ["libmujoco.so"],
)