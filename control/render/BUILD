cc_library(
    name = "render",
    srcs = [
        "render.cpp",
        "render.hpp"
    ],
    copts = ["-Iexternal/mujoco/include", "-Iexternal/glfw/include"],
    hdrs = ["render.hpp"],
    deps = ["@mujoco", "@glfw"],
    includes = ["external/mujoco/include/mujoco", "external/glfw/include"],
    visibility = ["//visibility:public"],
)