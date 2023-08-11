cc_library(
    name = "render",
    srcs = [
        "render.cc",
        "render.hpp"
    ],
    copts = ["-Iexternal/mujoco/include", "-Iexternal/glfw/include"],
    hdrs = ["render.hpp"],
    deps = ["@mujoco", "@glfw"],
    data = ["//assets:assets"],
    includes = ["external/mujoco/include/mujoco", "external/glfw/include"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "render_model",
    srcs = ["main.cc"],
    copts = ["-Iexternal/mujoco/include", "-Iexternal/glfw/include"],
    linkopts = ["-lpthread"],
    deps = [":render", "@mujoco", "@glfw"],
    visibility = ["//visibility:public"],
)
