cc_library(
    name = "mujoco",
    srcs = glob(["build/lib/*"]),
    hdrs = glob(["include/mujoco/*"]),
    includes = glob(["include/mujoco/*"]),
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "mujoco-simulate",
    srcs = ["build/bin/simulate"],
    visibility = ["//visibility:public"],
    deps = [
        ":mujoco",
    ],
)
