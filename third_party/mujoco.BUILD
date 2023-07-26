cc_library(
    name = "mujoco",
    hdrs = glob(["include/*.h"]),
    srcs = glob(["build/lib/*.so"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    alwayslink = 1,
)