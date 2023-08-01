cc_library(
    name = "glfw",
    srcs = glob(["build/src/*.a"]),
    hdrs = [
        "include/GLFW/glfw3.h",
        "include/GLFW/glfw3native.h",
    ],
    includes = glob(["include/GLFW/*"]),
    linkopts = ["-lX11", "-lXrandr", "-lXinerama", "-lXi", "-lXcursor", "-lrt", "-lm", "-ldl"],
    visibility = ["//visibility:public"],
)
