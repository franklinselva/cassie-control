load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Mujoco
http_archive(
    name = "mujoco",
    url = "https://github.com/deepmind/mujoco/archive/refs/tags/2.3.7.zip",
    strip_prefix = "mujoco-2.3.7",
    build_file = "//:third_party/mujoco.BUILD",
    patch_cmds = [
        "mkdir build",
        "cd build && cmake ..",
        "cd build && make -j8",
    ]
)

# Glfw
http_archive(
    name = "glfw",
    url = "https://github.com/glfw/glfw/archive/refs/tags/3.3.8.zip",
    strip_prefix = "glfw-3.3.8",
    build_file = "//:third_party/glfw.BUILD",
    # patch_cmds = [
    #     "mkdir build",
    #     "cd build && cmake ..",
    #     "cd build && make -j8",
    # ]
)
# Eigen
# http_archive(
#     name = "eigen",
#     url = "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip",
#     strip_prefix = "eigen-3.4.0",
#     build_file = "//:third_party/eigen.BUILD",
# )

# GTest
http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip",
    strip_prefix = "googletest-release-1.8.1",
    build_file = "//:third_party/gtest.BUILD",
)
