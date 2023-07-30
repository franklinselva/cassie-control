load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Mujoco
http_archive(
    name = "mujoco",
    build_file = "//:third_party/mujoco.BUILD",
    patch_cmds = [
        "mkdir build",
        "cd build && cmake ..",
        "cd build && make -j 4",
    ],
    sha256 = "2a3d1757b084cf015cc680bed9e3ab9ca8fe1568bbede3d3112d87133efbb636",
    strip_prefix = "mujoco-2.3.7",
    url = "https://github.com/deepmind/mujoco/archive/refs/tags/2.3.7.zip",
)

# Glfw
http_archive(
    name = "glfw",
    build_file = "//:third_party/glfw.BUILD",
    patch_cmds = [
        "mkdir build",
        "cd build && cmake ..",
        "cd build && make -j 4",
    ],
    sha256 = "8106e1a432305a8780b986c24922380df6a009a96b2ca590392cb0859062c8ff",
    strip_prefix = "glfw-3.3.8",
    url = "https://github.com/glfw/glfw/archive/refs/tags/3.3.8.zip",
)

# GTest
http_archive(
    name = "gtest",
    build_file = "//:third_party/gtest.BUILD",
    strip_prefix = "googletest-release-1.8.1",
    url = "https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip",
)

http_archive(
    name = "rules_foreign_cc",
    sha256 = "6041f1374ff32ba711564374ad8e007aef77f71561a7ce784123b9b4b88614fc",
    strip_prefix = "rules_foreign_cc-0.8.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.8.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()
