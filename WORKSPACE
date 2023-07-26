load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Mujoco
http_archive(
    name = "mujoco",
    url = "https://github.com/deepmind/mujoco/archive/refs/tags/2.3.7.zip",
    strip_prefix = "mujoco-2.3.7",
    build_file = "//:third_party/mujoco.BUILD",
)

# Glfw
http_archive(
    name = "glfw",
    url = "https://github.com/glfw/glfw/archive/refs/tags/3.3.8.zip",
    strip_prefix = "glfw-3.3.8",
    build_file = "//:third_party/glfw.BUILD",
)

# GTest
http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip",
    strip_prefix = "googletest-release-1.8.1",
    build_file = "//:third_party/gtest.BUILD",
)

http_archive(
    name = "rules_foreign_cc",
    strip_prefix = "rules_foreign_cc-0.8.0",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.8.0.tar.gz",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()
