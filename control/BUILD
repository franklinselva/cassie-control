cc_binary(
    name = "main",
    srcs = ["main.cc"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    linkopts = ["-lpthread"],
    deps = [
        "//control/render:render",
        "@mujoco",
        "@eigen",
    ],
)
