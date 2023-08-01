cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    linkopts = ["-lpthread"],
    deps = [
        "//control/render:render",
        "@mujoco",
        "@eigen",
    ],
)
