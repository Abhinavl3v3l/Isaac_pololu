load("//bzl:module.bzl", "isaac_component")

isaac_component(
    name = "pololu_driver",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
        "//packages/pololu/gems:maestro",
        "//packages/engine_gems/state:io",
        "@com_nvidia_isaac_engine//engine/core/tensor:universal_tensor",
        "@com_nvidia_isaac_engine//engine/gems/tensor",
        "//packages/composite/gems:parser",
        "//packages/map:kinematic_tree",
        "//packages/math/gems/kinematic_tree",
        
    ],
)



