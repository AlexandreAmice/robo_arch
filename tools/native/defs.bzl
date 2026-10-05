"""Declare platform wheels from one explicit component/resource inventory."""

def native_wheels(name, extensions, resources = {}):
    """Keep wheel assembly generic; component targets own compilation."""
    inputs = extensions.values() + resources.values()
    arguments = " ".join([
        "--extension {} $(execpath {})".format(module, label)
        for module, label in extensions.items()
    ] + [
        "--resource {} $(execpath {})".format(path, label)
        for path, label in resources.items()
    ])
    for suffix, python, platform, constraints in [
        ("linux", "3.12", "linux_x86_64", ["@platforms//os:linux", "@platforms//cpu:x86_64"]),
        ("macos", "3.13", "macosx_15_0_arm64", ["@platforms//os:macos", "@platforms//cpu:aarch64"]),
    ]:
        tag = "cp" + python.replace(".", "")
        native.genrule(
            name = name + "_" + suffix,
            srcs = inputs,
            outs = ["robo_arch_native-0.1.0-{}-{}-{}.whl".format(tag, tag, platform)],
            cmd = "$(execpath :wheel_builder) $@ --python {} --platform {} {}".format(python, platform, arguments),
            target_compatible_with = constraints,
            tools = [":wheel_builder"],
        )
    native.alias(
        name = name,
        actual = select({
            ":macos_arm64": ":" + name + "_macos",
            "//conditions:default": ":" + name + "_linux",
        }),
        visibility = ["//visibility:public"],
    )
