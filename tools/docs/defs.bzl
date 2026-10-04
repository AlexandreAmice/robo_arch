"""Extract selected public comments with the project's C++ toolchain."""

load("@rules_cc//cc:action_names.bzl", "ACTION_NAMES")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cpp_toolchain", "use_cc_toolchain")
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")

def _cpp_docstrings_impl(ctx):
    toolchain = find_cpp_toolchain(ctx)
    features = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compilation = ctx.attr.library[CcInfo].compilation_context
    variables = cc_common.create_compile_variables(
        cc_toolchain = toolchain,
        feature_configuration = features,
        user_compile_flags = ctx.fragments.cpp.cxxopts,
        include_directories = compilation.includes,
        quote_include_directories = compilation.quote_includes,
        system_include_directories = compilation.system_includes,
        preprocessor_defines = compilation.defines,
    )
    flags = cc_common.get_memory_inefficient_command_line(
        feature_configuration = features,
        action_name = ACTION_NAMES.cpp_compile,
        variables = variables,
    )
    compiler = cc_common.get_tool_for_action(
        feature_configuration = features,
        action_name = ACTION_NAMES.cpp_compile,
    )
    args = ctx.actions.args()
    args.add_all(["--compiler", compiler, "--header", ctx.file.header.path])
    args.add_all(["--manifest", ctx.file.manifest.path])
    args.add_all(["--cpp-output", ctx.outputs.cpp.path])
    args.add_all(["--python-output", ctx.outputs.python.path])
    args.add("--")
    args.add_all(flags)
    ctx.actions.run(
        executable = ctx.executable._extract,
        arguments = [args],
        inputs = depset(
            [ctx.file.header, ctx.file.manifest],
            transitive = [compilation.headers, toolchain.all_files],
        ),
        outputs = [ctx.outputs.cpp, ctx.outputs.python],
        mnemonic = "CppDocstrings",
    )
    return [DefaultInfo(files = depset([ctx.outputs.cpp, ctx.outputs.python]))]

cpp_docstrings = rule(
    implementation = _cpp_docstrings_impl,
    attrs = {
        "library": attr.label(mandatory = True, providers = [CcInfo]),
        "header": attr.label(mandatory = True, allow_single_file = [".h"]),
        "manifest": attr.label(mandatory = True, allow_single_file = [".json"]),
        "cpp": attr.output(mandatory = True),
        "python": attr.output(mandatory = True),
        "_extract": attr.label(
            default = "//tools/docs:extract",
            executable = True,
            cfg = "exec",
        ),
    },
    fragments = ["cpp"],
    toolchains = use_cc_toolchain(),
)
