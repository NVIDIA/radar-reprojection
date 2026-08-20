def _impl(ctx):
    prefix = ctx.os.environ.get(ctx.attr.conda_prefix_env)
    if not prefix:
        fail("CONDA_PREFIX env var is not set; activate the radar_reprojection conda env.")
    ctx.symlink(prefix + "/include/eigen3", "include")
    ctx.file("BUILD.bazel", """
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "eigen",
    hdrs = glob(["include/Eigen/**", "include/unsupported/Eigen/**"], allow_empty = True),
    includes = ["include"],
)
""")

eigen_local = repository_rule(
    implementation = _impl,
    attrs = {"conda_prefix_env": attr.string(default = "CONDA_PREFIX")},
    environ = ["CONDA_PREFIX"],
)
