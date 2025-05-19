"""Loads the paddlelite library"""

# Sanitize a dependency so that it works correctly from code that includes
# Apollo as a submodule.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def clean_dep(dep):
    return str(Label(dep))

def repo():
    http_archive(
        name = "centerpoint_infer_op-x86_64",
        sha256 = "038470fc2e741ebc43aefe365fc23400bc162c1b4cbb74d8c8019f84f2498190",
        strip_prefix = "centerpoint_infer_op",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/centerpoint_infer_op_cu118.tar.gz"],
    )

    http_archive(
        name = "centerpoint_infer_op-aarch64",
        sha256 = "e7c933db4237399980c5217fa6a81dff622b00e3a23f0a1deb859743f7977fc1",
        strip_prefix = "centerpoint_infer_op",
        urls = ["https://apollo-pkg-beta.bj.bcebos.com/archive/centerpoint_infer_op-linux-aarch64-1.0.0.tar.gz"],
    )