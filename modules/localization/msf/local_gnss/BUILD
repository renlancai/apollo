load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_local_gnss",
    srcs = glob(["*.cc",]),
    hdrs = glob(["*.h","*.hpp",]),
    linkopts = [
#        "-lboost_system",
    ],
    deps = [
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_proto",
        "@eigen//:eigen",
    ],
)

cpplint()
