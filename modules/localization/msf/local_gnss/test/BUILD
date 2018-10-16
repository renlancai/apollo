load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "localization_local_gnss_test",
    size = "medium",
    timeout = "short",
    srcs = glob([
        "*.cc",
    ]),
    copts = [
        "-Dprivate=public",
    ],
    deps = [
        "//modules/localization/msf/local_gnss:localization_msf_local_gnss",
        "//modules/localization/proto:gnss_pnt_result_proto",
        "//modules/drivers/gnss/proto:gnss_raw_observation_proto",
        "@gtest//:main",
    ],
)

cpplint()
