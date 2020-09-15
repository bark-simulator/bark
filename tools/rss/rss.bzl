load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")

# Scripts were created follow the build steps in:
# ad-rss-lib: https://intel.github.io/ad-rss-lib/BUILDING/index.html
# map_support: https://ad-map-access.readthedocs.io/en/latest/BUILDING/index.html

def rss_dependencies():
    _maybe(
    http_archive,
    name = "ad_rss_lib",
    build_file = "@bark_project//tools/rss:rss.BUILD",
    sha256 = "2f86697f5cd7729b2025bb6fa67928249137c3245fe647d115d9decf5f9e240a",
    strip_prefix = "ad-rss-lib-4.1.0",
    urls = ["https://github.com/intel/ad-rss-lib/archive/v4.1.0.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "spdlog",
    build_file = "@bark_project//tools/rss:spdlog.BUILD",
    sha256 = "f0114a4d3c88be9e696762f37a7c379619443ce9d668546c61b21d41affe5b62",
    strip_prefix = "spdlog-1.7.0",
    urls = ["https://github.com/gabime/spdlog/archive/v1.7.0.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "map_support",
    build_file = "@bark_project//tools/rss:map_support.BUILD",
    sha256 = "2f2ea3da842ad599c6f70b8976287eb52c994e277626f545dbf7d5c51f0595ca",
    strip_prefix = "map-2.2.1",
    urls = ["https://github.com/carla-simulator/map/archive/v2.2.1.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "pugixml",
    build_file = "@bark_project//tools/rss:pugixml.BUILD",
    sha256 = "55f399fbb470942410d348584dc953bcaec926415d3462f471ef350f29b5870a",
    strip_prefix = "pugixml-1.10",
    urls = ["https://github.com/zeux/pugixml/releases/download/v1.10/pugixml-1.10.tar.gz"],
    )
    
    _maybe(
    http_archive,
    name = "proj",
    build_file = "@bark_project//tools/rss:proj.BUILD",
    sha256 = "876151e2279346f6bdbc63bd59790b48733496a957bccd5e51b640fdd26eaa8d",
    strip_prefix = "proj-7.1.0",
    urls = ["https://download.osgeo.org/proj/proj-7.1.0.tar.gz"],
    )


def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)

