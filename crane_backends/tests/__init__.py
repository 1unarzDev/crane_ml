import unittest

from crane_backends import find_platform

try:
    import crane_cluster  # noqa: F401
except ImportError:
    if find_platform() is None:
        raise unittest.SkipTest('no research-cluster checkout (RESEARCH_CLUSTER, ~/research-cluster) or installed worker')
