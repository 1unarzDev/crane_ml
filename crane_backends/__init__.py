"""crane's backends for the research cluster (github.com/1unarzDev/research-cluster, private; was crane_ml cluster/).

    fastsim          crane_backends.fastsim:ClusterBackend    the fast-sim core per seed (CPU)
    unity            crane_backends.unity:ClusterBackend      a crane player's benchmark per seed (GPU)
    theater-render   crane_backends.theater:ClusterBackend    ghost-theater video chunks (GPU, ffmpeg)

The platform finds them through the ``research_cluster.backends`` entry points in crane_ml's pyproject.toml
when crane_ml is pip-installed (``type: fastsim``), or by ``module:Class`` with this checkout on the backend's
``path`` (what the platform's install/provision.yaml writes). They import ``fastsim`` and ``theater`` from the
crane root, and the platform's ``crane_cluster`` package, which the worker provides.
"""


def find_platform():
    """Puts the platform's crane_cluster on sys.path for the tests and tools run from crane: RESEARCH_CLUSTER (a
    research-cluster checkout), ~/research-cluster, or an installed worker's lib. Returns the folder or None."""
    import os
    import sys
    from pathlib import Path
    data = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local' / 'share'))
    for d in (os.environ.get('RESEARCH_CLUSTER'), Path.home() / 'research-cluster', data / 'crane-cluster' / 'lib',
              Path(os.environ.get('LOCALAPPDATA', '')) / 'crane-cluster' / 'lib'):
        if d and (Path(d) / 'crane_cluster' / '__init__.py').is_file():
            if str(d) not in sys.path:
                sys.path.append(str(d))
            return str(d)
    return None
