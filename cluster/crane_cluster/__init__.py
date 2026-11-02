"""crane cluster: a domain-agnostic job cluster for staged simulation experiments.

The coordinator (a Cloudflare Worker in ../coordinator) hands out jobs; this
package holds the worker daemon, the backend interface, the promotion analysis
and the `crane-hyp` / `crane-worker` command lines.
"""
__version__ = "0.1.0"
