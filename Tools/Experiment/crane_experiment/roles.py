"""Role enforcement. The LLM role uses the same API as people with fewer rights."""
from .errors import RoleError

ALL = {"create", "edit_spec", "ask_train", "ask_heldout", "ask_real", "evaluate", "tell",
       "freeze", "report", "status", "claims_check", "verify", "edit_ledger",
       "edit_prereg", "compare"}

PERMISSIONS = {
    "human": ALL - {"edit_ledger", "edit_prereg"},   # nobody edits the ledger or the prereg via the API
    "llm": {"create", "ask_train", "evaluate", "report", "status", "claims_check",
            "verify", "compare"},
}


def require(role, action):
    if role not in PERMISSIONS:
        raise RoleError(f"unknown role {role!r}")
    if action not in PERMISSIONS[role]:
        raise RoleError(f"role {role!r} may not {action}")
