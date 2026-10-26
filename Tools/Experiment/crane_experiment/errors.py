"""Exception types. Every refusal is a distinct, testable type."""


class StudyError(Exception):
    code = "STUDY_ERROR"


class SpecError(StudyError):
    code = "SPEC_INVALID"


class SplitError(SpecError):
    code = "SPLIT_INVALID"


class SpecFrozen(StudyError):
    code = "SPEC_FROZEN"


class SpecTampered(StudyError):
    code = "SPEC_TAMPERED"


class LedgerCorrupt(StudyError):
    code = "LEDGER_CORRUPT"


class UnknownTrial(StudyError):
    code = "UNKNOWN_TRIAL"


class DuplicateTell(StudyError):
    code = "DUPLICATE_TELL"


class BudgetExhausted(StudyError):
    code = "BUDGET_EXHAUSTED"


class HeldOutLocked(StudyError):
    code = "HELDOUT_LOCKED"


class RoleError(StudyError):
    code = "ROLE_DENIED"


class NotRun(StudyError):
    code = "NOT_RUN"
