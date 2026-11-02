// What every job must say about itself (purpose, estimate, submitter, campaign), and the scrubber that
// keeps secrets and internal paths out of the public feed. Pure functions, unit-tested directly.
import { HttpError } from "./util";

export const PURPOSE_MIN = 20;
export const PURPOSE_MAX = 600;
export const EST_MAX_S = 7 * 24 * 3600;
const SUBMITTER = /^[A-Za-z0-9][A-Za-z0-9._@:/+ -]{0,63}$/;
const CAMPAIGN = /^[A-Za-z0-9][A-Za-z0-9._:/#+ -]{0,99}$/;

// Purposes that say nothing: a placeholder, a bare word, or the template default left as is.
const GENERIC = new Set(
  [
    "test", "testing", "a test", "test run", "run", "runs", "job", "jobs", "sweep", "a sweep", "experiment", "todo",
    "tbd", "n/a", "na", "none", "purpose", "no purpose", "legacy", "(legacy)", "debug", "debugging", "check",
    "trying something", "try", "try it", "just testing", "quick test", "sanity check", "smoke test", "benchmark",
    "run the benchmark", "run the sweep", "run the job", "run some seeds", "see what happens", "whatever",
  ].map((s) => s.replace(/[^a-z0-9]+/g, " ").trim()),
);

/** A purpose (1-2 sentences by the submitting agent) or a 400 saying what is wrong with it. */
export function checkPurpose(v: unknown, field = "purpose"): string {
  if (typeof v !== "string") throw new HttpError(400, `${field} required: 1-2 sentences on what this run tests or is for`);
  const s = v.replace(/\s+/g, " ").trim();
  if (s.length < PURPOSE_MIN) throw new HttpError(400, `${field} too short (${s.length} < ${PURPOSE_MIN} chars): say what hypothesis it tests or what it is for`);
  if (s.length > PURPOSE_MAX) throw new HttpError(400, `${field} too long (at most ${PURPOSE_MAX} chars, 1-2 sentences)`);
  const norm = s.toLowerCase().replace(/[^a-z0-9]+/g, " ").trim();
  const words = norm.split(" ").filter((w) => /[a-z]/.test(w));
  if (GENERIC.has(norm) || words.length < 4 || new Set(words).size < 3)
    throw new HttpError(400, `${field} is generic: say what hypothesis it tests or what it is for`);
  if (scrub(s) !== s) throw new HttpError(400, `${field} looks like it holds a secret, an address or a local path; the feed is public`);
  return s;
}

/** Seconds per run (per seed), as the submitter estimates it. */
export function checkEst(v: unknown, field = "est_seconds"): number {
  if (typeof v !== "number" || !Number.isFinite(v) || v <= 0 || v > EST_MAX_S)
    throw new HttpError(400, `${field} required: the estimated seconds per run (per seed), > 0 and at most ${EST_MAX_S}`);
  return v;
}

export function checkSubmitter(v: unknown): string {
  if (typeof v !== "string" || !SUBMITTER.test(v.trim()))
    throw new HttpError(400, "submitter required: the agent or session label (letters, digits, ._@:/+- and spaces, at most 64)");
  return v.trim();
}

export function checkCampaign(v: unknown): string | null {
  if (v === undefined || v === null || v === "") return null;
  if (typeof v !== "string" || !CAMPAIGN.test(v.trim())) throw new HttpError(400, "campaign must be a short label (at most 100 chars)");
  return v.trim();
}

// ------------------------------------------------------------------ scrubber

const RULES: [RegExp, string][] = [
  [/-----BEGIN [A-Z ]*PRIVATE KEY-----[\s\S]*?(-----END [A-Z ]*PRIVATE KEY-----|$)/g, "[secret]"],
  [/\b(bearer|basic)\s+[A-Za-z0-9._~+/=-]{8,}/gi, "$1 [secret]"],
  [/\b(api[_-]?key|apikey|access[_-]?key|secret|token|passw(or)?d|pwd|auth|credential|private[_-]?key)([A-Za-z0-9_-]*)\s*[:=]\s*("[^"]*"|'[^']*'|\S+)/gi, "$1$3=[secret]"],
  [/\b(?:sk|pk|rk)-(?:ant-|proj-|live-|test-)?[A-Za-z0-9_-]{16,}/g, "[secret]"],
  [/\b(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|glpat-[A-Za-z0-9_-]{16,}|xox[abprs]-[A-Za-z0-9-]{10,})/g, "[secret]"],
  [/\b(?:AKIA|ASIA)[A-Z0-9]{16}\b/g, "[secret]"],
  [/\bAIza[0-9A-Za-z_-]{30,}/g, "[secret]"],
  [/\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}/g, "[secret]"],
  [/\b[a-z][a-z0-9+.-]*:\/\/[^\s/:@]+:[^\s/@]+@/gi, "[url-with-credentials]@"],
  [/[?&](?:token|key|sig|signature|secret|password|access_token|X-Amz-[A-Za-z-]+)=[^\s&]+/gi, "?[secret]"],
  // Opaque random strings: 32+ chars of base64/base64url mixing upper, lower and digits (not a plain hex hash).
  [/(?<![A-Za-z0-9_\-+/])(?=[A-Za-z0-9_\-+/]*[A-Z])(?=[A-Za-z0-9_\-+/]*[a-z])(?=[A-Za-z0-9_\-+/]*[0-9])[A-Za-z0-9_\-+/]{32,}={0,2}(?![A-Za-z0-9_\-+/])/g, "[secret]"],
  [/[\w.+-]+@[\w-]+(\.[\w-]+)+/g, "[email]"],
  // Private network addresses.
  [/\b(?:10|127)\.\d{1,3}\.\d{1,3}\.\d{1,3}\b|\b192\.168\.\d{1,3}\.\d{1,3}\b|\b172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3}\b|\b100\.(?:6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.\d{1,3}\.\d{1,3}\b/g, "[addr]"],
  // Local paths: home and system folders, Windows drives, ~/.
  [/(?:~|\$HOME|%USERPROFILE%)[\\/][^\s"'`)]*/g, "[path]"],
  [/(?<![A-Za-z0-9_.])\/(?:home|Users|root|etc|var|tmp|opt|srv|mnt|media|run|proc|usr)\/[^\s"'`)]*/g, "[path]"],
  [/\b[A-Za-z]:\\[^\s"'`)]*/g, "[path]"],
];

/** Removes secrets, credentials, emails, private addresses and local paths from free text. */
export function scrub(s: string): string {
  let out = s;
  for (const [re, rep] of RULES) out = out.replace(re, rep);
  return out;
}

/** scrub() for nullable text, cut to max chars. */
export function scrubText(s: string | null | undefined, max = 600): string | null {
  if (s === null || s === undefined) return null;
  const t = scrub(String(s)).replace(/\s+/g, " ").trim();
  return t.length > max ? t.slice(0, max - 1) + "…" : t;
}

/** A worker name as a public alias: lower case, [a-z0-9-], at most 24 chars. */
export function alias(name: string | null | undefined): string | null {
  if (!name) return null;
  const host = name.toLowerCase().split("@").pop() ?? "";
  const a = host.split(".")[0];
  const clean = a.replace(/[^a-z0-9-]+/g, "-").replace(/^-+|-+$/g, "").slice(0, 24);
  return clean || "worker";
}
