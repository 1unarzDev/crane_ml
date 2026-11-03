// Scheduler matching: a job's needs against a worker's capabilities and policy.
// Pure functions, no I/O, so they are unit-tested directly.
import { HttpError, isObject } from "./util";

/** What a worker reports it has. Unknown numbers count as 0, unknown flags as false. */
export interface Caps {
  os?: string; // linux | windows | darwin
  arch?: string;
  cpu?: number; // logical cores available to jobs
  ram_gb?: number;
  gpu?: string | null;
  gpu_count?: number;
  vram_gb?: number;
  vulkan?: boolean;
  tags?: string[];
  backends?: string[]; // backend names registered in the worker's YAML
}

/** What a job requires. Every field is optional; an empty object matches any worker with the backend. */
export interface Needs {
  os?: string | string[];
  arch?: string | string[];
  gpu?: boolean;
  vulkan?: boolean;
  min_cpu?: number;
  min_ram_gb?: number;
  min_vram_gb?: number;
  tags?: string[]; // all required
  est_minutes?: number;
}

/** A worker's availability policy, as declared in its YAML. Time-of-day rules are applied by the worker itself. */
export interface Policy {
  max_est_minutes?: number; // refuse jobs estimated longer than this (shared machines)
  require_tags?: string[]; // only take jobs that carry at least one of these tags (reserved machines)
  quiet_hours?: string[];
  pause_on_activity?: boolean;
}

const asList = (v: string | string[] | undefined): string[] | null =>
  v === undefined ? null : (Array.isArray(v) ? v : [v]).map((s) => s.toLowerCase());

/** Returns null when the worker can run the job, otherwise the first reason it can't. */
export function mismatch(backend: string, needs: Needs, caps: Caps, policy: Policy = {}): string | null {
  if (!(caps.backends ?? []).includes(backend)) return `backend ${backend}`;
  const os = asList(needs.os);
  if (os && !os.includes((caps.os ?? "").toLowerCase())) return "os";
  const arch = asList(needs.arch);
  if (arch && !arch.includes((caps.arch ?? "").toLowerCase())) return "arch";
  const hasGpu = !!caps.gpu && (caps.gpu_count ?? 1) > 0;
  if (needs.gpu && !hasGpu) return "gpu";
  if (needs.vulkan && !caps.vulkan) return "vulkan";
  if ((needs.min_cpu ?? 0) > (caps.cpu ?? 0)) return "cpu";
  if ((needs.min_ram_gb ?? 0) > (caps.ram_gb ?? 0)) return "ram";
  if ((needs.min_vram_gb ?? 0) > (hasGpu ? (caps.vram_gb ?? 0) : 0)) return "vram";
  const tags = new Set(caps.tags ?? []);
  for (const t of needs.tags ?? []) if (!tags.has(t)) return `tag ${t}`;
  if (policy.max_est_minutes !== undefined && (needs.est_minutes ?? 0) > policy.max_est_minutes) return "policy: too long";
  if (policy.require_tags?.length && !(needs.tags ?? []).some((t) => policy.require_tags!.includes(t))) return "policy: reserved";
  return null;
}

export const matches = (backend: string, needs: Needs, caps: Caps, policy: Policy = {}): boolean =>
  mismatch(backend, needs, caps, policy) === null;

// ---------------------------------------------------------------- validation

const num = (v: unknown) => typeof v === "number" && Number.isFinite(v) && v >= 0;
const strs = (v: unknown) => Array.isArray(v) && v.every((x) => typeof x === "string" && x.length <= 64) && v.length <= 32;
const strOrStrs = (v: unknown) => (typeof v === "string" && v.length <= 64) || strs(v);

/** Validates and normalizes a needs object; throws a message string on bad input. */
export function parseNeeds(v: unknown): Needs {
  if (v === undefined || v === null) return {};
  if (!isObject(v)) throw "needs must be an object";
  const out: Needs = {};
  for (const [k, x] of Object.entries(v)) {
    switch (k) {
      case "os":
      case "arch":
        if (!strOrStrs(x)) throw `needs.${k} must be a string or list of strings`;
        out[k] = x as string | string[];
        break;
      case "gpu":
      case "vulkan":
        if (typeof x !== "boolean") throw `needs.${k} must be a boolean`;
        out[k] = x;
        break;
      case "min_cpu":
      case "min_ram_gb":
      case "min_vram_gb":
      case "est_minutes":
        if (!num(x)) throw `needs.${k} must be a non-negative number`;
        out[k] = x as number;
        break;
      case "tags":
        if (!strs(x)) throw "needs.tags must be a list of strings";
        out.tags = x as string[];
        break;
      default:
        throw `unknown needs field ${k}`;
    }
  }
  return out;
}

/** Keeps only known capability fields with the right types (worker input is untrusted). */
export function sanitizeCaps(v: unknown): Caps {
  if (!isObject(v)) return {};
  const out: Caps = {};
  const s = (x: unknown) => (typeof x === "string" ? x.slice(0, 120) : undefined);
  if (s(v.os)) out.os = s(v.os)!.toLowerCase();
  if (s(v.arch)) out.arch = s(v.arch)!.toLowerCase();
  if (typeof v.gpu === "string" || v.gpu === null) out.gpu = v.gpu === null ? null : s(v.gpu);
  for (const k of ["cpu", "ram_gb", "gpu_count", "vram_gb"] as const) if (num(v[k])) out[k] = v[k] as number;
  if (typeof v.vulkan === "boolean") out.vulkan = v.vulkan;
  if (strs(v.tags)) out.tags = v.tags as string[];
  if (strs(v.backends)) out.backends = v.backends as string[];
  return out;
}

export function sanitizePolicy(v: unknown): Policy {
  if (!isObject(v)) return {};
  const out: Policy = {};
  if (num(v.max_est_minutes)) out.max_est_minutes = v.max_est_minutes as number;
  if (strs(v.require_tags)) out.require_tags = v.require_tags as string[];
  // Informational only (the worker enforces them locally); kept for status views.
  if (strs(v.quiet_hours)) out.quiet_hours = v.quiet_hours as string[];
  if (typeof v.pause_on_activity === "boolean") out.pause_on_activity = v.pause_on_activity;
  return out;
}

/** The caps one lease call matches with. A lease body's `backends` (a multi-slot worker names the ones with a free
 * slot) narrows caps.backends for this call only; the stored caps keep every backend the worker offers. */
export function leaseCaps(caps: Caps, only: unknown): Caps {
  if (only === undefined) return caps;
  if (!strs(only)) throw new HttpError(400, "backends must be an array of backend names");
  const free = new Set(only as string[]);
  return { ...caps, backends: (caps.backends ?? []).filter((b) => free.has(b)) };
}
