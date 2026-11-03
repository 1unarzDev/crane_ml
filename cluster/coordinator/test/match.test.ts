import { describe, expect, it } from "vitest";
import { mismatch, parseNeeds, sanitizeCaps, sanitizePolicy } from "../src/match";

const cpu = { os: "linux", arch: "x86_64", cpu: 8, ram_gb: 16, gpu: null, vulkan: false, tags: ["lab"], backends: ["dummy", "fast"] };
const gpu = { ...cpu, os: "windows", gpu: "rtx 4070", gpu_count: 1, vram_gb: 12, vulkan: true, backends: ["dummy", "render"] };

describe("scheduler matching", () => {
  it("empty needs match any worker that has the backend", () => {
    expect(mismatch("dummy", {}, cpu)).toBeNull();
    expect(mismatch("render", {}, cpu)).toBe("backend render");
    expect(mismatch("dummy", {}, {})).toBe("backend dummy");
  });

  it("gpu, vulkan and vram needs only match GPU workers", () => {
    expect(mismatch("dummy", { gpu: true }, cpu)).toBe("gpu");
    expect(mismatch("dummy", { vulkan: true }, cpu)).toBe("vulkan");
    expect(mismatch("dummy", { gpu: true, vulkan: true, min_vram_gb: 8 }, gpu)).toBeNull();
    expect(mismatch("dummy", { min_vram_gb: 16 }, gpu)).toBe("vram");
    // vram reported without a GPU does not count
    expect(mismatch("dummy", { min_vram_gb: 1 }, { ...cpu, vram_gb: 24 })).toBe("vram");
    expect(mismatch("dummy", { gpu: true }, { ...gpu, gpu_count: 0 })).toBe("gpu");
  });

  it("os and arch accept one value or a list, case-insensitively", () => {
    expect(mismatch("dummy", { os: "linux" }, cpu)).toBeNull();
    expect(mismatch("dummy", { os: ["Linux", "darwin"] }, cpu)).toBeNull();
    expect(mismatch("dummy", { os: "linux" }, gpu)).toBe("os");
    expect(mismatch("dummy", { arch: "arm64" }, cpu)).toBe("arch");
  });

  it("cpu, ram and tags are minimums / all-required", () => {
    expect(mismatch("dummy", { min_cpu: 8, min_ram_gb: 16 }, cpu)).toBeNull();
    expect(mismatch("dummy", { min_cpu: 9 }, cpu)).toBe("cpu");
    expect(mismatch("dummy", { min_ram_gb: 32 }, cpu)).toBe("ram");
    expect(mismatch("dummy", { tags: ["lab"] }, cpu)).toBeNull();
    expect(mismatch("dummy", { tags: ["lab", "x"] }, cpu)).toBe("tag x");
  });

  it("availability policy: length cap and reserved machines", () => {
    expect(mismatch("dummy", { est_minutes: 90 }, cpu, { max_est_minutes: 60 })).toBe("policy: too long");
    expect(mismatch("dummy", { est_minutes: 30 }, cpu, { max_est_minutes: 60 })).toBeNull();
    expect(mismatch("dummy", {}, cpu, { require_tags: ["lab"] })).toBe("policy: reserved");
    expect(mismatch("dummy", { tags: ["lab"] }, cpu, { require_tags: ["lab"] })).toBeNull();
  });

  it("parseNeeds rejects unknown fields and wrong types", () => {
    expect(parseNeeds({ gpu: true, os: ["linux"], min_ram_gb: 4 })).toEqual({ gpu: true, os: ["linux"], min_ram_gb: 4 });
    expect(() => parseNeeds({ gpus: 1 })).toThrow();
    expect(() => parseNeeds({ gpu: "yes" })).toThrow();
    expect(() => parseNeeds({ min_cpu: -1 })).toThrow();
    expect(parseNeeds(undefined)).toEqual({});
  });

  it("sanitizeCaps keeps known, well-typed fields only", () => {
    expect(sanitizeCaps({ os: "Linux", cpu: 4, vulkan: "yes", evil: 1, backends: ["dummy"] })).toEqual({ os: "linux", cpu: 4, backends: ["dummy"] });
  });
});

describe("a GPU runner's window", () => {
  it("matches only while its window is open, and keeps a valid window from the worker", () => {
    const open = { window: { enabled: true, open: true, until: "2026-11-03T08:00-06:00" } };
    expect(mismatch("render", {}, gpu, open)).toBeNull();
    expect(mismatch("render", {}, gpu, { window: { enabled: true, open: false } })).toBe("policy: outside its GPU window");
    expect(mismatch("render", {}, gpu, { window: { enabled: false, open: true } })).toBe("policy: outside its GPU window");
    const p = sanitizePolicy({ window: { open: false, next_at: "2026-11-03T01:00-06:00", until: "soon", waiting: "x".repeat(200), extra: 1 } });
    expect(p.window).toEqual({ enabled: true, open: false, next_at: "2026-11-03T01:00-06:00", waiting: "x".repeat(80) });
    expect(sanitizePolicy({ window: { next_at: "2026-11-03T01:00-06:00" } }).window).toBeUndefined();
  });
});
