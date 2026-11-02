// "cloudflare:workers" for the self-hosted server: the Hub extends DurableObject and keeps a little state in
// ctx.storage (the read book). Here the Hub is one object in the process; its storage is a table in the same SQLite.
import type { SqliteD1 } from "../d1";

export class KvStorage {
  constructor(private d1: SqliteD1) {
    d1.db.exec("CREATE TABLE IF NOT EXISTS _server_kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)");
  }
  async get<T>(key: string): Promise<T | undefined> {
    const r = this.d1.db.prepare("SELECT value FROM _server_kv WHERE key = ?").get(key) as { value: string } | undefined;
    return r ? (JSON.parse(r.value) as T) : undefined;
  }
  async put(key: string, value: unknown): Promise<void> {
    this.d1.db
      .prepare("INSERT INTO _server_kv(key, value) VALUES (?, ?) ON CONFLICT(key) DO UPDATE SET value = excluded.value")
      .run(key, JSON.stringify(value));
  }
  async delete(key: string): Promise<boolean> {
    return Number(this.d1.db.prepare("DELETE FROM _server_kv WHERE key = ?").run(key).changes) > 0;
  }
}

export interface DoState {
  storage: KvStorage;
  id: { toString(): string; name?: string };
  waitUntil(p: Promise<unknown>): void;
  blockConcurrencyWhile<T>(fn: () => Promise<T>): Promise<T>;
}

export class DurableObject<Env = unknown> {
  constructor(
    public ctx: DoState,
    public env: Env,
  ) {}
}

export class WorkerEntrypoint<Env = unknown> {
  constructor(
    public ctx: unknown,
    public env: Env,
  ) {}
}
