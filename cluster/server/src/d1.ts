// D1Database over a local SQLite file (node:sqlite, WAL). The coordinator's code runs unchanged on it: prepare/bind,
// first/all/run/raw, batch (one transaction, rolled back whole on error, like D1) and exec. node:sqlite is
// synchronous, so a statement or a batch never interleaves with another request's.
import { DatabaseSync, type StatementSync } from "node:sqlite";

type Row = Record<string, unknown>;
export interface Meta {
  duration: number;
  changes: number;
  last_row_id: number;
  rows_read: number;
  rows_written: number;
  changed_db: boolean;
  size_after: number;
}
export interface Result<T = Row> {
  results: T[];
  success: true;
  meta: Meta;
}

function value(v: unknown): null | number | bigint | string | Uint8Array {
  if (v === undefined || v === null) return null;
  if (typeof v === "boolean") return v ? 1 : 0;
  if (typeof v === "number" || typeof v === "bigint" || typeof v === "string") return v;
  if (v instanceof Uint8Array) return v;
  if (v instanceof ArrayBuffer) return new Uint8Array(v);
  if (ArrayBuffer.isView(v)) return new Uint8Array(v.buffer, v.byteOffset, v.byteLength);
  throw new TypeError(`D1_TYPE_ERROR: type '${typeof v}' not supported for value '${String(v)}'`);
}

export class SqliteD1 {
  readonly db: DatabaseSync;
  private cache = new Map<string, StatementSync>();
  private totalStmt: StatementSync;

  constructor(readonly path: string) {
    this.db = new DatabaseSync(path, { enableForeignKeyConstraints: true });
    if (path !== ":memory:") this.db.exec("PRAGMA journal_mode = WAL");
    this.db.exec("PRAGMA synchronous = NORMAL; PRAGMA busy_timeout = 5000; PRAGMA temp_store = MEMORY");
    this.totalStmt = this.db.prepare("SELECT total_changes() AS t, changes() AS c, last_insert_rowid() AS r");
  }

  prepare(sql: string): SqliteStatement {
    return new SqliteStatement(this, sql, []);
  }

  /** A cached compiled statement (the coordinator prepares the same few hundred strings over and over). */
  compiled(sql: string): StatementSync {
    let s = this.cache.get(sql);
    if (!s) {
      s = this.db.prepare(sql);
      if (this.cache.size > 1000) this.cache.clear();
      this.cache.set(sql, s);
    }
    return s;
  }

  /** Runs one statement; RETURNING and SELECT rows come back in results, writes count in meta.changes. */
  execute(sql: string, params: unknown[]): Result {
    const t0 = performance.now();
    const st = this.compiled(sql);
    const before = (this.totalStmt.get() as { t: number }).t;
    const args = params.map(value);
    let results: Row[];
    if (st.columns().length) results = (st.all(...args) as Row[]).map((r) => ({ ...r }));
    else {
      st.run(...args);
      results = [];
    }
    const after = this.totalStmt.get() as { t: number; c: number; r: number };
    const changes = after.t !== before ? after.c : 0;
    return {
      results,
      success: true,
      meta: {
        duration: performance.now() - t0,
        changes,
        last_row_id: Number(after.r),
        // SQLite does not expose rows scanned; the returned rows stand in (the read counter is informational here).
        rows_read: results.length,
        rows_written: changes,
        changed_db: changes > 0,
        size_after: 0,
      },
    };
  }

  async batch(stmts: SqliteStatement[]): Promise<Result[]> {
    if (!stmts.length) return [];
    if (this.db.isTransaction) return stmts.map((s) => this.execute(s.sql, s.params));
    this.db.exec("BEGIN IMMEDIATE");
    try {
      const out = stmts.map((s) => this.execute(s.sql, s.params));
      this.db.exec("COMMIT");
      return out;
    } catch (e) {
      this.db.exec("ROLLBACK");
      throw e;
    }
  }

  async exec(sql: string): Promise<{ count: number; duration: number }> {
    const t0 = performance.now();
    this.db.exec(sql);
    return { count: sql.split(";").filter((s) => s.trim()).length, duration: performance.now() - t0 };
  }

  async dump(): Promise<ArrayBuffer> {
    throw new Error("dump() is not supported; use the nightly backup (VACUUM INTO)");
  }

  withSession() {
    return this;
  }

  close() {
    this.cache.clear();
    this.db.close();
  }
}

export class SqliteStatement {
  constructor(
    private d1: SqliteD1,
    readonly sql: string,
    readonly params: unknown[],
  ) {}

  bind(...values: unknown[]): SqliteStatement {
    return new SqliteStatement(this.d1, this.sql, values);
  }

  async all<T = Row>(): Promise<Result<T>> {
    return this.d1.execute(this.sql, this.params) as Result<T>;
  }

  async run<T = Row>(): Promise<Result<T>> {
    return this.d1.execute(this.sql, this.params) as Result<T>;
  }

  async first<T = unknown>(col?: string): Promise<T | null> {
    const row = this.d1.execute(this.sql, this.params).results[0];
    if (!row) return null;
    if (col === undefined) return row as T;
    if (!(col in row)) throw new Error(`D1_COLUMN_NOTFOUND: Column not found (${col})`);
    return row[col] as T;
  }

  async raw<T = unknown[]>(opts?: { columnNames?: boolean }): Promise<T[]> {
    const st = this.d1.compiled(this.sql);
    const { results } = this.d1.execute(this.sql, this.params);
    const names = st.columns().map((c) => c.name);
    const rows = results.map((r) => names.map((n) => r[n])) as T[];
    return opts?.columnNames ? [names as T, ...rows] : rows;
  }
}

// ---------------------------------------------------------------- migrations (wrangler's d1_migrations table)

export interface Migration {
  name: string;
  sql: string;
}

/** Applies the migrations not yet in d1_migrations, in order (a D1 export carries that table, so an import resumes). */
export function applyMigrations(d1: SqliteD1, migrations: Migration[]): string[] {
  d1.db.exec(`CREATE TABLE IF NOT EXISTS d1_migrations(
    id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT UNIQUE, applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL)`);
  const done = new Set((d1.db.prepare("SELECT name FROM d1_migrations").all() as { name: string }[]).map((r) => r.name));
  const applied: string[] = [];
  for (const m of migrations) {
    if (done.has(m.name)) continue;
    d1.db.exec("BEGIN");
    try {
      d1.db.exec(m.sql);
      d1.db.prepare("INSERT INTO d1_migrations(name) VALUES (?)").run(m.name);
      d1.db.exec("COMMIT");
    } catch (e) {
      d1.db.exec("ROLLBACK");
      throw new Error(`migration ${m.name}: ${String(e)}`);
    }
    applied.push(m.name);
  }
  return applied;
}
