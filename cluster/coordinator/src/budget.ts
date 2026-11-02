// D1 read budget: every query's meta.rows_read is counted per route and UTC hour. The Worker's isolates tally
// locally and push their tallies to the Hub (at most once a minute each); the Hub keeps the day's book,
// serves GET /v1/admin/reads, and turns on the guard once the day passes D1_GUARD_FRAC of D1_READ_BUDGET
// (the free plan's 5M rows read/day, which the account shares with other projects).

export interface Count {
  r: number; // rows read
  w: number; // rows written
  q: number; // queries
}
/** hour ("2026-11-02T21") -> label ("POST /v1/lease") -> count */
export type Book = Record<string, Record<string, Count>>;

export const hourKey = (ms = Date.now()) => new Date(ms).toISOString().slice(0, 13);
export const dayKey = (ms = Date.now()) => new Date(ms).toISOString().slice(0, 10);

export function addTo(book: Book, hour: string, label: string, r: number, w: number, q = 1) {
  const h = (book[hour] ??= {});
  const c = (h[label] ??= { r: 0, w: 0, q: 0 });
  c.r += r;
  c.w += w;
  c.q += q;
}

export function mergeBook(into: Book, from: Book) {
  for (const [h, labels] of Object.entries(from)) for (const [l, c] of Object.entries(labels)) addTo(into, h, l, c.r, c.w, c.q);
}

type Meta = { rows_read?: number; rows_written?: number } | undefined;
type Sink = (label: string, meta: Meta) => void;

/** A D1PreparedStatement that reports each result's meta to the sink. first() and raw() go through all()/raw(). */
class CountedStatement {
  constructor(
    readonly inner: D1PreparedStatement,
    private label: () => string,
    private sink: Sink,
  ) {}
  bind(...values: unknown[]) {
    return new CountedStatement(this.inner.bind(...values), this.label, this.sink);
  }
  async all<T = Record<string, unknown>>(): Promise<D1Result<T>> {
    const r = await this.inner.all<T>();
    this.sink(this.label(), r.meta);
    return r;
  }
  async run<T = Record<string, unknown>>(): Promise<D1Result<T>> {
    const r = await this.inner.run<T>();
    this.sink(this.label(), r.meta);
    return r;
  }
  // D1's first() runs the whole statement too (it does not add a LIMIT), so all() costs the same rows.
  async first<T = unknown>(col?: string): Promise<T | null> {
    const r = await this.all<Record<string, unknown>>();
    const row = r.results[0];
    if (!row) return null;
    return (col === undefined ? row : (row[col] ?? null)) as T;
  }
  async raw<T = unknown[]>(opts?: { columnNames?: boolean }): Promise<T[]> {
    this.sink(this.label(), undefined); // raw() carries no meta: counted as a query only
    return (this.inner.raw as (o?: { columnNames?: boolean }) => Promise<T[]>)(opts);
  }
}

/** Wraps a D1 binding so every statement's rows read and written land in the sink under label(). */
export function countingDb(db: D1Database, label: () => string, sink: Sink): D1Database {
  const wrapped = {
    prepare: (q: string) => new CountedStatement(db.prepare(q), label, sink),
    batch: async (stmts: (CountedStatement | D1PreparedStatement)[]) => {
      const inner = stmts.map((s) => (s instanceof CountedStatement ? s.inner : s));
      const res = await db.batch(inner);
      for (const r of res) sink(label(), r.meta);
      return res;
    },
    exec: (q: string) => db.exec(q),
    dump: () => (db as unknown as { dump: () => Promise<ArrayBuffer> }).dump(),
    withSession: (c?: string) => db.withSession(c as never),
  };
  return wrapped as unknown as D1Database;
}

/** A sink that tallies into a Book. */
export function bookSink(book: Book): Sink {
  return (label, meta) => addTo(book, hourKey(), label, meta?.rows_read ?? 0, meta?.rows_written ?? 0);
}

export const bookReads = (book: Book) => Object.values(book).reduce((s, l) => s + Object.values(l).reduce((t, c) => t + c.r, 0), 0);
