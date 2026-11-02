// The Hub's stub, apart from src/hub.ts so route modules reach it without importing the Hub (an import cycle).
import type { Env } from "./util";

export const hubStub = (env: Env) => env.HUB.get(env.HUB.idFromName("hub"));
