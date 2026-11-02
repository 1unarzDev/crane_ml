# crane-server: the coordinator self-hosted (see README.md). Build from the repository root:
#   docker build -f cluster/server/Dockerfile -t crane-server:local .
# The build applies hub.patch to coordinator/src, bundles with esbuild and runs the whole test suite (the Worker's
# own vitest suites against this runtime); RUN_TESTS=0 skips them. The runtime image is node + dist/ only.
FROM node:26-slim AS build
ARG RUN_TESTS=1
RUN apt-get update && apt-get install -y --no-install-recommends patch && rm -rf /var/lib/apt/lists/*
WORKDIR /src/cluster/server
COPY cluster/server/package.json cluster/server/package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY cluster/coordinator/src /src/cluster/coordinator/src
COPY cluster/coordinator/test /src/cluster/coordinator/test
COPY cluster/coordinator/migrations /src/cluster/coordinator/migrations
COPY cluster/coordinator/public /src/cluster/coordinator/public
COPY cluster/coordinator/wrangler.toml /src/cluster/coordinator/wrangler.toml
COPY theater/tests/golden /src/theater/tests/golden
COPY cluster/server/hub.patch cluster/server/vitest.config.ts ./
COPY cluster/server/scripts ./scripts
COPY cluster/server/src ./src
COPY cluster/server/test ./test
RUN node scripts/build.mjs && if [ "$RUN_TESTS" = 1 ]; then npx vitest run && CRANE_TEST_HTTP=1 npx vitest run; fi

FROM node:26-slim
ENV NODE_ENV=production CRANE_DATA_DIR=/data HOST=0.0.0.0 PORT=8787
WORKDIR /app
COPY --from=build /src/cluster/server/dist /app
RUN mkdir -p /data && chown node:node /data
USER node
VOLUME /data
EXPOSE 8787
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s --retries=3 \
  CMD node -e "fetch('http://127.0.0.1:'+(process.env.PORT||8787)+'/healthz').then(r=>process.exit(r.ok?0:1),()=>process.exit(1))"
STOPSIGNAL SIGTERM
CMD ["node", "--disable-warning=ExperimentalWarning", "--enable-source-maps", "main.mjs"]
