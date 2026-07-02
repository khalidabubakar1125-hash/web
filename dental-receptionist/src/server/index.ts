import "dotenv/config";
import Fastify from "fastify";
import { registerWebhookRoute } from "./routes/webhook.js";
import { registerAdminRoutes } from "./routes/admin.js";

const app = Fastify({
  logger: {
    level: process.env.LOG_LEVEL ?? "info",
    // Redact anything that might carry patient PII from log output.
    redact: ["req.body.message.transcript", "req.body.message.artifact"],
  },
});

app.get("/health", async () => ({ ok: true }));

await registerWebhookRoute(app);
await registerAdminRoutes(app);

const port = Number(process.env.PORT ?? 3000);
const host = process.env.HOST ?? "0.0.0.0";

app
  .listen({ port, host })
  .then(() => app.log.info(`dental-receptionist webhook server listening on ${host}:${port}`))
  .catch((err) => {
    app.log.error(err);
    process.exit(1);
  });
