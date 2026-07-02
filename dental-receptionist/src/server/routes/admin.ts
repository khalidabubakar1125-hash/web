import type { FastifyInstance } from "fastify";
import { timingSafeEqual } from "node:crypto";
import { buildPracticeRegistry } from "../practiceRegistry.js";
import { SqliteBookingProvider } from "../../booking/SqliteBookingProvider.js";
import { listRecentCalls } from "../../db/calls.js";
import { listRecentMessages } from "../../db/messages.js";
import { renderAdminPage } from "../views/adminPage.js";

const booking = new SqliteBookingProvider();

function safeEqual(a: string, b: string): boolean {
  const bufA = Buffer.from(a);
  const bufB = Buffer.from(b);
  if (bufA.length !== bufB.length) return false;
  return timingSafeEqual(bufA, bufB);
}

export async function registerAdminRoutes(app: FastifyInstance): Promise<void> {
  const registry = buildPracticeRegistry();

  app.addHook("onRequest", async (request, reply) => {
    if (!request.url.startsWith("/admin")) return;

    const user = process.env.ADMIN_USERNAME;
    const pass = process.env.ADMIN_PASSWORD;
    if (!user || !pass) {
      reply.code(500).send("Admin page is not configured: set ADMIN_USERNAME and ADMIN_PASSWORD.");
      return;
    }

    const header = request.headers.authorization ?? "";
    const [scheme, encoded] = header.split(" ");
    const decoded = encoded ? Buffer.from(encoded, "base64").toString("utf8") : "";
    const [providedUser, providedPass] = decoded.split(":");

    const authorized =
      scheme === "Basic" &&
      providedUser !== undefined &&
      providedPass !== undefined &&
      safeEqual(providedUser, user) &&
      safeEqual(providedPass, pass);

    if (!authorized) {
      reply
        .code(401)
        .header("WWW-Authenticate", 'Basic realm="Admin"')
        .send("Authentication required.");
    }
  });

  app.get("/admin", async (request, reply) => {
    const query = request.query as { practice?: string };
    const practices = registry.all;

    if (practices.length === 0) {
      reply.type("text/html").send("<p>No practice configs found in /configs.</p>");
      return;
    }

    const activePractice =
      registry.bySlug.get(query.practice ?? "") ??
      registry.bySlug.get(process.env.DEFAULT_PRACTICE_SLUG ?? "") ??
      practices[0];

    const [calls, bookings, messages] = [
      listRecentCalls(activePractice.slug, 100),
      await booking.listBookings(activePractice.slug, 100),
      listRecentMessages(activePractice.slug, 100),
    ];

    reply.type("text/html").send(
      renderAdminPage({ practices, activePractice, calls, bookings, messages }),
    );
  });
}
