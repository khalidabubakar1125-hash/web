import type { FastifyInstance } from "fastify";
import { timingSafeEqual } from "node:crypto";
import { buildPracticeRegistry, resolvePractice } from "../practiceRegistry.js";
import { SqliteBookingProvider } from "../../booking/SqliteBookingProvider.js";
import { ensureCall, finalizeCall, getCall, setCallIntent } from "../../db/calls.js";
import { classifyIntentFromTranscript } from "../../db/classify.js";
import {
  extractAssistantId,
  extractCallId,
  extractCallerNumber,
  extractMetadataSlug,
  extractPhoneNumberId,
  extractToolCalls,
  normalizeEndOfCallReport,
} from "../vapiPayload.js";
import { handleCheckAvailability, type HandlerContext } from "../handlers/checkAvailability.js";
import { handleBookAppointment } from "../handlers/bookAppointment.js";
import { handleTakeMessage } from "../handlers/takeMessage.js";
import { handleFlagEmergency } from "../handlers/flagEmergency.js";

const booking = new SqliteBookingProvider();

type Handler = (args: Record<string, unknown>, ctx: HandlerContext) => Promise<string>;

const HANDLERS: Record<string, Handler> = {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  check_availability: handleCheckAvailability as any,
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  book_appointment: handleBookAppointment as any,
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  take_message: handleTakeMessage as any,
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  flag_emergency: handleFlagEmergency as any,
};

const INTENT_LABELS: Record<string, string> = {
  check_availability: "booking_enquiry",
  book_appointment: "booking",
  take_message: "message",
  flag_emergency: "emergency",
};

function secretMatches(provided: unknown, expected: string): boolean {
  if (typeof provided !== "string" || provided.length !== expected.length) return false;
  return timingSafeEqual(Buffer.from(provided), Buffer.from(expected));
}

export async function registerWebhookRoute(app: FastifyInstance): Promise<void> {
  const registry = buildPracticeRegistry();

  app.post("/webhook/vapi", async (request, reply) => {
    const expectedSecret = process.env.VAPI_SERVER_SECRET;
    if (expectedSecret && !secretMatches(request.headers["x-vapi-secret"], expectedSecret)) {
      return reply.code(401).send({ error: "invalid secret" });
    }

    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    const body = request.body as any;
    const message = body?.message ?? body;
    if (!message?.type) return reply.code(200).send({});

    const config = resolvePractice(registry, {
      assistantId: extractAssistantId(message),
      phoneNumberId: extractPhoneNumberId(message),
      metadataSlug: extractMetadataSlug(message),
    });

    if (!config) {
      request.log.warn({ type: message.type }, "Could not resolve practice for webhook event");
      return reply.code(200).send({});
    }

    const vapiCallId = extractCallId(message);
    const callerNumber = extractCallerNumber(message);

    if (message.type === "tool-calls") {
      ensureCall({ vapiCallId, practiceSlug: config.slug, callerNumber });

      const toolCalls = extractToolCalls(message);
      const results = [];
      for (const call of toolCalls) {
        const handler = HANDLERS[call.name];
        let result: string;
        try {
          if (!handler) {
            result = `Unknown tool "${call.name}".`;
          } else {
            result = await handler(call.arguments, { config, booking, vapiCallId });
            const intentLabel = INTENT_LABELS[call.name];
            if (intentLabel) setCallIntent(vapiCallId, intentLabel);
          }
        } catch (err) {
          request.log.error(err, `Handler for ${call.name} failed`);
          result =
            "Something went wrong on our end handling that. Apologise briefly and take a message instead so the practice can follow up.";
        }
        results.push({ toolCallId: call.id, result });
      }

      return reply.code(200).send({ results });
    }

    if (message.type === "end-of-call-report") {
      const normalized = normalizeEndOfCallReport(message);
      finalizeCall({ vapiCallId, practiceSlug: config.slug, callerNumber, ...normalized });

      // Best-effort: label calls that never triggered a tool call (pure Q&A, or a hang-up).
      const call = getCall(vapiCallId);
      if (call && !call.intent && normalized.transcript) {
        const label = await classifyIntentFromTranscript(normalized.transcript);
        if (label) setCallIntent(vapiCallId, label);
      }

      return reply.code(200).send({});
    }

    // Other event types (status-update, speech-update, hang, etc.) — nothing to do.
    return reply.code(200).send({});
  });
}
