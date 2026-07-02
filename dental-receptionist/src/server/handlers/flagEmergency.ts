import type { HandlerContext } from "./checkAvailability.js";
import { createMessage } from "../../db/messages.js";
import { markCallFlag } from "../../db/calls.js";

export async function handleFlagEmergency(
  args: {
    description?: string;
    severity?: "urgent" | "critical";
    callerName?: string;
    callerPhone?: string;
  },
  ctx: HandlerContext,
): Promise<string> {
  if (!args.description) return "Missing an emergency description — ask what's wrong, then try again.";

  createMessage({
    practiceSlug: ctx.config.slug,
    callId: ctx.vapiCallId,
    callerName: args.callerName ?? null,
    callerPhone: args.callerPhone ?? null,
    reason: `EMERGENCY (${args.severity ?? "urgent"}): ${args.description}`,
    urgent: true,
  });
  markCallFlag(ctx.vapiCallId, "emergency");

  return args.severity === "critical"
    ? "Logged as critical. Confirm you have already told the caller to call 999 or go to A&E immediately if breathing or swallowing is affected."
    : "Emergency logged for the practice. Continue offering the same-day emergency slot or out-of-hours advice as appropriate.";
}
