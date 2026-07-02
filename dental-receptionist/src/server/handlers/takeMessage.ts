import type { HandlerContext } from "./checkAvailability.js";
import { createMessage } from "../../db/messages.js";
import { markCallFlag } from "../../db/calls.js";

export async function handleTakeMessage(
  args: { callerName?: string; callerPhone?: string; reason?: string },
  ctx: HandlerContext,
): Promise<string> {
  const { callerName, callerPhone, reason } = args;
  if (!callerName || !callerPhone || !reason) {
    return "Missing details — ask the caller for whichever of their name, phone number, or reason is still missing, then try again.";
  }

  createMessage({
    practiceSlug: ctx.config.slug,
    callId: ctx.vapiCallId,
    callerName,
    callerPhone,
    reason,
  });
  markCallFlag(ctx.vapiCallId, "message");

  return `Message logged for ${callerName}. Reassure them the practice will call back on ${callerPhone}.`;
}
