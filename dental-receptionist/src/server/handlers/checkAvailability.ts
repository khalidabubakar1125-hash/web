import type { PracticeConfig } from "../../config/schema.js";
import type { BookingProvider } from "../../booking/BookingProvider.js";
import type { AvailabilitySlot } from "../../booking/types.js";
import { formatSpoken } from "../format.js";

export interface HandlerContext {
  config: PracticeConfig;
  booking: BookingProvider;
  vapiCallId: string;
}

function filterByTimeOfDay(slots: AvailabilitySlot[], timeOfDay?: string): AvailabilitySlot[] {
  if (!timeOfDay || timeOfDay === "any") return slots;
  return slots.filter((s) => {
    const hour = new Date(s.startTime).getHours();
    if (timeOfDay === "morning") return hour < 12;
    if (timeOfDay === "afternoon") return hour >= 12 && hour < 17;
    if (timeOfDay === "evening") return hour >= 17;
    return true;
  });
}

export async function handleCheckAvailability(
  args: { treatmentId?: string; preferredDate?: string; timeOfDay?: string },
  ctx: HandlerContext,
): Promise<string> {
  const treatment = ctx.config.treatments.find((t) => t.id === args.treatmentId);
  if (!args.treatmentId || !treatment) {
    const ids = ctx.config.treatments.map((t) => t.id).join(", ");
    return `Unknown treatmentId "${args.treatmentId ?? ""}". Valid ids: ${ids}.`;
  }

  let fromDate = new Date();
  if (args.preferredDate) {
    const parsed = new Date(args.preferredDate);
    if (!Number.isNaN(parsed.getTime())) fromDate = parsed;
  }

  const slots = await ctx.booking.getAvailability({
    practiceSlug: ctx.config.slug,
    treatmentId: treatment.id,
    fromDate: fromDate.toISOString(),
    limit: 8,
  });

  const filtered = filterByTimeOfDay(slots, args.timeOfDay);
  const chosen = (filtered.length > 0 ? filtered : slots).slice(0, 3);

  if (chosen.length === 0) {
    return `No ${treatment.name} slots are available in the near future. Apologise and offer to take a message so the practice can call back with options.`;
  }

  const lines = chosen.map((s) => `slotId ${s.slotId}: ${formatSpoken(new Date(s.startTime))} with ${s.dentist}`);
  return `Available ${treatment.name} slots (read 2-3 of these naturally, use the exact slotId when calling book_appointment):\n${lines.join("\n")}`;
}
