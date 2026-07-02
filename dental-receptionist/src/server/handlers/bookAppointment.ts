import type { HandlerContext } from "./checkAvailability.js";
import { markBookingSmsStatus } from "../../db/bookings.js";
import { markCallFlag } from "../../db/calls.js";
import { sendBookingConfirmation } from "../../sms/sendConfirmation.js";
import { formatSpoken } from "../format.js";

export async function handleBookAppointment(
  args: {
    slotId?: string;
    treatmentId?: string;
    patientName?: string;
    patientPhone?: string;
  },
  ctx: HandlerContext,
): Promise<string> {
  const { slotId, treatmentId, patientName, patientPhone } = args;
  if (!slotId || !treatmentId || !patientName || !patientPhone) {
    return "Missing booking details — ask the caller for whichever of slot choice, treatment, name or phone number is still missing, then try again.";
  }

  const treatment = ctx.config.treatments.find((t) => t.id === treatmentId);
  if (!treatment) return `Unknown treatmentId "${treatmentId}". Call check_availability again to get a valid one.`;

  let booking;
  try {
    booking = await ctx.booking.bookSlot({
      practiceSlug: ctx.config.slug,
      slotId,
      patientName,
      patientPhone,
      treatmentId,
      callId: ctx.vapiCallId,
    });
  } catch (err) {
    const code = err instanceof Error ? err.message : "";
    if (code === "SLOT_ALREADY_BOOKED") {
      return "That slot has just been taken. Call check_availability again and offer the caller fresh alternatives.";
    }
    if (code === "SLOT_NOT_FOUND") {
      return "That slotId isn't recognised. Call check_availability again to get valid slot ids before retrying.";
    }
    throw err;
  }

  markCallFlag(ctx.vapiCallId, "booked");

  const smsResult = await sendBookingConfirmation({ config: ctx.config, booking, treatment }).catch(
    (err: unknown) => ({ ok: false as const, error: err instanceof Error ? err.message : String(err) }),
  );
  markBookingSmsStatus(booking.bookingId, smsResult.ok ? "sent" : "failed");

  const spoken = formatSpoken(new Date(booking.startTime));
  const smsNote = smsResult.ok
    ? "A confirmation text has been sent to that number."
    : "(Note for the assistant: the confirmation text failed to send — tell the caller a team member will confirm by phone instead.)";

  return `Booked: ${treatment.name} with ${booking.dentist} on ${spoken} for ${patientName}. ${smsNote}`;
}
