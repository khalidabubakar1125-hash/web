import twilio from "twilio";
import type { PracticeConfig, Treatment } from "../config/schema.js";
import type { BookingRecord } from "../booking/types.js";
import { formatSpoken } from "../server/format.js";

let client: ReturnType<typeof twilio> | null = null;

function getClient() {
  if (client) return client;
  const sid = process.env.TWILIO_ACCOUNT_SID;
  const token = process.env.TWILIO_AUTH_TOKEN;
  if (!sid || !token) throw new Error("TWILIO_ACCOUNT_SID / TWILIO_AUTH_TOKEN are not set");
  client = twilio(sid, token);
  return client;
}

/**
 * Normalises a UK number to E.164 (+44...) for Twilio. Accepts common caller-entered
 * formats: 07..., +447..., 447..., with spaces/dashes.
 */
export function toE164UK(rawNumber: string): string {
  const digits = rawNumber.replace(/[^\d+]/g, "");
  if (digits.startsWith("+")) return digits;
  if (digits.startsWith("0")) return `+44${digits.slice(1)}`;
  if (digits.startsWith("44")) return `+${digits}`;
  return `+44${digits}`;
}

export interface SendResult {
  ok: boolean;
  sid?: string;
  error?: string;
}

export async function sendBookingConfirmation(params: {
  config: PracticeConfig;
  booking: BookingRecord;
  treatment: Treatment;
}): Promise<SendResult> {
  const from = params.config.sms.fromNumber ?? process.env.TWILIO_PHONE_NUMBER;
  if (!from) return { ok: false, error: "No Twilio from-number configured" };

  const start = new Date(params.booking.startTime);
  const body =
    `${params.config.name}: your ${params.treatment.name.toLowerCase()} appointment is ` +
    `confirmed for ${formatSpoken(start)} with ${params.booking.dentist}. ` +
    `${params.config.address}. Reply or call ${params.config.phoneDisplay} to change it.`;

  try {
    const message = await getClient().messages.create({
      to: toE164UK(params.booking.patientPhone),
      from,
      body,
    });
    return { ok: true, sid: message.sid };
  } catch (err) {
    return { ok: false, error: err instanceof Error ? err.message : String(err) };
  }
}
