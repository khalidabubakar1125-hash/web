import { z } from "zod";

const dayHoursSchema = z
  .object({
    open: z.string().regex(/^\d{2}:\d{2}$/, "use HH:MM"),
    close: z.string().regex(/^\d{2}:\d{2}$/, "use HH:MM"),
  })
  .nullable();

export const openingHoursSchema = z.object({
  mon: dayHoursSchema,
  tue: dayHoursSchema,
  wed: dayHoursSchema,
  thu: dayHoursSchema,
  fri: dayHoursSchema,
  sat: dayHoursSchema,
  sun: dayHoursSchema,
});

export const dentistSchema = z.object({
  name: z.string(),
  role: z.string(), // e.g. "Dentist", "Hygienist", "Principal Dentist"
});

export const treatmentSchema = z.object({
  id: z.string(), // stable slug used by booking + function-call args, e.g. "hygiene"
  name: z.string(), // spoken/display name, e.g. "Hygiene appointment"
  priceDisplay: z.string(), // e.g. "£75" or "NHS Band 1 – £25.80"
  category: z.enum(["nhs", "private", "mixed"]).default("private"),
  durationMinutes: z.number().int().positive().default(30),
  notes: z.string().optional(),
});

export const emergencyProtocolSchema = z.object({
  offersSameDaySlot: z.boolean().default(true),
  inHoursAdvice: z.string(), // what to say/do when practice is open
  outOfHoursAdvice: z.string(), // e.g. direct to NHS 111
  criticalAdvice: z.string(), // e.g. swelling affecting breathing -> 999
});

export const bookingRulesSchema = z.object({
  newPatientsRequireExam: z.boolean().default(true),
  cancellationNoticeHours: z.number().int().nonnegative().default(24),
  depositRequiredTreatmentIds: z.array(z.string()).default([]),
});

export const voiceSchema = z.object({
  provider: z.string().default("playht"), // Vapi voice provider id
  voiceId: z.string().default("jennifer"), // British-accented voice recommended
});

export const vapiLinkSchema = z.object({
  assistantId: z.string().optional(), // filled in automatically after `npm run create-assistant`
  phoneNumberId: z.string().optional(), // Vapi phone number resource id (imported Twilio number)
});

export const smsSchema = z.object({
  fromNumber: z.string().optional(), // overrides TWILIO_PHONE_NUMBER for this practice
});

export const practiceConfigSchema = z.object({
  slug: z
    .string()
    .regex(/^[a-z0-9-]+$/, "lowercase, numbers and hyphens only"),
  name: z.string(),
  legalName: z.string().optional(),
  city: z.string(),
  address: z.string(),
  postcode: z.string(),
  phoneDisplay: z.string(), // human-readable number quoted to callers, e.g. "0161 000 0000"
  parkingInfo: z.string(),
  timezone: z.string().default("Europe/London"),
  openingHours: openingHoursSchema,
  dentists: z.array(dentistSchema).min(1),
  treatments: z.array(treatmentSchema).min(1),
  nhsInfo: z.string(),
  emergencyProtocol: emergencyProtocolSchema,
  bookingRules: bookingRulesSchema,
  voice: voiceSchema.default({ provider: "playht", voiceId: "jennifer" }),
  vapi: vapiLinkSchema.default({}),
  sms: smsSchema.default({}),
});

export type PracticeConfig = z.infer<typeof practiceConfigSchema>;
export type Treatment = z.infer<typeof treatmentSchema>;
export type Dentist = z.infer<typeof dentistSchema>;
export type OpeningHours = z.infer<typeof openingHoursSchema>;
