export interface AvailabilitySlot {
  slotId: string;
  dentist: string;
  treatmentId: string;
  startTime: string; // ISO 8601, Europe/London
  endTime: string;
}

export interface BookSlotRequest {
  practiceSlug: string;
  slotId: string;
  patientName: string;
  patientPhone: string;
  treatmentId: string;
  callId?: string | null;
}

export interface BookingRecord {
  bookingId: string;
  practiceSlug: string;
  slotId: string;
  patientName: string;
  patientPhone: string;
  treatmentId: string;
  dentist: string;
  startTime: string;
  endTime: string;
  status: "confirmed" | "cancelled";
}

export interface GetAvailabilityParams {
  practiceSlug: string;
  treatmentId?: string;
  fromDate?: string; // ISO date, defaults to now
  limit?: number;
}
