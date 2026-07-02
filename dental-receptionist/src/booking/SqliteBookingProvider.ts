import { db } from "../db/client.js";
import type { BookingProvider } from "./BookingProvider.js";
import type {
  AvailabilitySlot,
  BookingRecord,
  BookSlotRequest,
  GetAvailabilityParams,
} from "./types.js";

interface SlotRow {
  id: number;
  practice_slug: string;
  dentist: string;
  treatment_id: string;
  start_time: string;
  end_time: string;
  is_booked: number;
}

interface BookingRow {
  id: number;
  practice_slug: string;
  slot_id: number;
  patient_name: string;
  patient_phone: string;
  treatment_id: string;
  dentist: string;
  start_time: string;
  end_time: string;
  status: "confirmed" | "cancelled";
}

function toSlot(row: SlotRow): AvailabilitySlot {
  return {
    slotId: String(row.id),
    dentist: row.dentist,
    treatmentId: row.treatment_id,
    startTime: row.start_time,
    endTime: row.end_time,
  };
}

function toBooking(row: BookingRow): BookingRecord {
  return {
    bookingId: String(row.id),
    practiceSlug: row.practice_slug,
    slotId: String(row.slot_id),
    patientName: row.patient_name,
    patientPhone: row.patient_phone,
    treatmentId: row.treatment_id,
    dentist: row.dentist,
    startTime: row.start_time,
    endTime: row.end_time,
    status: row.status,
  };
}

export class SqliteBookingProvider implements BookingProvider {
  async getAvailability(params: GetAvailabilityParams): Promise<AvailabilitySlot[]> {
    const from = params.fromDate ?? new Date().toISOString();
    const limit = params.limit ?? 5;

    const rows = params.treatmentId
      ? (db
          .prepare(
            `SELECT * FROM slots
             WHERE practice_slug = ? AND treatment_id = ? AND is_booked = 0 AND start_time >= ?
             ORDER BY start_time ASC LIMIT ?`,
          )
          .all(params.practiceSlug, params.treatmentId, from, limit) as unknown as SlotRow[])
      : (db
          .prepare(
            `SELECT * FROM slots
             WHERE practice_slug = ? AND is_booked = 0 AND start_time >= ?
             ORDER BY start_time ASC LIMIT ?`,
          )
          .all(params.practiceSlug, from, limit) as unknown as SlotRow[]);

    return rows.map(toSlot);
  }

  async bookSlot(params: BookSlotRequest): Promise<BookingRecord> {
    const slotId = Number(params.slotId);

    db.exec("BEGIN IMMEDIATE");
    try {
      const slot = db
        .prepare(`SELECT * FROM slots WHERE id = ? AND practice_slug = ?`)
        .get(slotId, params.practiceSlug) as SlotRow | undefined;

      if (!slot) {
        db.exec("ROLLBACK");
        throw new Error("SLOT_NOT_FOUND");
      }
      if (slot.is_booked) {
        db.exec("ROLLBACK");
        throw new Error("SLOT_ALREADY_BOOKED");
      }

      db.prepare(`UPDATE slots SET is_booked = 1 WHERE id = ?`).run(slotId);

      const insert = db.prepare(
        `INSERT INTO bookings
           (practice_slug, call_id, slot_id, patient_name, patient_phone, treatment_id, dentist, start_time, end_time)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)`,
      );
      const result = insert.run(
        params.practiceSlug,
        params.callId ?? null,
        slotId,
        params.patientName,
        params.patientPhone,
        params.treatmentId,
        slot.dentist,
        slot.start_time,
        slot.end_time,
      );

      db.exec("COMMIT");

      const bookingRow = db
        .prepare(`SELECT * FROM bookings WHERE id = ?`)
        .get(Number(result.lastInsertRowid)) as unknown as BookingRow;

      return toBooking(bookingRow);
    } catch (err) {
      try {
        db.exec("ROLLBACK");
      } catch {
        // no-op: transaction may already have been rolled back above
      }
      throw err;
    }
  }

  async listBookings(practiceSlug: string, limit = 50): Promise<BookingRecord[]> {
    const rows = db
      .prepare(
        `SELECT * FROM bookings WHERE practice_slug = ? ORDER BY created_at DESC LIMIT ?`,
      )
      .all(practiceSlug, limit) as unknown as BookingRow[];
    return rows.map(toBooking);
  }
}
