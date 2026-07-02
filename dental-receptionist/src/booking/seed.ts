import { db } from "../db/client.js";
import type { Dentist, PracticeConfig, Treatment } from "../config/schema.js";

const DAY_KEYS = ["sun", "mon", "tue", "wed", "thu", "fri", "sat"] as const;

function isHygienist(dentist: Dentist): boolean {
  return dentist.role.toLowerCase().includes("hygienist");
}

// Mock-slot simplification: NHS band entries are price-quoting only (a caller asks
// "what's an NHS check-up?"), not separately bookable — they map onto the same
// private-equivalent appointment types. Hygienists only offer the hygiene treatment.
function treatmentsForDentist(config: PracticeConfig, dentist: Dentist): Treatment[] {
  const bookable = config.treatments.filter((t) => t.category !== "nhs");
  return isHygienist(dentist)
    ? bookable.filter((t) => t.id === "hygiene")
    : bookable.filter((t) => t.id !== "hygiene");
}

/**
 * Regenerates mock availability for a practice over the next `days` days.
 * Only ever deletes/replaces slots that are still unbooked, so re-running is safe.
 */
export function seedSlots(config: PracticeConfig, days = 14): number {
  db.prepare(`DELETE FROM slots WHERE practice_slug = ? AND is_booked = 0`).run(config.slug);

  const insertStmt = db.prepare(
    `INSERT INTO slots (practice_slug, dentist, treatment_id, start_time, end_time) VALUES (?, ?, ?, ?, ?)`,
  );

  let created = 0;
  const now = new Date();

  for (let dayOffset = 0; dayOffset < days; dayOffset++) {
    const date = new Date(now);
    date.setDate(date.getDate() + dayOffset);
    const dayKey = DAY_KEYS[date.getDay()];
    const hours = config.openingHours[dayKey];
    if (!hours) continue;

    for (const dentist of config.dentists) {
      const treatments = treatmentsForDentist(config, dentist);
      if (treatments.length === 0) continue;

      const [openH, openM] = hours.open.split(":").map(Number);
      const [closeH, closeM] = hours.close.split(":").map(Number);

      let cursor = new Date(date);
      cursor.setHours(openH, openM, 0, 0);
      const dayEnd = new Date(date);
      dayEnd.setHours(closeH, closeM, 0, 0);

      let treatmentIndex = 0;
      // Round-robins through the dentist's treatment list so a working day offers
      // a realistic mix rather than one appointment type booked back-to-back.
      while (true) {
        const treatment = treatments[treatmentIndex % treatments.length];
        const slotStart = new Date(cursor);
        const slotEnd = new Date(cursor.getTime() + treatment.durationMinutes * 60_000);
        if (slotEnd > dayEnd) break;

        if (slotStart.getTime() > now.getTime()) {
          insertStmt.run(
            config.slug,
            dentist.name,
            treatment.id,
            slotStart.toISOString(),
            slotEnd.toISOString(),
          );
          created++;
        }

        cursor = slotEnd;
        treatmentIndex++;
      }
    }
  }

  return created;
}
