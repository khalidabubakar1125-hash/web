import { db } from "./client.js";

export function markBookingSmsStatus(bookingId: string, status: "sent" | "failed"): void {
  db.prepare(`UPDATE bookings SET sms_status = ? WHERE id = ?`).run(status, Number(bookingId));
}
