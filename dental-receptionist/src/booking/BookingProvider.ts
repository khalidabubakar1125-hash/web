import type {
  AvailabilitySlot,
  BookingRecord,
  BookSlotRequest,
  GetAvailabilityParams,
} from "./types.js";

/**
 * Everything the assistant needs from a booking backend. The demo implementation
 * (SqliteBookingProvider) is backed by mock slots in SQLite. Swapping to Google
 * Calendar or a practice management system's API later means writing a new
 * class against this interface — nothing in the assistant or webhook layer changes.
 */
export interface BookingProvider {
  getAvailability(params: GetAvailabilityParams): Promise<AvailabilitySlot[]>;
  bookSlot(params: BookSlotRequest): Promise<BookingRecord>;
  listBookings(practiceSlug: string, limit?: number): Promise<BookingRecord[]>;
}
