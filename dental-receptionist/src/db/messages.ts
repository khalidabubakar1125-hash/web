import { db } from "./client.js";

export interface MessageRow {
  id: number;
  practice_slug: string;
  call_id: string | null;
  caller_name: string | null;
  caller_phone: string | null;
  reason: string;
  urgent: number;
  created_at: string;
}

export function createMessage(params: {
  practiceSlug: string;
  callId?: string | null;
  callerName?: string | null;
  callerPhone?: string | null;
  reason: string;
  urgent?: boolean;
}): MessageRow {
  const stmt = db.prepare(
    `INSERT INTO messages (practice_slug, call_id, caller_name, caller_phone, reason, urgent)
     VALUES (?, ?, ?, ?, ?, ?)`,
  );
  const result = stmt.run(
    params.practiceSlug,
    params.callId ?? null,
    params.callerName ?? null,
    params.callerPhone ?? null,
    params.reason,
    params.urgent ? 1 : 0,
  );
  return db
    .prepare(`SELECT * FROM messages WHERE id = ?`)
    .get(Number(result.lastInsertRowid)) as unknown as MessageRow;
}

export function listRecentMessages(practiceSlug: string, limit = 50): MessageRow[] {
  return db
    .prepare(`SELECT * FROM messages WHERE practice_slug = ? ORDER BY created_at DESC LIMIT ?`)
    .all(practiceSlug, limit) as unknown as MessageRow[];
}
