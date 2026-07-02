import { db } from "./client.js";

export type CallOutcome =
  | "in_progress"
  | "booked"
  | "message_taken"
  | "emergency_escalated"
  | "info_given"
  | "no_action";

export interface CallRow {
  id: number;
  vapi_call_id: string;
  practice_slug: string;
  caller_number: string | null;
  started_at: string | null;
  ended_at: string | null;
  duration_seconds: number | null;
  ended_reason: string | null;
  intent: string | null;
  outcome: CallOutcome;
  flag_booked: number;
  flag_message: number;
  flag_emergency: number;
  transcript: string | null;
  summary: string | null;
  recording_url: string | null;
  created_at: string;
  updated_at: string;
}

/** Creates the call row on first tool-call if it doesn't exist yet; otherwise no-op. */
export function ensureCall(params: {
  vapiCallId: string;
  practiceSlug: string;
  callerNumber?: string | null;
}): void {
  const stmt = db.prepare(
    `INSERT INTO calls (vapi_call_id, practice_slug, caller_number, started_at)
     VALUES (?, ?, ?, datetime('now'))
     ON CONFLICT(vapi_call_id) DO NOTHING`,
  );
  stmt.run(params.vapiCallId, params.practiceSlug, params.callerNumber ?? null);
}

export function markCallFlag(vapiCallId: string, flag: "booked" | "message" | "emergency"): void {
  const column = `flag_${flag}`;
  db.prepare(
    `UPDATE calls SET ${column} = 1, updated_at = datetime('now') WHERE vapi_call_id = ?`,
  ).run(vapiCallId);
}

export function setCallIntent(vapiCallId: string, intent: string): void {
  db.prepare(
    `UPDATE calls SET intent = ?, updated_at = datetime('now') WHERE vapi_call_id = ? AND intent IS NULL`,
  ).run(intent, vapiCallId);
}

function computeOutcome(row: Pick<CallRow, "flag_booked" | "flag_message" | "flag_emergency">): CallOutcome {
  if (row.flag_emergency) return "emergency_escalated";
  if (row.flag_booked) return "booked";
  if (row.flag_message) return "message_taken";
  return "info_given";
}

export interface EndOfCallReportInput {
  vapiCallId: string;
  practiceSlug: string;
  callerNumber?: string | null;
  endedAt?: string | null;
  durationSeconds?: number | null;
  endedReason?: string | null;
  transcript?: string | null;
  summary?: string | null;
  recordingUrl?: string | null;
}

/** Finalises a call row from the end-of-call-report webhook. Upserts in case no tool-call ever fired. */
export function finalizeCall(input: EndOfCallReportInput): void {
  ensureCall({
    vapiCallId: input.vapiCallId,
    practiceSlug: input.practiceSlug,
    callerNumber: input.callerNumber,
  });

  const existing = db
    .prepare(`SELECT flag_booked, flag_message, flag_emergency FROM calls WHERE vapi_call_id = ?`)
    .get(input.vapiCallId) as Pick<CallRow, "flag_booked" | "flag_message" | "flag_emergency"> | undefined;

  const outcome = existing
    ? computeOutcome(existing)
    : ("no_action" as CallOutcome);

  db.prepare(
    `UPDATE calls SET
       caller_number = COALESCE(?, caller_number),
       ended_at = COALESCE(?, datetime('now')),
       duration_seconds = COALESCE(?, duration_seconds),
       ended_reason = COALESCE(?, ended_reason),
       transcript = COALESCE(?, transcript),
       summary = COALESCE(?, summary),
       recording_url = COALESCE(?, recording_url),
       outcome = ?,
       updated_at = datetime('now')
     WHERE vapi_call_id = ?`,
  ).run(
    input.callerNumber ?? null,
    input.endedAt ?? null,
    input.durationSeconds ?? null,
    input.endedReason ?? null,
    input.transcript ?? null,
    input.summary ?? null,
    input.recordingUrl ?? null,
    outcome,
    input.vapiCallId,
  );
}

export function listRecentCalls(practiceSlug: string, limit = 50): CallRow[] {
  return db
    .prepare(
      `SELECT * FROM calls WHERE practice_slug = ? ORDER BY created_at DESC LIMIT ?`,
    )
    .all(practiceSlug, limit) as unknown as CallRow[];
}

export function getCall(vapiCallId: string): CallRow | undefined {
  return db.prepare(`SELECT * FROM calls WHERE vapi_call_id = ?`).get(vapiCallId) as
    | CallRow
    | undefined;
}
