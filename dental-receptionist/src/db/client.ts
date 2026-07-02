import { DatabaseSync } from "node:sqlite";
import { mkdirSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const DATA_DIR = path.resolve(__dirname, "../../data");
const DB_PATH = process.env.SQLITE_PATH ?? path.join(DATA_DIR, "dental-receptionist.db");

mkdirSync(DATA_DIR, { recursive: true });

// node:sqlite ships with Node 22+; it's flagged experimental upstream but the
// synchronous API is stable enough for this workload and avoids a native
// build step (better-sqlite3) that can fail in constrained deploy environments.
export const db = new DatabaseSync(DB_PATH);
db.exec("PRAGMA journal_mode = WAL;");
db.exec("PRAGMA foreign_keys = ON;");

export const SCHEMA_SQL = `
CREATE TABLE IF NOT EXISTS slots (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  practice_slug TEXT NOT NULL,
  dentist TEXT NOT NULL,
  treatment_id TEXT NOT NULL,
  start_time TEXT NOT NULL,
  end_time TEXT NOT NULL,
  is_booked INTEGER NOT NULL DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_slots_practice_time ON slots(practice_slug, start_time);

CREATE TABLE IF NOT EXISTS calls (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  vapi_call_id TEXT UNIQUE NOT NULL,
  practice_slug TEXT NOT NULL,
  caller_number TEXT,
  started_at TEXT,
  ended_at TEXT,
  duration_seconds INTEGER,
  ended_reason TEXT,
  intent TEXT,
  outcome TEXT NOT NULL DEFAULT 'in_progress',
  flag_booked INTEGER NOT NULL DEFAULT 0,
  flag_message INTEGER NOT NULL DEFAULT 0,
  flag_emergency INTEGER NOT NULL DEFAULT 0,
  transcript TEXT,
  summary TEXT,
  recording_url TEXT,
  created_at TEXT NOT NULL DEFAULT (datetime('now')),
  updated_at TEXT NOT NULL DEFAULT (datetime('now'))
);
CREATE INDEX IF NOT EXISTS idx_calls_practice_created ON calls(practice_slug, created_at);

CREATE TABLE IF NOT EXISTS bookings (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  practice_slug TEXT NOT NULL,
  call_id TEXT,
  slot_id INTEGER NOT NULL REFERENCES slots(id),
  patient_name TEXT NOT NULL,
  patient_phone TEXT NOT NULL,
  treatment_id TEXT NOT NULL,
  dentist TEXT NOT NULL,
  start_time TEXT NOT NULL,
  end_time TEXT NOT NULL,
  status TEXT NOT NULL DEFAULT 'confirmed',
  sms_status TEXT NOT NULL DEFAULT 'pending',
  created_at TEXT NOT NULL DEFAULT (datetime('now'))
);
CREATE INDEX IF NOT EXISTS idx_bookings_practice_created ON bookings(practice_slug, created_at);

CREATE TABLE IF NOT EXISTS messages (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  practice_slug TEXT NOT NULL,
  call_id TEXT,
  caller_name TEXT,
  caller_phone TEXT,
  reason TEXT NOT NULL,
  urgent INTEGER NOT NULL DEFAULT 0,
  created_at TEXT NOT NULL DEFAULT (datetime('now'))
);
CREATE INDEX IF NOT EXISTS idx_messages_practice_created ON messages(practice_slug, created_at);
`;

export function initSchema(): void {
  db.exec(SCHEMA_SQL);
}

initSchema();
