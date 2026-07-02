# RUNBOOK — AI Dental Receptionist

Playbook for standing up the demo line, and for onboarding every client after it.
Written so a delivery technician (not just the original builder) can follow it.

## Architecture recap

```
Caller ──PSTN──► Twilio UK number ──► Vapi (STT/TTS/telephony) ──webhook──► Fastify server
                                                                                  │
                                          ┌───────────────────────────────────────┼──────────────┐
                                          ▼                                       ▼              ▼
                                   SQLite (calls, bookings,               Twilio SMS      Anthropic (Claude,
                                   messages, mock slots)                  confirmations    assistant LLM +
                                                                                            call classification)
```

- **`/configs/*.yaml`** — one file per practice. This is the entire client-specific
  surface area. Nothing about a practice should ever be hardcoded in `/src`.
- **`/src/assistant`** — turns a config into a Vapi assistant via the API
  (`npm run create-assistant`), including the system prompt and the four tools
  (`check_availability`, `book_appointment`, `take_message`, `flag_emergency`).
- **`/src/server`** — the Fastify webhook server Vapi calls during and after every
  call, plus the admin page.
- **`/src/booking`** — `BookingProvider` interface + `SqliteBookingProvider` (mock
  slots). Swap in a Google Calendar or PMS-backed provider later without touching the
  assistant or webhook code.
- **`/src/sms`** — Twilio booking confirmations.
- **`/src/db`** — SQLite schema + call/booking/message logging (uses Node's built-in
  `node:sqlite`, so there's no native module to compile).

## 1. Local setup

```bash
cd dental-receptionist
npm install
cp .env.example .env
```

Fill in `.env`:
- `VAPI_API_KEY` — from the Vapi dashboard.
- `VAPI_SERVER_SECRET` — any long random string, e.g. `openssl rand -hex 32`. This is
  what proves incoming webhook requests really came from Vapi.
- `ANTHROPIC_API_KEY` — used as the assistant's LLM (via `VAPI_LLM_PROVIDER=anthropic`)
  and for optional call-intent classification.
- `TWILIO_ACCOUNT_SID` / `TWILIO_AUTH_TOKEN` / `TWILIO_PHONE_NUMBER` — a UK Twilio
  number (buy one in the Twilio console under Phone Numbers → Buy a number, filter to
  GB, look for SMS + Voice capability).
- `ADMIN_USERNAME` / `ADMIN_PASSWORD` — protects `/admin` (it shows transcripts and
  patient contact details).
- `DEFAULT_PRACTICE_SLUG=brightside-demo` for the demo deployment.

Seed mock availability:

```bash
npm run seed
```

This regenerates the next 14 days of appointment slots per dentist/hygienist, based
on `openingHours` and each treatment's `durationMinutes` in the config. Safe to
re-run any time — it never touches slots that are already booked.

Run the server:

```bash
npm run dev
```

Check `curl http://localhost:3000/health` returns `{"ok":true}`.

## 2. Deploy (Railway)

Railway is a good fit: simple git-push deploys, persistent volumes for the SQLite
file, and UK/EU regions.

1. Create a new Railway project from this repo (or `dental-receptionist/` as the
   root directory if deploying from a monorepo — set the Railway service's root
   directory accordingly).
2. Set the region to `europe-west4` (or another EU/UK region Railway offers).
3. Add a persistent volume mounted at `/app/data` (matches `SQLITE_PATH` default of
   `./data/dental-receptionist.db`) so bookings/call logs survive redeploys.
4. Set all the environment variables from `.env` in Railway's dashboard.
5. Build command: `npm run build`. Start command: `npm start` (or `node dist/server/index.js`
   once built — check `package.json` scripts match what you configure).
6. Once deployed, note the public URL (e.g. `https://dental-receptionist-production.up.railway.app`)
   and set `PUBLIC_WEBHOOK_BASE_URL` to it (both in Railway's env vars and your local
   `.env`, since the assistant-creation script runs locally).
7. Run `npm run seed` once against the deployed database (either via Railway's shell,
   or point `SQLITE_PATH` at the same volume from a local run — simplest is
   `railway run npm run seed`).

## 3. Create the Vapi assistant

With `PUBLIC_WEBHOOK_BASE_URL` pointing at your deployed server:

```bash
npm run create-assistant -- brightside-demo
```

This POSTs the config-derived assistant definition to Vapi, saves the returned
`assistantId` back into `configs/brightside-demo.yaml`, and — if
`TWILIO_ACCOUNT_SID`/`TWILIO_AUTH_TOKEN`/`TWILIO_PHONE_NUMBER` are set — imports that
Twilio number into Vapi and links it to the assistant automatically.

If you'd rather import the number by hand: Vapi dashboard → Phone Numbers → Import →
Twilio, paste the SID/token/number, then set the assistant on that number to the
`assistantId` printed by the script (or paste the phone number's Vapi id into
`vapi.phoneNumberId` in the config yourself).

Re-running `npm run create-assistant -- brightside-demo` after editing the config
(prices, hours, prompt copy, anything) **updates** the existing assistant in place —
it's idempotent, not a duplicate-creator.

## 4. Test it end to end

**Call the demo number** from your own phone and try each of these:

1. **Booking a hygiene appointment** (the golden path):
   - "Hi, I'd like to book a hygiene appointment."
   - Give a preferred day if asked, then pick one of the times it offers.
   - Give your name and mobile number.
   - Expect: a spoken confirmation, and an SMS within a few seconds confirming
     practice name, date/time, and treatment.
   - Check `/admin` (basic auth) → the call should show outcome `booked`, and the
     booking should appear in the Bookings table.

2. **Price question**: "How much is a check-up?" → should answer directly from the
   config (£45) without any function call, in `/admin` outcome will show `info_given`
   (or a classified label like `price_question` if `ANTHROPIC_API_KEY` is set).

3. **NHS question**: "Do you do NHS?" → should explain the mixed NHS/private setup
   and waiting list, referencing `nhsInfo` from the config.

4. **Emergency**: "I've knocked my tooth out and it's bleeding a lot" → should show
   empathy, offer the same-day emergency slot (or out-of-hours advice if you call
   outside opening hours), and log a `flag_emergency` call. Check `/admin` → outcome
   `emergency_escalated`, and the emergency should appear in the Messages table
   marked `URGENT`.

5. **Something it can't handle**: "I need to move my appointment next week" → should
   apologise, take your name/number/reason via `take_message`, and NOT attempt to
   book/cancel anything itself. `/admin` outcome: `message_taken`.

6. **"Are you a real person?"** → should honestly say it's an AI receptionist for the
   practice, never claim to be human.

7. **Recording disclosure**: listen to the opening greeting — it should mention calls
   may be recorded, before you say anything.

If a call doesn't behave as expected, check the Railway logs first (the server logs
every webhook event and handler error), then re-read the system prompt Vapi actually
has stored for the assistant in its dashboard (compare against
`src/assistant/systemPrompt.ts` — if they differ, re-run `create-assistant`).

## 5. Onboarding a new practice (client #2, #3, ...)

This should require **zero code changes**:

1. Copy `configs/brightside-demo.yaml` to `configs/<new-client-slug>.yaml`.
2. Fill in every field: name, address, hours, dentists, treatments/prices, NHS info,
   emergency protocol wording, booking rules. Leave `vapi: {}` and `sms: {}` empty.
3. Buy/port a UK Twilio number for this client if they don't already have one you're
   reusing, and set it as `sms.fromNumber` in their config (or leave empty to use the
   shared `TWILIO_PHONE_NUMBER` default — not recommended for real clients, each
   practice should have its own line).
4. Run `npm run seed` (seeds every config found in `/configs`, including the new one).
5. Run `npm run create-assistant -- <new-client-slug>`.
6. Test using the checklist in step 4 above, calling their number.
7. Point them at `https://<your-deployment>/admin?practice=<new-client-slug>` (same
   basic-auth credentials for now — per-client admin logins are a future improvement,
   see below).

## Known limitations / next steps for production

- **Admin auth** is a single shared basic-auth login across all practices. Fine for
  an agency-run demo/pilot; move to per-client accounts before scaling past a
  handful of clients.
- **Cancellations/rescheduling** aren't self-service — the assistant always takes a
  message. Wire up a `cancel_appointment`/`reschedule_appointment` tool once there's
  a real booking backend that supports it safely (double-booking protection, etc).
- **Call-intent classification** (used when a call never triggers a tool call) is a
  best-effort Claude call and silently no-ops without `ANTHROPIC_API_KEY` — don't
  rely on it for anything beyond an admin-page label.
- **Data retention** isn't automated yet — see `PRIVACY.md`.
- **BookingProvider** is SQLite/mock slots for the demo. Before a real client goes
  live, implement a new class against `src/booking/BookingProvider.ts` (Google
  Calendar, or their practice management system's API) and swap it in
  `src/server/routes/webhook.ts` and `src/server/routes/admin.ts` — no other files
  should need to change.
