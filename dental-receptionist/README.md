# AI Dental Receptionist (Vapi)

A voice AI receptionist for UK dental practices, built on [Vapi](https://vapi.ai).
Every practice is a config file (`/configs/*.yaml`) — adding a new client is a new
YAML file plus one CLI command, no code changes.

- `configs/` — one YAML file per practice (start here to onboard a client)
- `src/config/` — config schema + loader
- `src/assistant/` — turns a config into a Vapi assistant via the API
- `src/server/` — Fastify webhook server (Vapi function calls + end-of-call logging)
  and the admin page
- `src/booking/` — `BookingProvider` interface + SQLite mock-availability implementation
- `src/sms/` — Twilio booking confirmations
- `src/db/` — SQLite schema + call/booking/message logging
- `docs/RUNBOOK.md` — full onboarding + testing playbook
- `PRIVACY.md` — data flows, for building a client-facing DPA later

See `docs/RUNBOOK.md` for setup, deployment, and exactly what to test.

```bash
npm install
cp .env.example .env   # fill in Vapi / Twilio / Anthropic / admin credentials
npm run seed            # generate mock availability
npm run dev              # local webhook server
npm run create-assistant -- brightside-demo   # once deployed + PUBLIC_WEBHOOK_BASE_URL is set
```
