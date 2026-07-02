# Privacy & data flows

This document exists so a client-facing DPA (Data Processing Agreement) can be
built from it later. It describes what personal data this system handles, where it
flows, and why — not legal advice.

## What we collect

For every call:
- Caller's phone number (from telephony metadata)
- Full call transcript and an AI-generated summary
- A recording of the call (audio)
- Whatever the caller volunteers to the assistant: name, callback number, treatment
  interest, and — for messages/emergencies — a free-text description of their query

For bookings:
- Patient name, phone number, chosen treatment, appointment time

We never ask for, and the assistant is instructed to refuse, card or payment details
over the phone.

## Data flow

```
Caller (PSTN)
   │
   ▼
Twilio (UK number) ── call audio ──► Vapi (speech-to-text, text-to-speech, telephony)
                                            │
                                            │  webhook: tool calls, end-of-call report
                                            │  (transcript, recording URL, summary)
                                            ▼
                                   Our webhook server (Fastify)
                                            │
                              ┌─────────────┼──────────────────┐
                              ▼             ▼                  ▼
                        SQLite DB     Twilio (SMS out)   Anthropic (Claude)
                     (calls, bookings,  booking confirmation   assistant's LLM +
                      messages)         to patient             optional call-intent
                                                                classification
```

Third parties in the flow, and their role:
- **Vapi** — orchestrates the call: speech-to-text, text-to-speech, telephony
  routing, and generates the call recording and transcript. Vapi's own sub-processors
  (transcription/voice vendors) apply per Vapi's terms.
- **Twilio** — provides the UK phone number and carries the underlying call/SMS.
- **Anthropic (Claude)** — powers the assistant's responses during the call, and
  optionally classifies a call's intent from the transcript after it ends.
- **Our infrastructure (Railway, UK/EU region)** — hosts the webhook server and the
  SQLite database. This is where call logs, bookings, and messages are stored
  long-term.

## Data minimisation

- No payment/card data is ever requested or stored.
- The database stores only what's needed to run the practice: contact details,
  appointment details, call outcomes, and transcripts/recordings for quality and
  dispute-resolution purposes.
- The admin page (internal staff only, protected by HTTP basic auth) is the only
  interface onto this data — there's no public API exposing call logs or bookings.

## Retention

Not yet automated for the demo build. Before onboarding a real client, agree and
implement a retention window (e.g. call recordings/transcripts purged after N months)
— see the "Next steps" note in docs/RUNBOOK.md.

## Caller-facing disclosure

The assistant discloses that calls may be recorded for quality purposes in its
opening greeting (see `src/assistant/systemPrompt.ts`), and discloses that it is an
AI assistant if asked directly. This is a UK GDPR-relevant control baked into the
assistant's instructions, not an afterthought.

## Hosting region

Deploy the webhook server and database to a UK or EU Railway region. Vapi and Twilio
are used as sub-processors for call handling; check their current data-residency
options if a client requires strict in-region processing end-to-end.
