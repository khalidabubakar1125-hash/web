/**
 * Vapi tool ("function") definitions. Every practice's assistant gets the same four
 * tools — the practice-specific data lives in the config, not here. Vapi calls our
 * webhook (server.url) with the chosen tool name + arguments whenever the model
 * decides to invoke one of these mid-call.
 */
export function buildToolDefinitions(webhookUrl: string, serverSecret: string) {
  const server = { url: webhookUrl, secret: serverSecret };

  return [
    {
      type: "function" as const,
      server,
      function: {
        name: "check_availability",
        description:
          "Look up real upcoming appointment slots for a given treatment. Always call this before promising a specific time — never invent availability.",
        parameters: {
          type: "object",
          properties: {
            treatmentId: {
              type: "string",
              description:
                "The treatment id from the practice's treatment list, e.g. 'hygiene', 'new-patient-exam', 'emergency'.",
            },
            preferredDate: {
              type: "string",
              description:
                "Caller's preferred date if given, as an ISO date (YYYY-MM-DD). Omit if they have no preference.",
            },
            timeOfDay: {
              type: "string",
              enum: ["morning", "afternoon", "evening", "any"],
              description: "Caller's preferred time of day, if mentioned.",
            },
          },
          required: ["treatmentId"],
        },
      },
    },
    {
      type: "function" as const,
      server,
      function: {
        name: "book_appointment",
        description:
          "Confirms a booking for a specific slot returned by check_availability. Only call this after the caller has explicitly chosen one of the offered times.",
        parameters: {
          type: "object",
          properties: {
            slotId: {
              type: "string",
              description: "The exact slotId of the chosen slot, as returned by check_availability.",
            },
            treatmentId: { type: "string", description: "The treatment id being booked." },
            patientName: { type: "string", description: "Full name of the patient." },
            patientPhone: {
              type: "string",
              description: "Patient's mobile number for the SMS confirmation, in a UK format.",
            },
          },
          required: ["slotId", "treatmentId", "patientName", "patientPhone"],
        },
      },
    },
    {
      type: "function" as const,
      server,
      function: {
        name: "take_message",
        description:
          "Logs a message for the practice to follow up on, for anything the assistant can't resolve directly on the call (cancellations, reschedules, clinical questions, complaints, etc).",
        parameters: {
          type: "object",
          properties: {
            callerName: { type: "string", description: "Caller's name." },
            callerPhone: { type: "string", description: "Best contact number for the caller." },
            reason: { type: "string", description: "Short summary of what they need." },
          },
          required: ["callerName", "callerPhone", "reason"],
        },
      },
    },
    {
      type: "function" as const,
      server,
      function: {
        name: "flag_emergency",
        description:
          "Logs a dental emergency so the practice sees it immediately. Call this any time the triage rules in your instructions identify a genuine emergency, in addition to any booking or message you take.",
        parameters: {
          type: "object",
          properties: {
            description: { type: "string", description: "Brief description of the emergency." },
            severity: {
              type: "string",
              enum: ["urgent", "critical"],
              description: "'critical' if breathing/swallowing is affected or advised to call 999, otherwise 'urgent'.",
            },
            callerName: { type: "string", description: "Caller's name, if given." },
            callerPhone: { type: "string", description: "Caller's contact number, if given." },
          },
          required: ["description", "severity"],
        },
      },
    },
  ];
}
