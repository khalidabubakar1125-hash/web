const INTENT_LABELS = [
  "booking",
  "booking_enquiry",
  "price_question",
  "hours_question",
  "emergency",
  "message",
  "cancellation",
  "other",
];

/**
 * Best-effort intent label for calls where no function was ever called (e.g. the
 * caller just asked a question and hung up). Optional: silently returns null if
 * ANTHROPIC_API_KEY isn't set or the request fails, so it never blocks call logging.
 */
export async function classifyIntentFromTranscript(transcript: string | null): Promise<string | null> {
  const apiKey = process.env.ANTHROPIC_API_KEY;
  if (!apiKey || !transcript) return null;

  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 6000);

  try {
    const res = await fetch("https://api.anthropic.com/v1/messages", {
      method: "POST",
      headers: {
        "x-api-key": apiKey,
        "anthropic-version": "2023-06-01",
        "content-type": "application/json",
      },
      body: JSON.stringify({
        model: process.env.CLASSIFY_MODEL ?? "claude-3-5-haiku-20241022",
        max_tokens: 10,
        messages: [
          {
            role: "user",
            content:
              `Classify this dental receptionist phone call transcript into exactly one of: ` +
              `${INTENT_LABELS.join(", ")}. Reply with only the label, nothing else.\n\n` +
              `Transcript:\n${transcript.slice(0, 4000)}`,
          },
        ],
      }),
      signal: controller.signal,
    });

    if (!res.ok) return null;
    const data = (await res.json()) as { content?: { text?: string }[] };
    const label = data.content?.[0]?.text?.trim().toLowerCase();
    return label && INTENT_LABELS.includes(label) ? label : null;
  } catch {
    return null;
  } finally {
    clearTimeout(timeout);
  }
}
