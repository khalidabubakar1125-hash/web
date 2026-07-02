/**
 * Vapi's webhook payload shapes have shifted across API versions and aren't fully
 * pinned down in public docs at the time of writing. These extractors read every
 * field defensively (several possible locations) rather than assuming one exact
 * schema, so the server keeps working if Vapi tweaks field names/nesting.
 */

export interface NormalizedToolCall {
  id: string;
  name: string;
  arguments: Record<string, unknown>;
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type AnyMessage = any;

export function extractToolCalls(message: AnyMessage): NormalizedToolCall[] {
  const list: AnyMessage[] = message.toolCallList ?? message.toolCalls ?? [];
  return list.map((tc) => {
    const fn = tc.function ?? tc;
    const id: string = tc.id ?? tc.toolCallId ?? "";
    const name: string = fn.name ?? tc.name ?? "";
    let args: unknown = fn.arguments ?? tc.parameters ?? tc.arguments ?? {};
    if (typeof args === "string") {
      try {
        args = JSON.parse(args);
      } catch {
        args = {};
      }
    }
    return { id, name, arguments: (args ?? {}) as Record<string, unknown> };
  });
}

export function extractCallerNumber(message: AnyMessage): string | null {
  return message.call?.customer?.number ?? message.customer?.number ?? null;
}

export function extractCallId(message: AnyMessage): string {
  return message.call?.id ?? message.callId ?? "";
}

export function extractAssistantId(message: AnyMessage): string | null {
  return message.call?.assistantId ?? message.assistant?.id ?? null;
}

export function extractPhoneNumberId(message: AnyMessage): string | null {
  return message.call?.phoneNumberId ?? null;
}

export function extractMetadataSlug(message: AnyMessage): string | null {
  return (
    message.call?.assistant?.metadata?.practiceSlug ??
    message.assistant?.metadata?.practiceSlug ??
    null
  );
}

export interface NormalizedEndOfCallReport {
  transcript: string | null;
  summary: string | null;
  recordingUrl: string | null;
  durationSeconds: number | null;
  endedReason: string | null;
  endedAt: string | null;
}

export function normalizeEndOfCallReport(message: AnyMessage): NormalizedEndOfCallReport {
  const artifact = message.artifact ?? {};
  const recording = artifact.recording ?? {};

  return {
    transcript: message.transcript ?? artifact.transcript ?? null,
    summary: message.summary ?? message.analysis?.summary ?? null,
    recordingUrl:
      message.recordingUrl ??
      artifact.recordingUrl ??
      recording.stereoUrl ??
      recording.mono?.combinedUrl ??
      recording.url ??
      null,
    durationSeconds: message.durationSeconds ?? computeDuration(message) ?? null,
    endedReason: message.endedReason ?? null,
    endedAt: message.call?.endedAt ?? message.endedAt ?? null,
  };
}

function computeDuration(message: AnyMessage): number | null {
  const start = message.call?.startedAt ?? message.startedAt;
  const end = message.call?.endedAt ?? message.endedAt;
  if (!start || !end) return null;
  const ms = new Date(end).getTime() - new Date(start).getTime();
  return ms > 0 ? Math.round(ms / 1000) : null;
}
