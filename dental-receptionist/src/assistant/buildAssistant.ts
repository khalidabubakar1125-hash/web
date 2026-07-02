import type { PracticeConfig } from "../config/schema.js";
import { buildSystemPrompt } from "./systemPrompt.js";
import { buildToolDefinitions } from "./functions.js";

/**
 * Turns a practice config into the payload Vapi's POST/PATCH /assistant expects.
 * This is the whole "assistant-as-config" contract: nothing here is practice-specific
 * except what comes out of `config`.
 */
export function buildAssistantPayload(config: PracticeConfig, opts: { webhookUrl: string }) {
  const serverSecret = requiredEnv("VAPI_SERVER_SECRET");

  // Swap point: Vapi supports several LLM providers. We default to Claude via Vapi's
  // "anthropic" provider per the brief; if that's not enabled on the Vapi plan in use,
  // set VAPI_LLM_PROVIDER=openai / VAPI_LLM_MODEL=gpt-4o (or whatever Vapi currently
  // recommends) in .env — nothing else about the assistant needs to change.
  const llmProvider = process.env.VAPI_LLM_PROVIDER ?? "anthropic";
  const llmModel = process.env.VAPI_LLM_MODEL ?? "claude-3-5-sonnet-20241022";

  return {
    name: `${config.name} — AI Receptionist`,
    metadata: { practiceSlug: config.slug },
    firstMessageMode: "assistant-speaks-first-with-model-generated-message",
    model: {
      provider: llmProvider,
      model: llmModel,
      temperature: 0.4,
      messages: [{ role: "system", content: buildSystemPrompt(config) }],
      tools: buildToolDefinitions(opts.webhookUrl, serverSecret),
    },
    voice: {
      provider: config.voice.provider,
      voiceId: config.voice.voiceId,
    },
    transcriber: {
      provider: "deepgram",
      model: "nova-2",
      language: "en-GB",
    },
    server: {
      url: opts.webhookUrl,
      secret: serverSecret,
    },
    recordingEnabled: true,
    endCallFunctionEnabled: true,
    silenceTimeoutSeconds: 30,
    maxDurationSeconds: 900,
  };
}

function requiredEnv(name: string): string {
  const value = process.env[name];
  if (!value) throw new Error(`${name} is not set`);
  return value;
}
