import { loadAllPracticeConfigs } from "../config/loader.js";
import type { PracticeConfig } from "../config/schema.js";

export interface PracticeRegistry {
  byAssistantId: Map<string, PracticeConfig>;
  byPhoneNumberId: Map<string, PracticeConfig>;
  bySlug: Map<string, PracticeConfig>;
  all: PracticeConfig[];
}

export function buildPracticeRegistry(): PracticeRegistry {
  const configs = loadAllPracticeConfigs();
  return {
    byAssistantId: new Map(
      configs.filter((c) => c.vapi.assistantId).map((c) => [c.vapi.assistantId!, c]),
    ),
    byPhoneNumberId: new Map(
      configs.filter((c) => c.vapi.phoneNumberId).map((c) => [c.vapi.phoneNumberId!, c]),
    ),
    bySlug: new Map(configs.map((c) => [c.slug, c])),
    all: configs,
  };
}

/**
 * Resolves which practice a webhook event belongs to. Tries the strongest signal
 * first (assistant id), falling back to phone number, then explicit metadata, then
 * a single-practice DEFAULT_PRACTICE_SLUG for the demo deployment. This is what lets
 * adding practice #2 be "just another config file" — the server never hardcodes a slug.
 */
export function resolvePractice(
  registry: PracticeRegistry,
  ids: { assistantId?: string | null; phoneNumberId?: string | null; metadataSlug?: string | null },
): PracticeConfig | undefined {
  if (ids.assistantId && registry.byAssistantId.has(ids.assistantId)) {
    return registry.byAssistantId.get(ids.assistantId);
  }
  if (ids.phoneNumberId && registry.byPhoneNumberId.has(ids.phoneNumberId)) {
    return registry.byPhoneNumberId.get(ids.phoneNumberId);
  }
  if (ids.metadataSlug && registry.bySlug.has(ids.metadataSlug)) {
    return registry.bySlug.get(ids.metadataSlug);
  }
  const defaultSlug = process.env.DEFAULT_PRACTICE_SLUG;
  if (defaultSlug && registry.bySlug.has(defaultSlug)) {
    return registry.bySlug.get(defaultSlug);
  }
  return undefined;
}
