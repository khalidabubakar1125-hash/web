import { readFileSync, readdirSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";
import yaml from "js-yaml";
import { practiceConfigSchema, type PracticeConfig } from "./schema.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
export const CONFIGS_DIR = path.resolve(__dirname, "../../configs");

function parseConfigFile(filePath: string): PracticeConfig {
  const raw = readFileSync(filePath, "utf8");
  const data = yaml.load(raw);
  const result = practiceConfigSchema.safeParse(data);
  if (!result.success) {
    throw new Error(
      `Invalid practice config at ${filePath}:\n${result.error.issues
        .map((i) => `  - ${i.path.join(".")}: ${i.message}`)
        .join("\n")}`,
    );
  }
  return result.data;
}

/** Loads a single practice config by its slug (filename without extension, or matching `slug` field). */
export function loadPracticeConfig(slug: string): PracticeConfig {
  const files = readdirSync(CONFIGS_DIR).filter(
    (f) => f.endsWith(".yaml") || f.endsWith(".yml"),
  );
  for (const file of files) {
    const config = parseConfigFile(path.join(CONFIGS_DIR, file));
    if (config.slug === slug) return config;
  }
  throw new Error(`No practice config found with slug "${slug}" in ${CONFIGS_DIR}`);
}

/** Loads every practice config found in /configs. Adding a new practice = adding a new file here. */
export function loadAllPracticeConfigs(): PracticeConfig[] {
  const files = readdirSync(CONFIGS_DIR).filter(
    (f) => f.endsWith(".yaml") || f.endsWith(".yml"),
  );
  return files.map((file) => parseConfigFile(path.join(CONFIGS_DIR, file)));
}

/** Persists a config back to its YAML file, e.g. after the assistant builder records a new assistantId. */
export function savePracticeConfig(config: PracticeConfig): void {
  const files = readdirSync(CONFIGS_DIR).filter(
    (f) => f.endsWith(".yaml") || f.endsWith(".yml"),
  );
  const file = files.find((f) => parseConfigFile(path.join(CONFIGS_DIR, f)).slug === config.slug);
  if (!file) throw new Error(`Cannot save: no existing config file for slug "${config.slug}"`);
  const dump = yaml.dump(config, { lineWidth: 100, noRefs: true });
  writeFileSync(path.join(CONFIGS_DIR, file), dump, "utf8");
}
