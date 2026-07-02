import "dotenv/config";
import { loadAllPracticeConfigs } from "../src/config/loader.js";
import { seedSlots } from "../src/booking/seed.js";

const configs = loadAllPracticeConfigs();

if (configs.length === 0) {
  console.log("No practice configs found in /configs — nothing to seed.");
  process.exit(0);
}

for (const config of configs) {
  const created = seedSlots(config);
  console.log(`Seeded ${created} mock availability slots for ${config.name} (${config.slug})`);
}
