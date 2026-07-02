import "dotenv/config";
import { loadPracticeConfig, savePracticeConfig } from "../config/loader.js";
import { buildAssistantPayload } from "./buildAssistant.js";
import {
  createAssistant,
  updateAssistant,
  importTwilioPhoneNumber,
  updatePhoneNumberAssistant,
} from "./vapiClient.js";

/**
 * Creates (or updates) a practice's Vapi assistant from its config file, and links a
 * Twilio phone number to it. Idempotent: re-running after editing the config PATCHes
 * the existing assistant instead of creating a duplicate.
 *
 * Usage: npm run create-assistant -- <practice-slug>
 */
async function main() {
  const slug = process.argv[2];
  if (!slug) {
    console.error("Usage: npm run create-assistant -- <practice-slug>");
    console.error("Example: npm run create-assistant -- brightside-demo");
    process.exit(1);
  }

  const webhookBase = requiredEnv("PUBLIC_WEBHOOK_BASE_URL"); // e.g. https://your-app.up.railway.app
  const webhookUrl = `${webhookBase.replace(/\/$/, "")}/webhook/vapi`;

  const config = loadPracticeConfig(slug);
  const payload = buildAssistantPayload(config, { webhookUrl });

  let assistantId = config.vapi.assistantId;
  if (assistantId) {
    console.log(`Updating existing assistant ${assistantId} for ${config.name}...`);
    await updateAssistant(assistantId, payload);
    console.log("Assistant updated.");
  } else {
    console.log(`Creating a new assistant for ${config.name}...`);
    const assistant = await createAssistant(payload);
    assistantId = assistant.id;
    config.vapi.assistantId = assistantId;
    savePracticeConfig(config);
    console.log(`Assistant created: ${assistantId} (saved to configs/${slug}.yaml)`);
  }

  if (!config.vapi.phoneNumberId) {
    const twilioSid = process.env.TWILIO_ACCOUNT_SID;
    const twilioToken = process.env.TWILIO_AUTH_TOKEN;
    const twilioNumber = config.sms.fromNumber ?? process.env.TWILIO_PHONE_NUMBER;

    if (twilioSid && twilioToken && twilioNumber) {
      console.log(`Importing Twilio number ${twilioNumber} into Vapi...`);
      const phoneNumber = await importTwilioPhoneNumber({
        twilioAccountSid: twilioSid,
        twilioAuthToken: twilioToken,
        twilioPhoneNumber: twilioNumber,
        assistantId: assistantId!,
        name: `${config.name} demo line`,
      });
      config.vapi.phoneNumberId = phoneNumber.id;
      savePracticeConfig(config);
      console.log(`Phone number imported and linked: ${phoneNumber.id}`);
    } else {
      console.log(
        "\nNo phone number linked yet. Set TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN and " +
          "TWILIO_PHONE_NUMBER in .env and re-run this command, or import the number " +
          "manually in the Vapi dashboard and paste its id into vapi.phoneNumberId in " +
          `configs/${slug}.yaml. See docs/RUNBOOK.md.`,
      );
    }
  } else {
    console.log(`Ensuring phone number ${config.vapi.phoneNumberId} points at assistant ${assistantId}...`);
    await updatePhoneNumberAssistant(config.vapi.phoneNumberId, assistantId!);
    console.log("Phone number linked.");
  }

  console.log(`\nDone. Assistant id: ${assistantId}`);
}

function requiredEnv(name: string): string {
  const value = process.env[name];
  if (!value) throw new Error(`${name} is not set in .env`);
  return value;
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
