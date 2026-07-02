const VAPI_BASE_URL = process.env.VAPI_BASE_URL ?? "https://api.vapi.ai";

function apiKey(): string {
  const key = process.env.VAPI_API_KEY;
  if (!key) throw new Error("VAPI_API_KEY is not set");
  return key;
}

async function vapiRequest<T>(path: string, init: RequestInit = {}): Promise<T> {
  const res = await fetch(`${VAPI_BASE_URL}${path}`, {
    ...init,
    headers: {
      Authorization: `Bearer ${apiKey()}`,
      "Content-Type": "application/json",
      ...init.headers,
    },
  });

  if (!res.ok) {
    const body = await res.text().catch(() => "");
    throw new Error(`Vapi API ${init.method ?? "GET"} ${path} failed: ${res.status} ${body}`);
  }

  return (await res.json()) as T;
}

export interface VapiAssistant {
  id: string;
  [key: string]: unknown;
}

export function createAssistant(payload: Record<string, unknown>): Promise<VapiAssistant> {
  return vapiRequest<VapiAssistant>("/assistant", {
    method: "POST",
    body: JSON.stringify(payload),
  });
}

export function updateAssistant(
  assistantId: string,
  payload: Record<string, unknown>,
): Promise<VapiAssistant> {
  return vapiRequest<VapiAssistant>(`/assistant/${assistantId}`, {
    method: "PATCH",
    body: JSON.stringify(payload),
  });
}

export interface VapiPhoneNumber {
  id: string;
  [key: string]: unknown;
}

/** Imports an already-purchased Twilio UK number into Vapi so it can route calls to an assistant. */
export function importTwilioPhoneNumber(params: {
  twilioAccountSid: string;
  twilioAuthToken: string;
  twilioPhoneNumber: string;
  assistantId: string;
  name?: string;
}): Promise<VapiPhoneNumber> {
  return vapiRequest<VapiPhoneNumber>("/phone-number", {
    method: "POST",
    body: JSON.stringify({
      provider: "twilio",
      number: params.twilioPhoneNumber,
      twilioAccountSid: params.twilioAccountSid,
      twilioAuthToken: params.twilioAuthToken,
      assistantId: params.assistantId,
      name: params.name,
    }),
  });
}

export function updatePhoneNumberAssistant(
  phoneNumberId: string,
  assistantId: string,
): Promise<VapiPhoneNumber> {
  return vapiRequest<VapiPhoneNumber>(`/phone-number/${phoneNumberId}`, {
    method: "PATCH",
    body: JSON.stringify({ assistantId }),
  });
}
