import type { PracticeConfig } from "../../config/schema.js";
import type { CallRow } from "../../db/calls.js";
import type { MessageRow } from "../../db/messages.js";
import type { BookingRecord } from "../../booking/types.js";

function escapeHtml(value: unknown): string {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function outcomeBadge(outcome: string): string {
  const colors: Record<string, string> = {
    booked: "#1a7f37",
    emergency_escalated: "#cf222e",
    message_taken: "#9a6700",
    info_given: "#57606a",
    in_progress: "#0969da",
    no_action: "#8c959f",
  };
  const color = colors[outcome] ?? "#57606a";
  return `<span style="background:${color};color:#fff;padding:2px 8px;border-radius:12px;font-size:12px;white-space:nowrap">${escapeHtml(
    outcome.replace(/_/g, " "),
  )}</span>`;
}

function treatmentName(config: PracticeConfig, treatmentId: string): string {
  return config.treatments.find((t) => t.id === treatmentId)?.name ?? treatmentId;
}

export function renderAdminPage(params: {
  practices: PracticeConfig[];
  activePractice: PracticeConfig;
  calls: CallRow[];
  bookings: BookingRecord[];
  messages: MessageRow[];
}): string {
  const { practices, activePractice, calls, bookings, messages } = params;

  const practiceOptions = practices
    .map(
      (p) =>
        `<option value="${escapeHtml(p.slug)}" ${p.slug === activePractice.slug ? "selected" : ""}>${escapeHtml(p.name)}</option>`,
    )
    .join("");

  const callRows = calls
    .map((c) => {
      const transcriptId = `t-${c.id}`;
      return `<tr>
        <td>${escapeHtml(new Date(c.created_at).toLocaleString("en-GB", { timeZone: "Europe/London" }))}</td>
        <td>${escapeHtml(c.caller_number ?? "—")}</td>
        <td>${c.duration_seconds != null ? `${c.duration_seconds}s` : "—"}</td>
        <td>${escapeHtml(c.intent ?? "—")}</td>
        <td>${outcomeBadge(c.outcome)}</td>
        <td>${escapeHtml(c.summary ?? "")}</td>
        <td>${
          c.transcript
            ? `<button onclick="document.getElementById('${transcriptId}').style.display='block'" style="cursor:pointer">View</button>
               <div id="${transcriptId}" style="display:none;white-space:pre-wrap;max-width:480px;margin-top:6px;font-size:12px;color:#57606a">${escapeHtml(c.transcript)}</div>`
            : "—"
        }</td>
      </tr>`;
    })
    .join("");

  const bookingRows = bookings
    .map(
      (b) => `<tr>
        <td>${escapeHtml(new Date(b.startTime).toLocaleString("en-GB", { timeZone: "Europe/London" }))}</td>
        <td>${escapeHtml(b.patientName)}</td>
        <td>${escapeHtml(b.patientPhone)}</td>
        <td>${escapeHtml(treatmentName(activePractice, b.treatmentId))}</td>
        <td>${escapeHtml(b.dentist)}</td>
        <td>${escapeHtml(b.status)}</td>
      </tr>`,
    )
    .join("");

  const messageRows = messages
    .map(
      (m) => `<tr>
        <td>${escapeHtml(new Date(m.created_at).toLocaleString("en-GB", { timeZone: "Europe/London" }))}</td>
        <td>${escapeHtml(m.caller_name ?? "—")}</td>
        <td>${escapeHtml(m.caller_phone ?? "—")}</td>
        <td>${m.urgent ? '<strong style="color:#cf222e">URGENT</strong> ' : ""}${escapeHtml(m.reason)}</td>
      </tr>`,
    )
    .join("");

  return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>${escapeHtml(activePractice.name)} — Call log</title>
<style>
  body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; margin: 0; background: #f6f8fa; color: #1f2328; }
  header { background: #1f2328; color: #fff; padding: 16px 24px; display: flex; align-items: center; justify-content: space-between; }
  header h1 { font-size: 18px; margin: 0; }
  main { padding: 24px; max-width: 1200px; margin: 0 auto; }
  section { background: #fff; border: 1px solid #d0d7de; border-radius: 8px; padding: 16px 20px; margin-bottom: 24px; }
  h2 { font-size: 15px; margin: 0 0 12px; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th, td { text-align: left; padding: 6px 8px; border-bottom: 1px solid #eaeef2; vertical-align: top; }
  th { color: #57606a; font-weight: 600; }
  select { padding: 4px 8px; font-size: 13px; }
  .empty { color: #8c959f; font-size: 13px; padding: 8px 0; }
</style>
</head>
<body>
<header>
  <h1>${escapeHtml(activePractice.name)} — Call log &amp; bookings</h1>
  <form method="get" style="margin:0">
    <select name="practice" onchange="this.form.submit()">${practiceOptions}</select>
  </form>
</header>
<main>
  <section>
    <h2>Recent calls</h2>
    ${
      calls.length
        ? `<table><thead><tr><th>Time</th><th>Caller</th><th>Duration</th><th>Intent</th><th>Outcome</th><th>Summary</th><th>Transcript</th></tr></thead><tbody>${callRows}</tbody></table>`
        : `<div class="empty">No calls logged yet.</div>`
    }
  </section>
  <section>
    <h2>Bookings</h2>
    ${
      bookings.length
        ? `<table><thead><tr><th>Time</th><th>Patient</th><th>Phone</th><th>Treatment</th><th>Dentist</th><th>Status</th></tr></thead><tbody>${bookingRows}</tbody></table>`
        : `<div class="empty">No bookings yet.</div>`
    }
  </section>
  <section>
    <h2>Messages</h2>
    ${
      messages.length
        ? `<table><thead><tr><th>Time</th><th>Name</th><th>Phone</th><th>Reason</th></tr></thead><tbody>${messageRows}</tbody></table>`
        : `<div class="empty">No messages yet.</div>`
    }
  </section>
</main>
</body>
</html>`;
}
