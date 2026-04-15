# Sentinel Recruitment Group Website

Premium, launch-ready Next.js website for Sentinel Recruitment Group, focused on Manchester cleaning and security recruitment.

## Quick start

```bash
npm install
npm run dev
```

Open http://localhost:3000.

## Pages

- `/` Home
- `/employers`
- `/candidates`
- `/contact`

## Editing jobs

Update `data/jobs.js`.

Each role uses:

```js
{
  id: string,
  title: string,
  pay: string,
  type: "Cleaning" | "Security",
  location: "Manchester",
  description: string,
  active: boolean
}
```

## Google Sheets integration setup

1. In `lib/constants.js`, paste your Apps Script web app endpoint(s):

```js
export const GOOGLE_SCRIPT_URL_CONTACT = "";
export const GOOGLE_SCRIPT_URL_APPLICATION = "";
```

You can use the same URL for both if your script uses `formType`.

2. Deploy the script as a Web App (`Anyone with link`).

### Google Apps Script starter (paste into script editor)

```javascript
const SPREADSHEET_NAME = 'Sentinel Website Leads';

function doPost(e) {
  const payload = JSON.parse(e.postData.contents || '{}');
  const ss = SpreadsheetApp.getActiveSpreadsheet();

  if (payload.formType === 'client_enquiry') {
    const sheet = ss.getSheetByName('Client Enquiries');
    sheet.appendRow([
      payload.submittedAt || '',
      payload.formType || '',
      payload.name || '',
      payload.email || '',
      payload.company || '',
      payload.message || ''
    ]);
  } else if (payload.formType === 'candidate_application') {
    const sheet = ss.getSheetByName('Candidate Applications');
    sheet.appendRow([
      payload.submittedAt || '',
      payload.formType || '',
      payload.selectedJob || '',
      payload.fullName || '',
      payload.email || '',
      payload.phone || '',
      payload.cvLink || ''
    ]);
  }

  return ContentService
    .createTextOutput(JSON.stringify({ ok: true }))
    .setMimeType(ContentService.MimeType.JSON);
}
```

## Vercel deployment

1. Push repository to GitHub.
2. Import project in Vercel.
3. Framework preset: `Next.js` (auto).
4. Build command: `npm run build`.
5. Output directory: leave default.
6. Add optional environment variables if needed later.
7. Deploy.

