"use client";

import { useMemo, useState } from "react";
import { GOOGLE_SCRIPT_URL_APPLICATION, GOOGLE_SCRIPT_URL_CONTACT } from "@/lib/constants";

function postToScript(url, payload) {
  if (!url) {
    return Promise.resolve({ ok: true, mocked: true });
  }

  return fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
}

function isValidEmail(email) {
  return /\S+@\S+\.\S+/.test(email);
}

function isValidUrl(url) {
  if (!url) return false;
  try {
    new URL(url);
    return true;
  } catch {
    return false;
  }
}

export function EmployerEnquiryForm() {
  const [form, setForm] = useState({ name: "", email: "", company: "", message: "" });
  const [errors, setErrors] = useState({});
  const [status, setStatus] = useState("idle");

  function validate() {
    const nextErrors = {};
    if (!form.name.trim()) nextErrors.name = "Name is required.";
    if (!isValidEmail(form.email)) nextErrors.email = "Enter a valid email address.";
    if (!form.company.trim()) nextErrors.company = "Company is required.";
    if (!form.message.trim()) nextErrors.message = "Message is required.";
    return nextErrors;
  }

  async function handleSubmit(event) {
    event.preventDefault();
    setStatus("idle");

    const nextErrors = validate();
    setErrors(nextErrors);
    if (Object.keys(nextErrors).length) return;

    setStatus("submitting");
    const payload = {
      submittedAt: new Date().toISOString(),
      formType: "client_enquiry",
      name: form.name.trim(),
      email: form.email.trim(),
      company: form.company.trim(),
      message: form.message.trim(),
    };

    try {
      const response = await postToScript(GOOGLE_SCRIPT_URL_CONTACT, payload);
      if (!response.ok) throw new Error("Submission failed.");

      setStatus("success");
      setForm({ name: "", email: "", company: "", message: "" });
    } catch {
      setStatus("error");
    }
  }

  return (
    <form className="form" onSubmit={handleSubmit} noValidate>
      <div className="field-grid">
        <Field label="Name" error={errors.name}>
          <input
            value={form.name}
            onChange={(e) => setForm((prev) => ({ ...prev, name: e.target.value }))}
          />
        </Field>
        <Field label="Work email" error={errors.email}>
          <input
            type="email"
            value={form.email}
            onChange={(e) => setForm((prev) => ({ ...prev, email: e.target.value }))}
          />
        </Field>
      </div>
      <Field label="Company" error={errors.company}>
        <input
          value={form.company}
          onChange={(e) => setForm((prev) => ({ ...prev, company: e.target.value }))}
        />
      </Field>
      <Field label="What staff do you need?" error={errors.message}>
        <textarea
          rows={5}
          value={form.message}
          onChange={(e) => setForm((prev) => ({ ...prev, message: e.target.value }))}
        />
      </Field>
      <SubmitRow status={status} submittingText="Sending enquiry..." buttonText="Request Staff" />
    </form>
  );
}

export function CandidateApplicationForm({ jobs }) {
  const jobOptions = useMemo(() => jobs.map((job) => job.title), [jobs]);

  const [form, setForm] = useState({
    selectedJob: jobOptions[0] ?? "",
    fullName: "",
    email: "",
    phone: "",
    cvLink: "",
  });
  const [errors, setErrors] = useState({});
  const [status, setStatus] = useState("idle");

  function validate() {
    const nextErrors = {};
    if (!form.selectedJob.trim()) nextErrors.selectedJob = "Select a role.";
    if (!form.fullName.trim()) nextErrors.fullName = "Full name is required.";
    if (!isValidEmail(form.email)) nextErrors.email = "Enter a valid email address.";
    if (!form.phone.trim()) nextErrors.phone = "Phone is required.";
    if (!isValidUrl(form.cvLink)) nextErrors.cvLink = "Enter a valid CV link.";
    return nextErrors;
  }

  async function handleSubmit(event) {
    event.preventDefault();
    setStatus("idle");

    const nextErrors = validate();
    setErrors(nextErrors);
    if (Object.keys(nextErrors).length) return;

    setStatus("submitting");
    const payload = {
      submittedAt: new Date().toISOString(),
      formType: "candidate_application",
      selectedJob: form.selectedJob,
      fullName: form.fullName.trim(),
      email: form.email.trim(),
      phone: form.phone.trim(),
      cvLink: form.cvLink.trim(),
    };

    try {
      const response = await postToScript(GOOGLE_SCRIPT_URL_APPLICATION, payload);
      if (!response.ok) throw new Error("Submission failed.");

      setStatus("success");
      setForm({
        selectedJob: jobOptions[0] ?? "",
        fullName: "",
        email: "",
        phone: "",
        cvLink: "",
      });
    } catch {
      setStatus("error");
    }
  }

  return (
    <form className="form" onSubmit={handleSubmit} noValidate>
      <Field label="Selected role" error={errors.selectedJob}>
        <select
          value={form.selectedJob}
          onChange={(e) => setForm((prev) => ({ ...prev, selectedJob: e.target.value }))}
        >
          {jobOptions.map((jobTitle) => (
            <option key={jobTitle} value={jobTitle}>
              {jobTitle}
            </option>
          ))}
        </select>
      </Field>
      <div className="field-grid">
        <Field label="Full name" error={errors.fullName}>
          <input
            value={form.fullName}
            onChange={(e) => setForm((prev) => ({ ...prev, fullName: e.target.value }))}
          />
        </Field>
        <Field label="Email" error={errors.email}>
          <input
            type="email"
            value={form.email}
            onChange={(e) => setForm((prev) => ({ ...prev, email: e.target.value }))}
          />
        </Field>
      </div>
      <div className="field-grid">
        <Field label="Phone" error={errors.phone}>
          <input
            value={form.phone}
            onChange={(e) => setForm((prev) => ({ ...prev, phone: e.target.value }))}
          />
        </Field>
        <Field label="CV link" error={errors.cvLink}>
          <input
            placeholder="https://"
            value={form.cvLink}
            onChange={(e) => setForm((prev) => ({ ...prev, cvLink: e.target.value }))}
          />
        </Field>
      </div>
      <SubmitRow status={status} submittingText="Submitting application..." buttonText="Apply Now" />
    </form>
  );
}

function Field({ label, error, children }) {
  return (
    <label>
      <span>{label}</span>
      {children}
      {error ? <small className="error">{error}</small> : null}
    </label>
  );
}

function SubmitRow({ status, submittingText, buttonText }) {
  return (
    <div className="submit-row">
      <button type="submit" disabled={status === "submitting"}>
        {status === "submitting" ? submittingText : buttonText}
      </button>
      {status === "success" ? <p className="success">Thanks. We have received your form.</p> : null}
      {status === "error" ? <p className="error">Something went wrong. Please try again.</p> : null}
    </div>
  );
}
