"use client";

import { useMemo, useState } from "react";

const FILTERS = ["All", "Cleaning", "Security"];

export default function JobsSection({ jobs, compact = false }) {
  const [filter, setFilter] = useState("All");

  const filteredJobs = useMemo(() => {
    if (filter === "All") return jobs;
    return jobs.filter((job) => job.type === filter);
  }, [jobs, filter]);

  return (
    <section className="section">
      <div className="container">
        <div className="section-head">
          <h2>Current Manchester Jobs</h2>
          <p>Simple, live listings split by cleaning and security roles.</p>
        </div>
        <div className="filters" role="tablist" aria-label="Job filters">
          {FILTERS.map((type) => (
            <button
              key={type}
              type="button"
              role="tab"
              aria-selected={filter === type}
              className={filter === type ? "active" : ""}
              onClick={() => setFilter(type)}
            >
              {type}
            </button>
          ))}
        </div>
        <div className={compact ? "jobs-grid compact" : "jobs-grid"}>
          {filteredJobs.map((job) => (
            <article key={job.id} className="job-card">
              <p className="chip">{job.type}</p>
              <h3>{job.title}</h3>
              <p>{job.description}</p>
              <ul>
                <li>
                  <strong>Pay:</strong> {job.pay}
                </li>
                <li>
                  <strong>Location:</strong> {job.location}
                </li>
              </ul>
            </article>
          ))}
        </div>
      </div>
    </section>
  );
}
