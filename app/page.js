import Header from "@/app/components/Header";
import Footer from "@/app/components/Footer";
import JobsSection from "@/app/components/JobsSection";
import { CandidateApplicationForm, EmployerEnquiryForm } from "@/app/components/forms";
import { ACTIVE_JOBS } from "@/data/jobs";

export default function HomePage() {
  return (
    <>
      <Header />
      <main>
        <section className="hero section">
          <div className="container hero-grid">
            <div>
              <p className="eyebrow">Sentinel Recruitment Group</p>
              <h1>Premium Manchester Recruitment for Cleaning & Security Teams</h1>
              <p className="lead">
                We help employers hire dependable staff quickly while making it simple for candidates
                to apply for live roles.
              </p>
              <div className="cta-row">
                <a href="/employers" className="btn btn-primary">Request Staff</a>
                <a href="/candidates" className="btn btn-secondary">View Jobs</a>
              </div>
            </div>
            <div className="hero-panel">
              <p>Manchester-only active jobs</p>
              <h2>{ACTIVE_JOBS.length} Live Roles</h2>
              <p>Cleaning and security vacancies updated in one place.</p>
            </div>
          </div>
        </section>

        <section className="section">
          <div className="container cards-2">
            <article className="card">
              <h2>For Employers</h2>
              <p>Fast access to vetted cleaning and security candidates across Manchester.</p>
              <a href="/employers" className="text-link">Request staff →</a>
            </article>
            <article className="card">
              <h2>For Candidates</h2>
              <p>Apply quickly to active jobs and hear back from the team directly.</p>
              <a href="/candidates" className="text-link">Apply for jobs →</a>
            </article>
          </div>
        </section>

        <section className="section section-alt">
          <div className="container cards-3">
            <article className="card"><h3>Services</h3><p>Cleaning and security staffing support tailored to employer demand.</p></article>
            <article className="card"><h3>Why choose Sentinel</h3><p>Professional communication, quality screening, and quick turnaround.</p></article>
            <article className="card"><h3>How it works</h3><p>Tell us your requirement, we shortlist, you interview, and we place.</p></article>
          </div>
        </section>

        <JobsSection jobs={ACTIVE_JOBS} />

        <section className="section section-alt">
          <div className="container split">
            <div>
              <h2>Employer Enquiry</h2>
              <p>Tell us what staffing support you need.</p>
              <EmployerEnquiryForm />
            </div>
            <div>
              <h2>Candidate Application</h2>
              <p>Apply directly for one of our current Manchester roles.</p>
              <CandidateApplicationForm jobs={ACTIVE_JOBS} />
            </div>
          </div>
        </section>

        <section className="section">
          <div className="container card contact-strip">
            <h2>Contact Sentinel</h2>
            <p>Email info@sentinelrecruitmentgroup.com for immediate support.</p>
            <a href="/contact" className="btn btn-primary">Open Contact Page</a>
          </div>
        </section>
      </main>
      <Footer />
    </>
  );
}
