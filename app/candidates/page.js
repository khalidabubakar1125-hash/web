import Header from "@/app/components/Header";
import Footer from "@/app/components/Footer";
import JobsSection from "@/app/components/JobsSection";
import { CandidateApplicationForm } from "@/app/components/forms";
import { ACTIVE_JOBS } from "@/data/jobs";

export const metadata = {
  title: "Candidates | Sentinel Recruitment Group",
};

export default function CandidatesPage() {
  return (
    <>
      <Header />
      <main>
        <section className="section">
          <div className="container stack-lg">
            <h1>Candidate Jobs & Applications</h1>
            <p className="lead">
              View live Manchester jobs in cleaning and security, filter roles, and apply quickly.
            </p>
          </div>
        </section>

        <JobsSection jobs={ACTIVE_JOBS} compact />

        <section className="section section-alt">
          <div className="container card">
            <h2>Apply in minutes</h2>
            <p>Select your role and send your application in one step.</p>
            <CandidateApplicationForm jobs={ACTIVE_JOBS} />
          </div>
        </section>
      </main>
      <Footer />
    </>
  );
}
