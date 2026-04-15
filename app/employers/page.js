import Header from "@/app/components/Header";
import Footer from "@/app/components/Footer";
import { EmployerEnquiryForm } from "@/app/components/forms";

export const metadata = {
  title: "Employers | Sentinel Recruitment Group",
};

export default function EmployersPage() {
  return (
    <>
      <Header />
      <main className="section">
        <div className="container stack-lg">
          <h1>Employer Staffing Support</h1>
          <p className="lead">
            Sentinel Recruitment Group supplies reliable cleaning and security staff for Manchester
            businesses that need quality and speed.
          </p>

          <section className="cards-2">
            <article className="card">
              <h2>Cleaning Staff</h2>
              <p>Office, school, hotel and commercial cleaning placements.</p>
            </article>
            <article className="card">
              <h2>Security Staff</h2>
              <p>SIA and site-based security coverage from trusted candidates.</p>
            </article>
          </section>

          <section className="cards-3">
            <article className="card"><h3>Why Sentinel</h3><p>Manchester focus, clear communication, and practical delivery.</p></article>
            <article className="card"><h3>Hiring process</h3><p>Share your brief, review shortlist, confirm start dates.</p></article>
            <article className="card"><h3>Fast response</h3><p>We handle urgent and planned hiring requirements.</p></article>
          </section>

          <section className="card">
            <h2>Request Staff</h2>
            <p>Complete the form and our team will contact you.</p>
            <EmployerEnquiryForm />
          </section>
        </div>
      </main>
      <Footer />
    </>
  );
}
