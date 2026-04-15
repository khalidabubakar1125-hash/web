import Header from "@/app/components/Header";
import Footer from "@/app/components/Footer";
import { EmployerEnquiryForm } from "@/app/components/forms";
import { BUSINESS } from "@/lib/constants";

export const metadata = {
  title: "Contact | Sentinel Recruitment Group",
};

export default function ContactPage() {
  return (
    <>
      <Header />
      <main className="section">
        <div className="container split">
          <section className="card">
            <h1>Contact Sentinel Recruitment Group</h1>
            <p>
              <strong>Email:</strong> {BUSINESS.email}
            </p>
            <p>
              <strong>Domain:</strong> {BUSINESS.domain}
            </p>
            <p>
              <strong>Location focus:</strong> Manchester
            </p>
            <p>Use the form for employer requirements and general enquiries.</p>
          </section>

          <section className="card">
            <h2>Contact Form</h2>
            <EmployerEnquiryForm />
          </section>
        </div>
      </main>
      <Footer />
    </>
  );
}
