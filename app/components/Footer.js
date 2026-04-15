import { BUSINESS } from "@/lib/constants";

export default function Footer() {
  return (
    <footer className="site-footer">
      <div className="container footer-grid">
        <div>
          <p className="wordmark small">SENTINEL</p>
          <p>{BUSINESS.name}</p>
          <p>Manchester-focused cleaning and security recruitment.</p>
        </div>
        <div>
          <p>
            <strong>Email:</strong> {BUSINESS.email}
          </p>
          <p>
            <strong>Domain:</strong> {BUSINESS.domain}
          </p>
          <p>
            <strong>Coverage:</strong> {BUSINESS.locationFocus}
          </p>
        </div>
      </div>
    </footer>
  );
}
