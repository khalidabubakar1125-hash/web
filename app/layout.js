import "./globals.css";

export const metadata = {
  title: "Sentinel Recruitment Group | Manchester Cleaning & Security Staffing",
  description:
    "Sentinel Recruitment Group connects Manchester employers with reliable cleaning and security staff.",
};

export default function RootLayout({ children }) {
  return (
    <html lang="en-GB">
      <body>{children}</body>
    </html>
  );
}
