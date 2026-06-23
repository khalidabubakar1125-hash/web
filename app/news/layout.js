export const metadata = {
  title: "World News Center",
  description: "Live breaking world news — politics, tech, science, sports, business, health, climate & more.",
  manifest: "/news-manifest.json",
  appleWebApp: {
    capable: true,
    title: "World News",
    statusBarStyle: "black-translucent",
  },
  other: {
    "apple-touch-icon": "/news-icon-192.png",
  },
};

export const viewport = {
  width: "device-width",
  initialScale: 1,
  maximumScale: 1,
  userScalable: false,
  viewportFit: "cover",
};

export default function NewsLayout({ children }) {
  return (
    <>
      <link rel="apple-touch-icon" href="/news-icon-192.png" />
      <link rel="manifest" href="/news-manifest.json" />
      {children}
    </>
  );
}
