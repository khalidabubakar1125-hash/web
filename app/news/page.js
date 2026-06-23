export const metadata = {
  title: "Global News Hub",
  description: "Live world news across every category, optimized for mobile.",
};

export default function NewsPage() {
  return (
    <>
      <link rel="manifest" href="/news-manifest.json" />
      <meta name="apple-mobile-web-app-capable" content="yes" />
      <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent" />
      <meta name="apple-mobile-web-app-title" content="World News" />
      <meta name="theme-color" content="#0a0a1a" />
      <NewsApp />
    </>
  );
}

function NewsApp() {
  return (
    <div id="news-root">
      <style>{`
        * { box-sizing: border-box; margin: 0; padding: 0; }
        :root {
          --bg: #0a0a1a;
          --surface: #111128;
          --card: #16162e;
          --border: #2a2a4a;
          --text: #f0f0ff;
          --muted: #8888aa;
          --accent1: #ff4d6d;
          --accent2: #4cc9f0;
          --accent3: #7b2fff;
          --accent4: #f72585;
          --accent5: #4361ee;
          --accent6: #06d6a0;
          --accent7: #ffd60a;
          --accent8: #ef476f;
          --accent9: #fb5607;
          --accent10: #3a86ff;
        }
        html, body { background: var(--bg); color: var(--text); font-family: -apple-system, BlinkMacSystemFont, 'SF Pro Display', 'Segoe UI', sans-serif; }
        #news-root { max-width: 430px; min-height: 100vh; margin: 0 auto; display: flex; flex-direction: column; }

        /* HEADER */
        .news-header {
          background: linear-gradient(135deg, #0d0d2b 0%, #1a0533 50%, #0d1b3e 100%);
          padding: 52px 16px 12px;
          position: sticky; top: 0; z-index: 100;
          border-bottom: 1px solid var(--border);
          backdrop-filter: blur(20px);
        }
        .header-top { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }
        .logo-area { display: flex; align-items: center; gap: 10px; }
        .logo-icon { width: 38px; height: 38px; background: linear-gradient(135deg, #ff4d6d, #7b2fff); border-radius: 10px; display: flex; align-items: center; justify-content: center; font-size: 20px; }
        .logo-text { font-size: 16px; font-weight: 800; background: linear-gradient(90deg, #4cc9f0, #7b2fff, #ff4d6d); -webkit-background-clip: text; -webkit-text-fill-color: transparent; background-clip: text; }
        .live-badge { display: flex; align-items: center; gap: 5px; background: rgba(255,77,109,0.2); border: 1px solid rgba(255,77,109,0.5); border-radius: 20px; padding: 4px 10px; font-size: 11px; color: var(--accent1); font-weight: 700; }
        .live-dot { width: 7px; height: 7px; background: var(--accent1); border-radius: 50%; animation: pulse 1.2s infinite; }
        @keyframes pulse { 0%,100%{opacity:1;transform:scale(1)} 50%{opacity:0.5;transform:scale(1.3)} }

        /* DATE BAR */
        .date-bar { display: flex; justify-content: space-between; align-items: center; font-size: 11px; color: var(--muted); margin-bottom: 10px; }
        .weather-chip { background: rgba(76,201,240,0.15); border: 1px solid rgba(76,201,240,0.3); border-radius: 20px; padding: 3px 10px; font-size: 11px; color: var(--accent2); }

        /* CATEGORY TABS */
        .cat-scroll { display: flex; gap: 8px; overflow-x: auto; padding-bottom: 4px; scrollbar-width: none; }
        .cat-scroll::-webkit-scrollbar { display: none; }
        .cat-btn {
          flex-shrink: 0; padding: 7px 14px; border-radius: 20px; border: 1px solid var(--border);
          background: transparent; color: var(--muted); font-size: 12px; font-weight: 600;
          cursor: pointer; transition: all 0.2s; white-space: nowrap;
        }
        .cat-btn.active { color: #fff; border-color: transparent; }
        .cat-btn[data-cat="world"].active { background: linear-gradient(135deg, #4361ee, #3a86ff); }
        .cat-btn[data-cat="us"].active { background: linear-gradient(135deg, #ef476f, #ff4d6d); }
        .cat-btn[data-cat="politics"].active { background: linear-gradient(135deg, #7b2fff, #f72585); }
        .cat-btn[data-cat="business"].active { background: linear-gradient(135deg, #fb5607, #ffd60a); }
        .cat-btn[data-cat="tech"].active { background: linear-gradient(135deg, #4cc9f0, #06d6a0); }
        .cat-btn[data-cat="science"].active { background: linear-gradient(135deg, #06d6a0, #4cc9f0); }
        .cat-btn[data-cat="health"].active { background: linear-gradient(135deg, #ef476f, #fb5607); }
        .cat-btn[data-cat="sports"].active { background: linear-gradient(135deg, #ffd60a, #fb5607); }
        .cat-btn[data-cat="entertainment"].active { background: linear-gradient(135deg, #f72585, #7b2fff); }
        .cat-btn[data-cat="climate"].active { background: linear-gradient(135deg, #06d6a0, #4361ee); }

        /* MAIN CONTENT */
        .news-content { flex: 1; overflow-y: auto; padding: 12px 12px 90px; }

        /* BREAKING BANNER */
        .breaking-banner {
          background: linear-gradient(135deg, rgba(255,77,109,0.15), rgba(123,47,255,0.15));
          border: 1px solid rgba(255,77,109,0.4);
          border-radius: 12px; padding: 10px 14px; margin-bottom: 14px;
          display: flex; align-items: center; gap: 10px;
        }
        .breaking-label { background: var(--accent1); color: #fff; font-size: 9px; font-weight: 900; padding: 3px 7px; border-radius: 4px; letter-spacing: 1px; flex-shrink: 0; }
        .breaking-text { font-size: 12px; font-weight: 600; color: var(--text); overflow: hidden; white-space: nowrap; text-overflow: ellipsis; }

        /* HERO CARD */
        .hero-card {
          border-radius: 16px; overflow: hidden; margin-bottom: 14px;
          background: var(--card); border: 1px solid var(--border);
          text-decoration: none; display: block;
        }
        .hero-img { width: 100%; height: 200px; object-fit: cover; display: block; }
        .hero-img-placeholder { width: 100%; height: 200px; display: flex; align-items: center; justify-content: center; font-size: 60px; }
        .hero-body { padding: 14px; }
        .hero-meta { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
        .source-tag { font-size: 10px; font-weight: 700; padding: 3px 8px; border-radius: 4px; }
        .time-tag { font-size: 10px; color: var(--muted); }
        .hero-title { font-size: 17px; font-weight: 800; line-height: 1.3; color: var(--text); margin-bottom: 8px; }
        .hero-desc { font-size: 13px; color: var(--muted); line-height: 1.5; }

        /* SECTION HEADER */
        .section-header { display: flex; align-items: center; gap: 8px; margin: 18px 0 10px; }
        .section-line { flex: 1; height: 1px; background: var(--border); }
        .section-title { font-size: 11px; font-weight: 800; letter-spacing: 1.5px; text-transform: uppercase; color: var(--muted); }

        /* NEWS CARD */
        .news-card {
          background: var(--card); border: 1px solid var(--border); border-radius: 14px;
          margin-bottom: 10px; overflow: hidden; text-decoration: none; display: flex;
          gap: 0; transition: transform 0.15s;
        }
        .news-card:active { transform: scale(0.98); }
        .card-thumb { width: 90px; flex-shrink: 0; }
        .card-thumb img { width: 90px; height: 90px; object-fit: cover; display: block; }
        .card-thumb-placeholder { width: 90px; height: 90px; display: flex; align-items: center; justify-content: center; font-size: 32px; }
        .card-body { flex: 1; padding: 10px 12px; display: flex; flex-direction: column; justify-content: space-between; }
        .card-title { font-size: 13px; font-weight: 700; line-height: 1.4; color: var(--text); display: -webkit-box; -webkit-line-clamp: 3; -webkit-box-orient: vertical; overflow: hidden; }
        .card-footer { display: flex; align-items: center; gap: 6px; margin-top: 6px; }
        .card-source { font-size: 10px; font-weight: 700; }
        .card-time { font-size: 10px; color: var(--muted); }

        /* WIDE CARD */
        .wide-card {
          background: var(--card); border: 1px solid var(--border); border-radius: 14px;
          padding: 14px; margin-bottom: 10px; text-decoration: none; display: block;
          transition: transform 0.15s;
        }
        .wide-card:active { transform: scale(0.98); }
        .wide-title { font-size: 14px; font-weight: 700; line-height: 1.4; color: var(--text); margin-bottom: 6px; }
        .wide-desc { font-size: 12px; color: var(--muted); line-height: 1.5; display: -webkit-box; -webkit-line-clamp: 2; -webkit-box-orient: vertical; overflow: hidden; }
        .wide-meta { display: flex; align-items: center; gap: 8px; margin-top: 8px; }

        /* STATS ROW */
        .stats-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 14px; }
        .stat-card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 14px; }
        .stat-icon { font-size: 24px; margin-bottom: 6px; }
        .stat-label { font-size: 10px; color: var(--muted); font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 2px; }
        .stat-value { font-size: 20px; font-weight: 900; }
        .stat-sub { font-size: 10px; color: var(--muted); margin-top: 2px; }

        /* TICKER */
        .ticker-wrap { background: rgba(76,201,240,0.08); border: 1px solid rgba(76,201,240,0.2); border-radius: 10px; padding: 8px 12px; margin-bottom: 14px; overflow: hidden; }
        .ticker-label { font-size: 9px; font-weight: 800; color: var(--accent2); letter-spacing: 1px; text-transform: uppercase; margin-bottom: 4px; }
        .ticker-text { font-size: 12px; color: var(--text); white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }

        /* LOADING */
        .loading-state { display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 60px 20px; gap: 16px; }
        .spinner { width: 44px; height: 44px; border: 3px solid var(--border); border-top-color: var(--accent2); border-radius: 50%; animation: spin 0.8s linear infinite; }
        @keyframes spin { to { transform: rotate(360deg); } }
        .loading-text { color: var(--muted); font-size: 14px; }

        /* ERROR */
        .error-state { text-align: center; padding: 40px 20px; }
        .error-icon { font-size: 48px; margin-bottom: 12px; }
        .error-text { color: var(--muted); font-size: 14px; }

        /* BOTTOM NAV */
        .bottom-nav {
          position: fixed; bottom: 0; left: 50%; transform: translateX(-50%);
          width: 100%; max-width: 430px;
          background: rgba(10,10,26,0.95); backdrop-filter: blur(20px);
          border-top: 1px solid var(--border); display: flex;
          padding: 8px 0 24px;
        }
        .nav-item { flex: 1; display: flex; flex-direction: column; align-items: center; gap: 3px; cursor: pointer; padding: 6px 0; }
        .nav-icon { font-size: 22px; }
        .nav-label { font-size: 9px; font-weight: 600; color: var(--muted); }
        .nav-item.active .nav-label { color: var(--accent2); }
        .nav-item.active .nav-icon { filter: drop-shadow(0 0 6px var(--accent2)); }

        /* MARKET TICKER */
        .market-row { display: flex; gap: 8px; overflow-x: auto; padding-bottom: 4px; margin-bottom: 14px; scrollbar-width: none; }
        .market-row::-webkit-scrollbar { display: none; }
        .market-chip { flex-shrink: 0; background: var(--card); border: 1px solid var(--border); border-radius: 10px; padding: 8px 12px; text-align: center; min-width: 80px; }
        .market-name { font-size: 10px; color: var(--muted); font-weight: 600; }
        .market-val { font-size: 14px; font-weight: 800; color: var(--text); }
        .market-change { font-size: 10px; font-weight: 700; }
        .up { color: #06d6a0; }
        .down { color: #ef476f; }

        /* CATEGORY COLOR ACCENT */
        [data-color="world"] { --cat-color: #3a86ff; }
        [data-color="us"] { --cat-color: #ef476f; }
        [data-color="politics"] { --cat-color: #7b2fff; }
        [data-color="business"] { --cat-color: #fb5607; }
        [data-color="tech"] { --cat-color: #4cc9f0; }
        [data-color="science"] { --cat-color: #06d6a0; }
        [data-color="health"] { --cat-color: #ef476f; }
        [data-color="sports"] { --cat-color: #ffd60a; }
        [data-color="entertainment"] { --cat-color: #f72585; }
        [data-color="climate"] { --cat-color: #06d6a0; }
        .source-tag { background: color-mix(in srgb, var(--cat-color) 20%, transparent); color: var(--cat-color); }
        .card-source { color: var(--cat-color); }
        .hero-card { border-top: 3px solid var(--cat-color); }
      `}</style>
      <script dangerouslySetInnerHTML={{ __html: NEWS_SCRIPT }} />
    </div>
  );
}

const NEWS_SCRIPT = `
(function() {
  const CATEGORIES = [
    { id: "world", label: "🌍 World", emoji: "🌍" },
    { id: "us", label: "🇺🇸 US", emoji: "🇺🇸" },
    { id: "politics", label: "🏛️ Politics", emoji: "🏛️" },
    { id: "business", label: "📈 Business", emoji: "📈" },
    { id: "tech", label: "💻 Tech", emoji: "💻" },
    { id: "science", label: "🔬 Science", emoji: "🔬" },
    { id: "health", label: "🏥 Health", emoji: "🏥" },
    { id: "sports", label: "⚽ Sports", emoji: "⚽" },
    { id: "entertainment", label: "🎬 Entmt", emoji: "🎬" },
    { id: "climate", label: "🌱 Climate", emoji: "🌱" },
  ];

  const CAT_EMOJIS = {
    world:"🌍",us:"🇺🇸",politics:"🏛️",business:"📈",tech:"💻",
    science:"🔬",health:"🏥",sports:"⚽",entertainment:"🎬",climate:"🌱"
  };

  const CAT_PLACEHOLDERS = {
    world:"🌍",us:"🗽",politics:"🏛️",business:"💹",tech:"⚡",
    science:"🔭",health:"💊",sports:"🏆",entertainment:"🎭",climate:"🌿"
  };

  const MARKETS = [
    { name: "S&P 500", val: "5,847", chg: "+0.42%", up: true },
    { name: "NASDAQ", val: "19,231", chg: "+0.61%", up: true },
    { name: "DOW", val: "42,654", chg: "-0.18%", up: false },
    { name: "BTC", val: "64,210", chg: "+2.3%", up: true },
    { name: "EUR/USD", val: "1.0812", chg: "-0.09%", up: false },
    { name: "GOLD", val: "2,391", chg: "+0.55%", up: true },
    { name: "OIL", val: "78.43", chg: "-1.1%", up: false },
  ];

  let currentCat = "world";
  let cache = {};

  function timeAgo(iso) {
    const diff = Math.floor((Date.now() - new Date(iso)) / 1000);
    if (diff < 60) return diff + "s ago";
    if (diff < 3600) return Math.floor(diff/60) + "m ago";
    if (diff < 86400) return Math.floor(diff/3600) + "h ago";
    return Math.floor(diff/86400) + "d ago";
  }

  function renderMarkets() {
    return '<div class="market-row">' +
      MARKETS.map(m =>
        '<div class="market-chip"><div class="market-name">' + m.name + '</div>' +
        '<div class="market-val">' + m.val + '</div>' +
        '<div class="market-change ' + (m.up ? 'up' : 'down') + '">' + m.chg + '</div></div>'
      ).join("") + '</div>';
  }

  function renderBreaking(items) {
    if (!items.length) return "";
    return '<div class="breaking-banner">' +
      '<span class="breaking-label">BREAKING</span>' +
      '<span class="breaking-text">' + items[0].title + '</span></div>';
  }

  function renderHeroCard(item, cat) {
    const ph = CAT_PLACEHOLDERS[cat] || "📰";
    const imgHtml = item.img
      ? '<img class="hero-img" src="' + item.img + '" alt="" onerror="this.parentNode.innerHTML=\'<div class=hero-img-placeholder>' + ph + '</div>\'" />'
      : '<div class="hero-img-placeholder" style="background:linear-gradient(135deg,rgba(76,201,240,0.1),rgba(123,47,255,0.1))">' + ph + '</div>';
    return '<a class="hero-card" href="' + item.link + '" target="_blank" rel="noopener">' +
      imgHtml +
      '<div class="hero-body">' +
      '<div class="hero-meta"><span class="source-tag">' + item.source + '</span><span class="time-tag">' + timeAgo(item.pubDate) + '</span></div>' +
      '<div class="hero-title">' + item.title + '</div>' +
      (item.desc ? '<div class="hero-desc">' + item.desc.slice(0,160) + '…</div>' : '') +
      '</div></a>';
  }

  function renderSmallCard(item, cat) {
    const ph = CAT_PLACEHOLDERS[cat] || "📰";
    const imgHtml = item.img
      ? '<div class="card-thumb"><img src="' + item.img + '" alt="" onerror="this.style.display=\'none\'" /></div>'
      : '<div class="card-thumb"><div class="card-thumb-placeholder" style="background:rgba(255,255,255,0.03)">' + ph + '</div></div>';
    return '<a class="news-card" href="' + item.link + '" target="_blank" rel="noopener">' +
      imgHtml +
      '<div class="card-body"><div class="card-title">' + item.title + '</div>' +
      '<div class="card-footer"><span class="card-source">' + item.source + '</span><span class="card-time">' + timeAgo(item.pubDate) + '</span></div>' +
      '</div></a>';
  }

  function renderWideCard(item, cat) {
    return '<a class="wide-card" href="' + item.link + '" target="_blank" rel="noopener">' +
      '<div class="wide-title">' + item.title + '</div>' +
      (item.desc ? '<div class="wide-desc">' + item.desc + '</div>' : '') +
      '<div class="wide-meta"><span class="source-tag">' + item.source + '</span><span class="card-time">' + timeAgo(item.pubDate) + '</span></div>' +
      '</a>';
  }

  function renderStats(items) {
    const sources = [...new Set(items.map(i => i.source))];
    const oldest = items.reduce((a,b) => new Date(a.pubDate) > new Date(b.pubDate) ? b : a, items[0]);
    return '<div class="stats-row">' +
      '<div class="stat-card"><div class="stat-icon">📡</div><div class="stat-label">Stories</div><div class="stat-value" style="color:var(--accent2)">' + items.length + '</div><div class="stat-sub">live right now</div></div>' +
      '<div class="stat-card"><div class="stat-icon">🗞️</div><div class="stat-label">Sources</div><div class="stat-value" style="color:var(--accent4)">' + sources.length + '</div><div class="stat-sub">' + sources.join(", ") + '</div></div>' +
      '</div>';
  }

  function renderContent(items, cat) {
    if (!items || !items.length) {
      return '<div class="error-state"><div class="error-icon">📭</div><div class="error-text">No stories found.<br>Try another category.</div></div>';
    }
    const hero = items[0];
    const small = items.slice(1, 5);
    const wide = items.slice(5);

    let html = '';
    html += renderBreaking(items);
    html += renderMarkets();
    html += renderStats(items, cat);

    html += renderHeroCard(hero, cat);

    if (small.length) {
      html += '<div class="section-header"><div class="section-line"></div><div class="section-title">Latest Stories</div><div class="section-line"></div></div>';
      html += small.map(i => renderSmallCard(i, cat)).join("");
    }

    if (wide.length) {
      html += '<div class="section-header"><div class="section-line"></div><div class="section-title">More News</div><div class="section-line"></div></div>';
      html += wide.map(i => renderWideCard(i, cat)).join("");
    }

    return html;
  }

  function renderHeader() {
    const now = new Date();
    const dateStr = now.toLocaleDateString("en-US", { weekday:"long", month:"long", day:"numeric", year:"numeric" });
    const timeStr = now.toLocaleTimeString("en-US", { hour:"2-digit", minute:"2-digit" });

    return '<div class="news-header">' +
      '<div class="header-top">' +
        '<div class="logo-area">' +
          '<div class="logo-icon">🌐</div>' +
          '<span class="logo-text">GLOBAL NEWS HUB</span>' +
        '</div>' +
        '<div class="live-badge"><div class="live-dot"></div>LIVE</div>' +
      '</div>' +
      '<div class="date-bar"><span>' + dateStr + '</span><span class="weather-chip">🕐 ' + timeStr + '</span></div>' +
      '<div class="cat-scroll">' +
        CATEGORIES.map(c =>
          '<button class="cat-btn' + (c.id === currentCat ? ' active' : '') + '" data-cat="' + c.id + '" onclick="switchCat(\'' + c.id + '\')">' + c.label + '</button>'
        ).join("") +
      '</div></div>';
  }

  function renderBottomNav() {
    return '<div class="bottom-nav">' +
      '<div class="nav-item active"><div class="nav-icon">📰</div><div class="nav-label" style="color:var(--accent2)">News</div></div>' +
      '<div class="nav-item"><div class="nav-icon">🔥</div><div class="nav-label">Trending</div></div>' +
      '<div class="nav-item"><div class="nav-icon">🌎</div><div class="nav-label">Map</div></div>' +
      '<div class="nav-item"><div class="nav-icon">🔖</div><div class="nav-label">Saved</div></div>' +
      '<div class="nav-item"><div class="nav-icon">⚙️</div><div class="nav-label">Settings</div></div>' +
    '</div>';
  }

  function render(items, loading) {
    const root = document.getElementById("news-root");
    root.setAttribute("data-color", currentCat);

    let contentHtml;
    if (loading) {
      contentHtml = '<div class="loading-state"><div class="spinner"></div><div class="loading-text">Fetching latest ' + (CAT_EMOJIS[currentCat]||"") + ' news…</div></div>';
    } else {
      contentHtml = renderContent(items, currentCat);
    }

    root.innerHTML =
      renderHeader() +
      '<div class="news-content" id="news-scroll">' + contentHtml + '</div>' +
      renderBottomNav();
  }

  window.switchCat = function(cat) {
    if (cat === currentCat) return;
    currentCat = cat;
    if (cache[cat]) {
      render(cache[cat], false);
    } else {
      render([], true);
      loadNews(cat);
    }
  };

  async function loadNews(cat) {
    try {
      const res = await fetch("/api/news?category=" + cat);
      const data = await res.json();
      cache[cat] = data.items;
      if (currentCat === cat) render(data.items, false);
    } catch(e) {
      if (currentCat === cat) render([], false);
    }
  }

  document.addEventListener("DOMContentLoaded", function() {
    render([], true);
    loadNews("world");
    // Preload all categories in background
    setTimeout(() => {
      ["us","politics","business","tech"].forEach((c,i) => setTimeout(() => loadNews(c), i*2000));
    }, 3000);
    setTimeout(() => {
      ["science","health","sports","entertainment","climate"].forEach((c,i) => setTimeout(() => loadNews(c), i*2000));
    }, 12000);
    // Refresh every 10 minutes
    setInterval(() => { cache = {}; loadNews(currentCat); }, 600000);
  });
})();
`;
