import { NextResponse } from "next/server";

const FEEDS = {
  world: [
    "https://feeds.bbci.co.uk/news/world/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/World.xml",
  ],
  us: [
    "https://feeds.bbci.co.uk/news/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/US.xml",
  ],
  tech: [
    "https://feeds.bbci.co.uk/news/technology/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Technology.xml",
  ],
  science: [
    "https://feeds.bbci.co.uk/news/science_and_environment/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Science.xml",
  ],
  business: [
    "https://feeds.bbci.co.uk/news/business/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Business.xml",
  ],
  health: [
    "https://feeds.bbci.co.uk/news/health/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Health.xml",
  ],
  sports: [
    "https://feeds.bbci.co.uk/sport/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Sports.xml",
  ],
  entertainment: [
    "https://feeds.bbci.co.uk/news/entertainment_and_arts/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Arts.xml",
  ],
  climate: [
    "https://feeds.bbci.co.uk/news/science_and_environment/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Climate.xml",
  ],
  politics: [
    "https://feeds.bbci.co.uk/news/politics/rss.xml",
    "https://rss.nytimes.com/services/xml/rss/nyt/Politics.xml",
  ],
};

function parseItems(xml, source, category) {
  const items = [];
  const itemMatches = xml.match(/<item>([\s\S]*?)<\/item>/g) || [];

  for (const item of itemMatches.slice(0, 6)) {
    const title = (item.match(/<title><!\[CDATA\[(.*?)\]\]><\/title>/) ||
      item.match(/<title>(.*?)<\/title>/))?.[1]?.trim();
    const link = (item.match(/<link>(.*?)<\/link>/) ||
      item.match(/<guid[^>]*>(https?:\/\/[^<]+)<\/guid>/))?.[1]?.trim();
    const desc = (item.match(/<description><!\[CDATA\[(.*?)\]\]><\/description>/) ||
      item.match(/<description>(.*?)<\/description>/))?.[1]
      ?.replace(/<[^>]+>/g, "")
      ?.trim();
    const pubDate = (item.match(/<pubDate>(.*?)<\/pubDate>/))?.[1]?.trim();
    const img =
      item.match(/url="(https?:\/\/[^"]+\.(?:jpg|jpeg|png|webp)[^"]*)"/)?.[1] ||
      item.match(/<media:thumbnail[^>]+url="([^"]+)"/)?.[1] ||
      item.match(/<media:content[^>]+url="([^"]+\.(?:jpg|jpeg|png|webp)[^"]*)"/)?.[1];

    if (title && link) {
      items.push({
        title: title.replace(/&amp;/g, "&").replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&quot;/g, '"').replace(/&#39;/g, "'"),
        link,
        desc: desc ? desc.replace(/&amp;/g, "&").replace(/&lt;/g, "<").replace(/&gt;/g, ">").slice(0, 200) : "",
        pubDate: pubDate ? new Date(pubDate).toISOString() : new Date().toISOString(),
        img: img || null,
        source,
        category,
      });
    }
  }
  return items;
}

async function fetchFeed(url, category) {
  try {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 5000);
    const res = await fetch(url, {
      signal: controller.signal,
      headers: { "User-Agent": "WorldNewsCenter/1.0" },
      next: { revalidate: 300 },
    });
    clearTimeout(timeout);
    if (!res.ok) return [];
    const xml = await res.text();
    const source = url.includes("bbc") ? "BBC News" : url.includes("nytimes") ? "NY Times" : "Reuters";
    return parseItems(xml, source, category);
  } catch {
    return [];
  }
}

export async function GET(request) {
  const { searchParams } = new URL(request.url);
  const category = searchParams.get("category") || "world";
  const feeds = FEEDS[category] || FEEDS.world;

  const results = await Promise.all(feeds.map((url) => fetchFeed(url, category)));
  const items = results.flat().sort((a, b) => new Date(b.pubDate) - new Date(a.pubDate));

  return NextResponse.json({ items, category, fetched: new Date().toISOString() });
}
