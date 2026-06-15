#!/usr/bin/env python3
"""
Cold Outreach Personaliser
Usage: python outreach.py https://example.com
"""

import sys
import re
import textwrap
from urllib.parse import urljoin, urlparse

import anthropic
import requests
from bs4 import BeautifulSoup


def scrape_website(url: str) -> str:
    """Fetch and extract readable text from a website."""
    headers = {
        "User-Agent": (
            "Mozilla/5.0 (Windows NT 10.0; Win64; x64) "
            "AppleWebKit/537.36 (KHTML, like Gecko) "
            "Chrome/120.0.0.0 Safari/537.36"
        )
    }

    try:
        response = requests.get(url, headers=headers, timeout=15)
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        print(f"Error fetching {url}: {e}", file=sys.stderr)
        sys.exit(1)

    soup = BeautifulSoup(response.text, "html.parser")

    # Remove noise
    for tag in soup(["script", "style", "nav", "footer", "iframe", "noscript"]):
        tag.decompose()

    # Pull key elements first
    sections = []

    title = soup.find("title")
    if title:
        sections.append(f"Page title: {title.get_text(strip=True)}")

    for tag in ["h1", "h2", "h3"]:
        for el in soup.find_all(tag)[:8]:
            text = el.get_text(strip=True)
            if text:
                sections.append(text)

    # Meta description
    meta_desc = soup.find("meta", attrs={"name": "description"})
    if meta_desc and meta_desc.get("content"):
        sections.append(f"Site description: {meta_desc['content']}")

    # Body paragraphs
    for p in soup.find_all("p")[:20]:
        text = p.get_text(strip=True)
        if len(text) > 40:
            sections.append(text)

    content = "\n\n".join(sections)

    # Trim to ~4000 chars so we don't blow the prompt
    if len(content) > 4000:
        content = content[:4000] + "\n...[content truncated]"

    return content


def generate_email(url: str, website_content: str) -> str:
    """Use Claude to generate a personalised cold email."""
    client = anthropic.Anthropic()

    domain = urlparse(url).netloc.replace("www.", "")

    prompt = f"""You are helping Khalid, founder of an AI automation agency, write a cold outreach email to a prospect.

Here is the content scraped from the prospect's website ({url}):

---
{website_content}
---

Based on this website content, write a personalised cold email from Khalid. The email must:

1. Reference something SPECIFIC from their website (a service, product, pain point, or unique aspect — not generic praise)
2. Be concise — MAXIMUM 150 words in the email body
3. Include a clear CTA to book a 15-minute call
4. Sound human and natural, not like a template
5. Position Khalid as someone who can help them save time and increase revenue using AI automation
6. Have a compelling subject line

Format your response exactly like this:
Subject: [subject line here]

[email body here]

Keep the tone warm but professional. Do not use filler phrases like "I hope this finds you well." Get straight to the point."""

    response = client.messages.create(
        model="claude-opus-4-8",
        max_tokens=512,
        thinking={"type": "adaptive"},
        messages=[{"role": "user", "content": prompt}],
    )

    return response.content[-1].text


def main():
    if len(sys.argv) != 2:
        print("Usage: python outreach.py <URL>")
        print("Example: python outreach.py https://example.com")
        sys.exit(1)

    url = sys.argv[1]

    # Basic URL validation
    parsed = urlparse(url)
    if not parsed.scheme or not parsed.netloc:
        print(f"Invalid URL: {url}")
        print("Please include the protocol, e.g. https://example.com")
        sys.exit(1)

    print(f"Scraping {url}...")
    content = scrape_website(url)
    print(f"Scraped {len(content)} characters of content.\n")

    print("Generating personalised email with Claude...\n")
    email = generate_email(url, content)

    # Display
    divider = "─" * 60
    print(divider)
    print("GENERATED EMAIL")
    print(divider)
    print(email)
    print(divider)

    # Save to file
    output_file = "output_email.txt"
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(f"Generated for: {url}\n")
        f.write(divider + "\n")
        f.write(email + "\n")

    print(f"\n✓ Email saved to {output_file}")


if __name__ == "__main__":
    main()
