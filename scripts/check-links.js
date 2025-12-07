#!/usr/bin/env node
/**
 * Check for broken links in the built site
 * Usage: node scripts/check-links.js
 */

const { SiteChecker } = require('broken-link-checker');

const baseUrl = process.env.SITE_URL || 'http://localhost:3000';

console.log(`🔍 Checking links for: ${baseUrl}`);
console.log('Starting link validation...\n');

let pageCount = 0;
let linkCount = 0;
let brokenCount = 0;
const brokenLinks = [];

const siteChecker = new SiteChecker(
  {
    excludeExternalLinks: true, // Only check internal links
    excludeInternalLinks: false,
    filterLevel: 3, // Check everything
    honorRobotExclusions: false,
    maxSocketsPerHost: 10,
  },
  {
    link: (result) => {
      linkCount++;
      if (result.broken) {
        brokenCount++;
        brokenLinks.push({
          page: result.base.resolved,
          link: result.url.resolved,
          reason: result.brokenReason,
        });
        console.log(`❌ Broken: ${result.url.resolved}`);
        console.log(`   From: ${result.base.resolved}`);
        console.log(`   Reason: ${result.brokenReason}\n`);
      }
    },
    page: (error, pageUrl) => {
      pageCount++;
      if (error) {
        console.log(`⚠️  Error checking page: ${pageUrl}`);
      } else {
        console.log(`✅ Checked: ${pageUrl}`);
      }
    },
    end: () => {
      console.log('\n=== Link Check Results ===');
      console.log(`Pages checked: ${pageCount}`);
      console.log(`Links checked: ${linkCount}`);
      console.log(`Broken links: ${brokenCount}`);

      if (brokenCount > 0) {
        console.log('\n❌ Link validation failed');
        console.log('\nBroken links found:');
        brokenLinks.forEach((broken) => {
          console.log(`- ${broken.link}`);
          console.log(`  From: ${broken.page}`);
          console.log(`  Reason: ${broken.reason}\n`);
        });
        process.exit(1);
      }

      console.log('\n✅ All links are valid');
      process.exit(0);
    },
  }
);

siteChecker.enqueue(baseUrl);
