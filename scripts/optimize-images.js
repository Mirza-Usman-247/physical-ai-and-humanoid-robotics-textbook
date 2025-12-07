#!/usr/bin/env node
/**
 * Optimize images using sharp and validate ≤500KB constraint
 * Usage: node scripts/optimize-images.js
 */

const fs = require('fs');
const path = require('path');
const sharp = require('sharp');
const glob = require('glob');

const MAX_SIZE_KB = 500;
const MAX_SIZE_BYTES = MAX_SIZE_KB * 1024;

async function optimizeImage(imagePath) {
  try {
    const stats = fs.statSync(imagePath);
    const originalSize = stats.size;

    // Skip if already small enough
    if (originalSize <= MAX_SIZE_BYTES) {
      console.log(`✅ ${imagePath} (${(originalSize / 1024).toFixed(2)}KB) - Already optimized`);
      return { optimized: false, size: originalSize };
    }

    console.log(`🔧 Optimizing ${imagePath} (${(originalSize / 1024).toFixed(2)}KB)...`);

    const image = sharp(imagePath);
    const metadata = await image.metadata();

    // Apply lossless compression
    let optimized;
    if (metadata.format === 'png') {
      optimized = await image
        .png({ compressionLevel: 9, quality: 90 })
        .toBuffer();
    } else if (metadata.format === 'jpeg' || metadata.format === 'jpg') {
      optimized = await image
        .jpeg({ quality: 85, mozjpeg: true })
        .toBuffer();
    } else {
      console.log(`⚠️  ${imagePath} - Unsupported format: ${metadata.format}`);
      return { optimized: false, size: originalSize };
    }

    // Write optimized image
    fs.writeFileSync(imagePath, optimized);

    const newSize = optimized.length;
    const savings = originalSize - newSize;
    const savingsPercent = ((savings / originalSize) * 100).toFixed(1);

    if (newSize > MAX_SIZE_BYTES) {
      console.log(`❌ ${imagePath} - Still too large after optimization: ${(newSize / 1024).toFixed(2)}KB`);
      return { optimized: true, size: newSize, error: 'TOO_LARGE' };
    }

    console.log(`✅ ${imagePath} - Optimized: ${(originalSize / 1024).toFixed(2)}KB → ${(newSize / 1024).toFixed(2)}KB (saved ${savingsPercent}%)`);
    return { optimized: true, size: newSize };

  } catch (error) {
    console.log(`❌ ${imagePath} - Error: ${error.message}`);
    return { optimized: false, size: 0, error: error.message };
  }
}

async function main() {
  console.log('🖼️  Image Optimization & Validation\n');
  console.log(`Max size: ${MAX_SIZE_KB}KB\n`);

  // Find all images
  const imagePatterns = [
    'static/img/**/*.png',
    'static/img/**/*.jpg',
    'static/img/**/*.jpeg',
    'docs/**/*.png',
    'docs/**/*.jpg',
    'docs/**/*.jpeg',
  ];

  const images = [];
  imagePatterns.forEach((pattern) => {
    const found = glob.sync(pattern, { cwd: process.cwd() });
    images.push(...found.map((f) => path.join(process.cwd(), f)));
  });

  if (images.length === 0) {
    console.log('⚠️  No images found');
    process.exit(0);
  }

  console.log(`Found ${images.length} images\n`);

  let totalOptimized = 0;
  let totalErrors = 0;
  let tooLarge = [];

  for (const imagePath of images) {
    const result = await optimizeImage(imagePath);
    if (result.optimized) totalOptimized++;
    if (result.error) {
      totalErrors++;
      if (result.error === 'TOO_LARGE') {
        tooLarge.push({ path: imagePath, size: result.size });
      }
    }
  }

  console.log('\n=== Optimization Results ===');
  console.log(`Total images: ${images.length}`);
  console.log(`Optimized: ${totalOptimized}`);
  console.log(`Errors: ${totalErrors}`);

  if (tooLarge.length > 0) {
    console.log('\n❌ The following images exceed 500KB limit:');
    tooLarge.forEach(({ path: p, size }) => {
      console.log(`   ${p} (${(size / 1024).toFixed(2)}KB)`);
    });
    process.exit(1);
  }

  console.log('\n✅ All images meet size requirements');
  process.exit(0);
}

main().catch((error) => {
  console.error('Fatal error:', error);
  process.exit(1);
});
