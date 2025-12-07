#!/usr/bin/env node
/**
 * Validate MDX frontmatter against JSON Schema
 * Usage: node scripts/validate-metadata.js
 */

const fs = require('fs');
const path = require('path');
const Ajv = require('ajv');
const glob = require('glob');

const ajv = new Ajv({ allErrors: true });

// Load schema
const schemaPath = path.join(__dirname, '../src/schemas/chapter-metadata-schema.json');
let schema;

try {
  schema = JSON.parse(fs.readFileSync(schemaPath, 'utf8'));
} catch (error) {
  console.log('⚠️  Schema not found yet - skipping validation');
  console.log('   (Schemas will be created in T023)');
  process.exit(0);
}

const validate = ajv.compile(schema);

// Find all MDX files
const mdxFiles = glob.sync('docs/**/*.mdx', { cwd: process.cwd() });

let totalFiles = 0;
let validFiles = 0;
let errors = [];

mdxFiles.forEach((file) => {
  const content = fs.readFileSync(file, 'utf8');

  // Extract frontmatter
  const frontmatterMatch = content.match(/^---\n([\s\S]*?)\n---/);

  if (!frontmatterMatch) {
    console.log(`⚠️  ${file}: No frontmatter found`);
    return;
  }

  totalFiles++;

  try {
    // Parse YAML frontmatter (simplified - in production use a YAML parser)
    const frontmatter = {};
    const lines = frontmatterMatch[1].split('\n');

    lines.forEach((line) => {
      const match = line.match(/^(\w+):\s*(.+)$/);
      if (match) {
        const [, key, value] = match;
        // Simple parsing - handle strings and arrays
        if (value.startsWith('[')) {
          frontmatter[key] = JSON.parse(value.replace(/'/g, '"'));
        } else {
          frontmatter[key] = value.replace(/^["']|["']$/g, '');
        }
      }
    });

    const valid = validate(frontmatter);

    if (valid) {
      validFiles++;
      console.log(`✅ ${file}`);
    } else {
      errors.push({
        file,
        errors: validate.errors,
      });
      console.log(`❌ ${file}`);
      validate.errors.forEach((err) => {
        console.log(`   ${err.instancePath} ${err.message}`);
      });
    }
  } catch (parseError) {
    console.log(`❌ ${file}: Parse error - ${parseError.message}`);
  }
});

console.log('\n=== Metadata Validation Results ===');
console.log(`Total files: ${totalFiles}`);
console.log(`Valid: ${validFiles}`);
console.log(`Invalid: ${totalFiles - validFiles}`);

if (errors.length > 0) {
  console.log('\n❌ Validation failed');
  process.exit(1);
}

console.log('\n✅ All metadata valid');
process.exit(0);
