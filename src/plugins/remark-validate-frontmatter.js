const path = require('path');
const fs = require('fs').promises;
const Ajv = require('ajv');
const addFormats = require('ajv-formats');

// Initialize AJV validator
const ajv = new Ajv();
addFormats(ajv);

// Schema cache to avoid reading files multiple times
const schemaCache = new Map();

/**
 * Load and compile a JSON schema
 */
async function loadSchema(schemaPath) {
  if (schemaCache.has(schemaPath)) {
    return schemaCache.get(schemaPath);
  }

  try {
    const schemaContent = await fs.readFile(schemaPath, 'utf-8');
    const schema = JSON.parse(schemaContent);
    const validate = ajv.compile(schema);
    schemaCache.set(schemaPath, validate);
    return validate;
  } catch (error) {
    console.error(`Error loading schema ${schemaPath}:`, error.message);
    throw error;
  }
}

/**
 * Remark plugin to validate frontmatter against JSON schemas
 */
async function validateFrontmatterPlugin() {
  // Determine the appropriate schema based on the file path
  const validate = async (tree, file) => {
    // Only validate MD and MDX files
    if (!file.path.endsWith('.md') && !file.path.endsWith('.mdx')) {
      return;
    }

    // Check if the file has frontmatter
    if (!file.data || !file.data.frontmatter) {
      console.warn(`File ${file.path} has no frontmatter to validate`);
      return;
    }

    const frontmatter = file.data.frontmatter;

    // Determine which schema to use based on the file location
    let schemaPath;

    if (file.path.includes('/glossary.md')) {
      schemaPath = path.resolve(__dirname, '../schemas/glossary-entry-schema.json');
    } else if (file.path.includes('/module-') && file.path.includes('/_category_.json')) {
      schemaPath = path.resolve(__dirname, '../schemas/module-metadata-schema.json');
    } else {
      // Default to chapter metadata schema for other files
      schemaPath = path.resolve(__dirname, '../schemas/chapter-metadata-schema.json');
    }

    try {
      // Check if schema file exists
      await fs.access(schemaPath);

      // Load and validate against the schema
      const validateSchema = await loadSchema(schemaPath);
      const valid = validateSchema(frontmatter);

      if (!valid) {
        console.error(`\n❌ Frontmatter validation failed for: ${file.path}`);
        console.error(`Schema: ${schemaPath}`);
        console.error('Validation errors:', validateSchema.errors);

        // Throw an error to fail the build
        throw new Error(`Frontmatter validation failed for ${file.path}: ${ajv.errorsText(validateSchema.errors)}`);
      } else {
        console.log(`✅ Frontmatter validation passed for: ${file.path}`);
      }
    } catch (error) {
      if (error.code === 'ENOENT') {
        console.warn(`Schema file not found: ${schemaPath}, skipping validation for ${file.path}`);
      } else {
        console.error(`Error during frontmatter validation for ${file.path}:`, error.message);
        throw error;
      }
    }
  };

  return validate;
}

// Export the plugin
module.exports = validateFrontmatterPlugin;