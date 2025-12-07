/**
 * FlexSearch Plugin for Docusaurus
 * Builds search index from glossary.md at build time
 */

const fs = require('fs');
const path = require('path');

function parseGlossaryMarkdown(filePath) {
  const content = fs.readFileSync(filePath, 'utf-8');
  const entries = [];

  // Parse markdown sections
  const sections = content.split(/^## /m).filter(Boolean);

  sections.forEach((section) => {
    const lines = section.split('\n');
    const category = lines[0].trim();

    let currentTerm = null;
    let currentDefinition = [];

    for (let i = 1; i < lines.length; i++) {
      const line = lines[i];

      if (line.startsWith('### ')) {
        // Save previous term if exists
        if (currentTerm) {
          entries.push({
            id: `${category.toLowerCase()}-${currentTerm.toLowerCase().replace(/\s+/g, '-')}`,
            term: currentTerm,
            definition: currentDefinition.join(' ').trim(),
            category: getCategoryName(category),
          });
        }

        // Start new term
        currentTerm = line.replace('### ', '').trim();
        currentDefinition = [];
      } else if (currentTerm && line.trim()) {
        currentDefinition.push(line.trim());
      }
    }

    // Save last term
    if (currentTerm) {
      entries.push({
        id: `${category.toLowerCase()}-${currentTerm.toLowerCase().replace(/\s+/g, '-')}`,
        term: currentTerm,
        definition: currentDefinition.join(' ').trim(),
        category: getCategoryName(category),
      });
    }
  });

  return entries;
}

function getCategoryName(letter) {
  const categoryMap = {
    'A': 'robotics',
    'B': 'robotics',
    'C': 'control-theory',
    'D': 'simulation',
    'E': 'ai',
    'F': 'simulation',
    'G': 'simulation',
    'H': 'hardware',
    'I': 'robotics',
    'J': 'robotics',
    'K': 'mathematics',
    'L': 'sensors',
    'M': 'ai',
    'N': 'software',
    'O': 'software',
    'P': 'simulation',
    'Q': 'software',
    'R': 'robotics',
    'S': 'simulation',
    'T': 'robotics',
    'U': 'simulation',
    'V': 'ai',
    'W': 'control-theory',
    'Z': 'ai',
  };

  return categoryMap[letter] || 'general';
}

module.exports = function (context, options) {
  return {
    name: 'flexsearch-plugin',

    async contentLoaded({ actions }) {
      const { setGlobalData } = actions;

      try {
        const glossaryPath = path.join(context.siteDir, 'docs', 'glossary.md');

        if (fs.existsSync(glossaryPath)) {
          const entries = parseGlossaryMarkdown(glossaryPath);

          // Save to global data for client-side access
          setGlobalData({
            glossaryEntries: entries,
            indexedAt: new Date().toISOString(),
            totalEntries: entries.length,
          });

          console.log(`✓ FlexSearch indexed ${entries.length} glossary terms`);
        } else {
          console.warn('⚠ Glossary file not found at:', glossaryPath);
        }
      } catch (error) {
        console.error('✗ FlexSearch plugin error:', error);
      }
    },
  };
};
