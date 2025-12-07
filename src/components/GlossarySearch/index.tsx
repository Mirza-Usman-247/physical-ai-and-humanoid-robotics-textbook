import React, { useState, useEffect, useMemo } from 'react';
import FlexSearch from 'flexsearch';
import styles from './styles.module.css';

interface GlossaryEntry {
  id: string;
  term: string;
  definition: string;
  category?: string;
  relatedTerms?: string[];
}

interface GlossarySearchProps {
  entries: GlossaryEntry[];
}

const GlossarySearch: React.FC<GlossarySearchProps> = ({ entries }) => {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<GlossaryEntry[]>([]);
  const [selectedCategory, setSelectedCategory] = useState<string>('all');

  // Initialize FlexSearch index
  const index = useMemo(() => {
    const flexIndex = new FlexSearch.Document({
      tokenize: 'forward',
      cache: 100,
      document: {
        id: 'id',
        index: ['term', 'definition'],
        store: ['term', 'definition', 'category', 'relatedTerms'],
      },
      context: {
        resolution: 9,
        depth: 3,
        bidirectional: true,
      },
    });

    // Add all entries to index
    entries.forEach((entry) => {
      flexIndex.add(entry);
    });

    return flexIndex;
  }, [entries]);

  // Get unique categories
  const categories = useMemo(() => {
    const cats = new Set<string>();
    entries.forEach((entry) => {
      if (entry.category) cats.add(entry.category);
    });
    return ['all', ...Array.from(cats).sort()];
  }, [entries]);

  // Perform search
  useEffect(() => {
    if (!query || query.length < 2) {
      setResults([]);
      return;
    }

    const startTime = performance.now();

    // Search the index
    const searchResults = index.search(query, {
      limit: 20,
      enrich: true,
    });

    // Extract and deduplicate results
    const foundEntries = new Map<string, GlossaryEntry>();

    searchResults.forEach((fieldResults: any) => {
      fieldResults.result.forEach((item: any) => {
        if (!foundEntries.has(item.id)) {
          foundEntries.set(item.id, {
            id: item.id,
            term: item.doc.term,
            definition: item.doc.definition,
            category: item.doc.category,
            relatedTerms: item.doc.relatedTerms,
          });
        }
      });
    });

    // Filter by category if selected
    let filtered = Array.from(foundEntries.values());
    if (selectedCategory !== 'all') {
      filtered = filtered.filter((entry) => entry.category === selectedCategory);
    }

    setResults(filtered);

    const endTime = performance.now();
    console.log(`Search completed in ${(endTime - startTime).toFixed(2)}ms`);
  }, [query, selectedCategory, index]);

  const highlightMatch = (text: string, query: string): React.ReactNode => {
    if (!query) return text;

    const parts = text.split(new RegExp(`(${query})`, 'gi'));
    return parts.map((part, i) =>
      part.toLowerCase() === query.toLowerCase() ? (
        <mark key={i} className={styles.highlight}>
          {part}
        </mark>
      ) : (
        part
      )
    );
  };

  return (
    <div className={styles.glossarySearch}>
      <div className={styles.searchContainer}>
        <input
          type="text"
          placeholder="Search glossary... (e.g., 'inverse kinematics', 'ROS 2')"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className={styles.searchInput}
          autoFocus
        />

        <select
          value={selectedCategory}
          onChange={(e) => setSelectedCategory(e.target.value)}
          className={styles.categoryFilter}
        >
          {categories.map((cat) => (
            <option key={cat} value={cat}>
              {cat === 'all' ? 'All Categories' : cat}
            </option>
          ))}
        </select>
      </div>

      {query && query.length >= 2 && (
        <div className={styles.resultsContainer}>
          {results.length > 0 ? (
            <>
              <div className={styles.resultsHeader}>
                Found {results.length} result{results.length !== 1 ? 's' : ''}
              </div>
              <ul className={styles.resultsList}>
                {results.map((entry) => (
                  <li key={entry.id} className={styles.resultItem}>
                    <h3 className={styles.resultTerm}>
                      {highlightMatch(entry.term, query)}
                    </h3>
                    {entry.category && (
                      <span className={styles.resultCategory}>{entry.category}</span>
                    )}
                    <p className={styles.resultDefinition}>
                      {highlightMatch(entry.definition.substring(0, 200), query)}
                      {entry.definition.length > 200 ? '...' : ''}
                    </p>
                    {entry.relatedTerms && entry.relatedTerms.length > 0 && (
                      <div className={styles.relatedTerms}>
                        <strong>Related:</strong>{' '}
                        {entry.relatedTerms.join(', ')}
                      </div>
                    )}
                  </li>
                ))}
              </ul>
            </>
          ) : (
            <div className={styles.noResults}>
              No results found for "{query}". Try different keywords or check spelling.
            </div>
          )}
        </div>
      )}

      {!query && (
        <div className={styles.searchHelp}>
          <p>
            <strong>Search Tips:</strong>
          </p>
          <ul>
            <li>Type at least 2 characters to search</li>
            <li>Search supports fuzzy matching for typo tolerance</li>
            <li>Filter by category using the dropdown</li>
            <li>Search works on both terms and definitions</li>
          </ul>
        </div>
      )}
    </div>
  );
};

export default GlossarySearch;
