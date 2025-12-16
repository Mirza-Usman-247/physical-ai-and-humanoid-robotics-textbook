/**
 * SourceList Component
 * Displays list of citation sources with metadata.
 * Implements FR-011: Display source references.
 */

import React from 'react';
import styles from './SourceList.module.css';

interface CitationSource {
  id: number;
  file_path: string;
  line_range: string;
  section: string;
  score: number;
}

interface SourceListProps {
  sources: CitationSource[];
  theme?: 'light' | 'dark';
}

/**
 * List of sources with relevance scores.
 *
 * Features:
 * - Numbered sources matching citations
 * - File path and line range
 * - Section breadcrumbs
 * - Relevance score visualization
 */
const SourceList: React.FC<SourceListProps> = ({ sources, theme = 'light' }) => {
  return (
    <div className={`${styles.sourceList} ${styles[theme]}`}>
      <h4 className={styles.title}>Sources</h4>
      {sources.map((source) => (
        <div
          key={source.id}
          id={`source-${source.id}`}
          className={styles.source}
        >
          <div className={styles.sourceHeader}>
            <span className={styles.sourceNumber}>[{source.id}]</span>
            <span className={styles.relevanceScore}>
              {Math.round(source.score * 100)}% relevant
            </span>
          </div>
          <div className={styles.sourceSection}>{source.section}</div>
          <code className={styles.sourcePath}>
            {source.file_path}:{source.line_range}
          </code>
        </div>
      ))}
    </div>
  );
};

export default SourceList;
