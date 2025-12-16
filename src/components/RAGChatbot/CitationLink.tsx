/**
 * CitationLink Component
 * Inline citation link [N] with tooltip.
 * Implements FR-010: Inline citations.
 */

import React, { useState } from 'react';
import styles from './CitationLink.module.css';

interface CitationSource {
  id: number;
  file_path: string;
  line_range: string;
  section: string;
  score: number;
}

interface CitationLinkProps {
  citationNumber: number;
  source: CitationSource;
}

/**
 * Clickable citation with hover tooltip.
 *
 * Features:
 * - [N] format display
 * - Hover tooltip with source info
 * - Click to scroll to source in list
 */
const CitationLink: React.FC<CitationLinkProps> = ({ citationNumber, source }) => {
  const [showTooltip, setShowTooltip] = useState(false);

  const handleClick = () => {
    // Scroll to source in source list
    const sourceElement = document.getElementById(`source-${citationNumber}`);
    if (sourceElement) {
      sourceElement.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    }
  };

  return (
    <span
      className={styles.citationLink}
      onMouseEnter={() => setShowTooltip(true)}
      onMouseLeave={() => setShowTooltip(false)}
      onClick={handleClick}
      role="button"
      tabIndex={0}
    >
      [{citationNumber}]
      {showTooltip && (
        <span className={styles.tooltip}>
          <strong>{source.section}</strong>
          <br />
          <code>{source.file_path}:{source.line_range}</code>
          <br />
          <span className={styles.relevance}>
            Relevance: {Math.round(source.score * 100)}%
          </span>
        </span>
      )}
    </span>
  );
};

export default CitationLink;
