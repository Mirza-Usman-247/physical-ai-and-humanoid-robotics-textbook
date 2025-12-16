/**
 * LoadingIndicator Component
 * Animated loading indicator for responses.
 * Implements FR-018: Loading indicator while waiting.
 */

import React from 'react';
import styles from './LoadingIndicator.module.css';

/**
 * Animated dots loading indicator.
 *
 * Features:
 * - Smooth pulse animation
 * - Three animated dots
 * - Accessible loading message
 */
const LoadingIndicator: React.FC = () => {
  return (
    <div className={styles.loadingIndicator} role="status" aria-label="Loading response">
      <div className={styles.dots}>
        <div className={styles.dot} />
        <div className={styles.dot} />
        <div className={styles.dot} />
      </div>
      <span className={styles.text}>Thinking...</span>
    </div>
  );
};

export default LoadingIndicator;
