/**
 * ErrorMessage Component
 * User-friendly error message display.
 * Implements FR-031: Friendly error messages.
 */

import React from 'react';
import styles from './ErrorMessage.module.css';

interface ErrorMessageProps {
  message: string;
  onDismiss?: () => void;
  onRetry?: () => void;
}

/**
 * Error message with dismiss and retry options.
 *
 * Features:
 * - User-friendly error display
 * - Optional dismiss button
 * - Optional retry button
 * - Icon indication
 */
const ErrorMessage: React.FC<ErrorMessageProps> = ({
  message,
  onDismiss,
  onRetry
}) => {
  return (
    <div className={styles.errorMessage} role="alert">
      <div className={styles.icon}>
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <circle cx="12" cy="12" r="10" />
          <line x1="12" y1="8" x2="12" y2="12" />
          <line x1="12" y1="16" x2="12.01" y2="16" />
        </svg>
      </div>
      <div className={styles.content}>
        <p className={styles.text}>{message}</p>
        <div className={styles.actions}>
          {onRetry && (
            <button className={styles.retryButton} onClick={onRetry}>
              Try Again
            </button>
          )}
          {onDismiss && (
            <button className={styles.dismissButton} onClick={onDismiss}>
              Dismiss
            </button>
          )}
        </div>
      </div>
    </div>
  );
};

export default ErrorMessage;
