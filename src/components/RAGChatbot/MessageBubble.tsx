/**
 * MessageBubble Component
 * Individual message bubble with citations and feedback.
 * Implements FR-007, FR-010, FR-012.
 */

import React, { useState } from 'react';
import SourceList from './SourceList';
import CitationLink from './CitationLink';
import styles from './MessageBubble.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: CitationSource[];
  confidence?: number;
  timestamp: Date;
}

interface CitationSource {
  id: number;
  file_path: string;
  line_range: string;
  section: string;
  score: number;
}

interface MessageBubbleProps {
  message: Message;
  onFeedback?: (messageId: string, rating: 'positive' | 'negative') => void;
  theme?: 'light' | 'dark';
}

/**
 * Message bubble with citations and feedback buttons.
 *
 * Features:
 * - Styled user/assistant messages
 * - Inline citation links
 * - Source list display
 * - Feedback buttons (thumbs up/down)
 * - Confidence indicator
 */
const MessageBubble: React.FC<MessageBubbleProps> = ({
  message,
  onFeedback,
  theme = 'light'
}) => {
  const [feedbackGiven, setFeedbackGiven] = useState<'positive' | 'negative' | null>(null);
  const [showSources, setShowSources] = useState(false);

  const handleFeedback = (rating: 'positive' | 'negative') => {
    setFeedbackGiven(rating);
    if (onFeedback) {
      onFeedback(message.id, rating);
    }
  };

  // Parse content for citations
  const renderContentWithCitations = () => {
    if (!message.sources || message.sources.length === 0) {
      return <p className={styles.content}>{message.content}</p>;
    }

    // Split by citation pattern [N]
    const parts = message.content.split(/(\[\d+\])/g);

    return (
      <p className={styles.content}>
        {parts.map((part, index) => {
          const citationMatch = part.match(/\[(\d+)\]/);
          if (citationMatch) {
            const citationNum = parseInt(citationMatch[1]);
            const source = message.sources?.find(s => s.id === citationNum);
            if (source) {
              return (
                <CitationLink
                  key={index}
                  citationNumber={citationNum}
                  source={source}
                />
              );
            }
          }
          return <span key={index}>{part}</span>;
        })}
      </p>
    );
  };

  return (
    <div className={`${styles.messageBubble} ${styles[message.role]} ${styles[theme]}`}>
      {/* Avatar */}
      <div className={styles.avatar}>
        {message.role === 'user' ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 3c1.66 0 3 1.34 3 3s-1.34 3-3 3-3-1.34-3-3 1.34-3 3-3zm0 14.2c-2.5 0-4.71-1.28-6-3.22.03-1.99 4-3.08 6-3.08 1.99 0 5.97 1.09 6 3.08-1.29 1.94-3.5 3.22-6 3.22z" />
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
            <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2z" />
          </svg>
        )}
      </div>

      {/* Content */}
      <div className={styles.bubble}>
        {renderContentWithCitations()}

        {/* Assistant-specific features */}
        {message.role === 'assistant' && (
          <>
            {/* Confidence indicator */}
            {message.confidence !== undefined && (
              <div className={styles.confidence}>
                <span className={styles.confidenceLabel}>Confidence:</span>
                <div className={styles.confidenceBar}>
                  <div
                    className={styles.confidenceFill}
                    style={{ width: `${message.confidence * 100}%` }}
                  />
                </div>
                <span className={styles.confidenceValue}>
                  {Math.round(message.confidence * 100)}%
                </span>
              </div>
            )}

            {/* Sources toggle */}
            {message.sources && message.sources.length > 0 && (
              <button
                className={styles.sourcesToggle}
                onClick={() => setShowSources(!showSources)}
              >
                {showSources ? 'Hide' : 'Show'} {message.sources.length} source{message.sources.length > 1 ? 's' : ''}
              </button>
            )}

            {/* Sources list */}
            {showSources && message.sources && (
              <SourceList sources={message.sources} theme={theme} />
            )}

            {/* Feedback buttons */}
            {onFeedback && (
              <div className={styles.feedback}>
                <button
                  className={`${styles.feedbackButton} ${feedbackGiven === 'positive' ? styles.active : ''}`}
                  onClick={() => handleFeedback('positive')}
                  disabled={feedbackGiven !== null}
                  aria-label="Helpful"
                  title="This answer was helpful"
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M1 21h4V9H1v12zm22-11c0-1.1-.9-2-2-2h-6.31l.95-4.57.03-.32c0-.41-.17-.79-.44-1.06L14.17 1 7.59 7.59C7.22 7.95 7 8.45 7 9v10c0 1.1.9 2 2 2h9c.83 0 1.54-.5 1.84-1.22l3.02-7.05c.09-.23.14-.47.14-.73v-2z" />
                  </svg>
                </button>
                <button
                  className={`${styles.feedbackButton} ${feedbackGiven === 'negative' ? styles.active : ''}`}
                  onClick={() => handleFeedback('negative')}
                  disabled={feedbackGiven !== null}
                  aria-label="Not helpful"
                  title="This answer was not helpful"
                >
                  <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                    <path d="M15 3H6c-.83 0-1.54.5-1.84 1.22l-3.02 7.05c-.09.23-.14.47-.14.73v2c0 1.1.9 2 2 2h6.31l-.95 4.57-.03.32c0 .41.17.79.44 1.06L9.83 23l6.59-6.59c.36-.36.58-.86.58-1.41V5c0-1.1-.9-2-2-2zm4 0v12h4V3h-4z" />
                  </svg>
                </button>
              </div>
            )}
          </>
        )}

        {/* Timestamp */}
        <div className={styles.timestamp}>
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      </div>
    </div>
  );
};

export default MessageBubble;
