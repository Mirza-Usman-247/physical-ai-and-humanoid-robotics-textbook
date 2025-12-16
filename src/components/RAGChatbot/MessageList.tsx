/**
 * MessageList Component
 * Displays list of chat messages with auto-scroll.
 * Implements FR-008: Display conversation history.
 */

import React, { useEffect, useRef } from 'react';
import MessageBubble from './MessageBubble';
import LoadingIndicator from './LoadingIndicator';
import ErrorMessage from './ErrorMessage';
import styles from './MessageList.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: CitationSource[];
  confidence?: number;
  timestamp: Date;
  isLoading?: boolean;
  error?: string;
}

interface CitationSource {
  id: number;
  file_path: string;
  line_range: string;
  section: string;
  score: number;
}

interface MessageListProps {
  messages: Message[];
  onFeedback?: (messageId: string, rating: 'positive' | 'negative') => void;
  theme?: 'light' | 'dark';
}

/**
 * Scrollable message list with auto-scroll to bottom.
 *
 * Features:
 * - Auto-scroll on new messages
 * - Smooth scrolling
 * - Loading states
 * - Error display
 */
const MessageList: React.FC<MessageListProps> = ({
  messages,
  onFeedback,
  theme = 'light'
}) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  return (
    <div ref={containerRef} className={`${styles.messageList} ${styles[theme]}`}>
      {messages.map((message, index) => (
        <div key={message.id} className={styles.messageWrapper}>
          {message.isLoading ? (
            <div className={styles.loadingWrapper}>
              <LoadingIndicator />
            </div>
          ) : message.error ? (
            <div className={styles.errorWrapper}>
              <ErrorMessage message={message.error} />
            </div>
          ) : (
            <MessageBubble
              message={message}
              onFeedback={onFeedback}
              theme={theme}
            />
          )}
        </div>
      ))}
      <div ref={messagesEndRef} />
    </div>
  );
};

export default MessageList;
