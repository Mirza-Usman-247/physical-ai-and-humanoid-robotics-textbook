/**
 * ChatWidget Component
 * Main chatbot widget with toggle button and floating interface.
 * Implements FR-005: Toggle chatbot visibility with button.
 */

import React, { useState, useEffect } from 'react';
import ChatInterface from './ChatInterface';
import styles from './ChatWidget.module.css';

interface ChatWidgetProps {
  /** API base URL for backend */
  apiBaseUrl: string;
  /** Initial open state */
  initialOpen?: boolean;
  /** Custom theme */
  theme?: 'light' | 'dark';
}

/**
 * Floating chat widget with toggle button.
 *
 * Features:
 * - Floating button in bottom-right corner
 * - Expandable chat interface
 * - Smooth animations
 * - Responsive design
 * - Persistent open/closed state
 */
const ChatWidget: React.FC<ChatWidgetProps> = ({
  apiBaseUrl,
  initialOpen = false,
  theme = 'light'
}) => {
  const [isOpen, setIsOpen] = useState(initialOpen);
  const [unreadCount, setUnreadCount] = useState(0);

  // Load persisted state from localStorage
  useEffect(() => {
    const savedState = localStorage.getItem('rag-chatbot-open');
    if (savedState !== null) {
      setIsOpen(savedState === 'true');
    }
  }, []);

  // Persist state to localStorage
  useEffect(() => {
    localStorage.setItem('rag-chatbot-open', String(isOpen));
  }, [isOpen]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setUnreadCount(0); // Clear unread count when opening
    }
  };

  const handleNewMessage = () => {
    if (!isOpen) {
      setUnreadCount(prev => prev + 1);
    }
  };

  return (
    <div className={`${styles.chatWidget} ${styles[theme]}`}>
      {/* Chat Interface */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <ChatInterface
            apiBaseUrl={apiBaseUrl}
            onClose={() => setIsOpen(false)}
            onNewMessage={handleNewMessage}
            theme={theme}
          />
        </div>
      )}

      {/* Toggle Button */}
      <button
        className={styles.toggleButton}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chatbot' : 'Open chatbot'}
        aria-expanded={isOpen}
      >
        {isOpen ? (
          // Close icon
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <line x1="18" y1="6" x2="6" y2="18" />
            <line x1="6" y1="6" x2="18" y2="18" />
          </svg>
        ) : (
          <>
            {/* Chat icon */}
            <svg
              width="24"
              height="24"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
            </svg>
            {/* Unread badge */}
            {unreadCount > 0 && (
              <span className={styles.unreadBadge}>
                {unreadCount > 9 ? '9+' : unreadCount}
              </span>
            )}
          </>
        )}
      </button>
    </div>
  );
};

export default ChatWidget;
