/**
 * ChatInterface Component
 * Main chat interface managing conversation state and API calls.
 * Implements FR-001 through FR-015.
 */

import React, { useState, useEffect, useCallback } from 'react';
import MessageList from './MessageList';
import InputBox from './InputBox';
import LoadingIndicator from './LoadingIndicator';
import ErrorMessage from './ErrorMessage';
import styles from './ChatInterface.module.css';

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

interface ChatInterfaceProps {
  apiBaseUrl: string;
  onClose: () => void;
  onNewMessage?: () => void;
  theme?: 'light' | 'dark';
}

/**
 * Chat interface with message history and input.
 *
 * Features:
 * - Real-time messaging
 * - Conversation persistence
 * - Selected text context
 * - Error handling
 * - Loading states
 */
const ChatInterface: React.FC<ChatInterfaceProps> = ({
  apiBaseUrl,
  onClose,
  onNewMessage,
  theme = 'light'
}) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [sessionId] = useState(() => {
    // Get or create session ID
    let id = localStorage.getItem('rag-chatbot-session-id');
    if (!id) {
      id = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('rag-chatbot-session-id', id);
    }
    return id;
  });

  // Load conversation history on mount
  useEffect(() => {
    const savedConversationId = localStorage.getItem('rag-chatbot-conversation-id');
    if (savedConversationId) {
      setConversationId(savedConversationId);
      loadConversationHistory(savedConversationId);
    }
  }, []);

  const loadConversationHistory = async (convId: string) => {
    try {
      const response = await fetch(
        `${apiBaseUrl}/chat/history?conversation_id=${convId}&limit=50`,
        {
          headers: {
            'X-Session-ID': sessionId
          }
        }
      );

      if (response.ok) {
        const data = await response.json();
        const loadedMessages: Message[] = data.messages.map((msg: any) => ({
          id: msg.message_id,
          role: msg.role,
          content: msg.content,
          sources: msg.role === 'assistant' ? [] : undefined, // Sources loaded separately
          confidence: msg.confidence,
          timestamp: new Date(msg.timestamp)
        }));
        setMessages(loadedMessages);
      }
    } catch (err) {
      console.error('Failed to load conversation history:', err);
    }
  };

  const handleSendMessage = useCallback(async (
    query: string,
    selectedText?: string
  ) => {
    if (!query.trim()) return;

    // Clear previous error
    setError(null);

    // Add user message
    const userMessage: Message = {
      id: `temp-user-${Date.now()}`,
      role: 'user',
      content: query,
      timestamp: new Date()
    };
    setMessages(prev => [...prev, userMessage]);

    // Add loading message
    const loadingMessage: Message = {
      id: `temp-loading-${Date.now()}`,
      role: 'assistant',
      content: '',
      timestamp: new Date(),
      isLoading: true
    };
    setMessages(prev => [...prev, loadingMessage]);
    setIsLoading(true);

    try {
      const response = await fetch(`${apiBaseUrl}/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Session-ID': sessionId
        },
        body: JSON.stringify({
          query,
          selected_text: selectedText || null,
          conversation_id: conversationId || null,
          answer_length: 'medium',
          answer_style: 'technical'
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Failed to get response');
      }

      const data = await response.json();

      // Update conversation ID
      if (!conversationId) {
        setConversationId(data.conversation_id);
        localStorage.setItem('rag-chatbot-conversation-id', data.conversation_id);
      }

      // Replace loading message with actual response
      setMessages(prev => {
        const filtered = prev.filter(m => m.id !== loadingMessage.id);
        return [
          ...filtered,
          {
            id: data.message_id,
            role: 'assistant',
            content: data.answer,
            sources: data.sources,
            confidence: data.confidence,
            timestamp: new Date(data.timestamp)
          }
        ];
      });

      // Notify parent
      if (onNewMessage) {
        onNewMessage();
      }
    } catch (err) {
      console.error('Failed to send message:', err);
      const errorMsg = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMsg);

      // Replace loading message with error
      setMessages(prev => {
        const filtered = prev.filter(m => m.id !== loadingMessage.id);
        return [
          ...filtered,
          {
            id: `error-${Date.now()}`,
            role: 'assistant',
            content: '',
            timestamp: new Date(),
            error: errorMsg
          }
        ];
      });
    } finally {
      setIsLoading(false);
    }
  }, [apiBaseUrl, sessionId, conversationId, onNewMessage]);

  const handleNewConversation = () => {
    setMessages([]);
    setConversationId(null);
    setError(null);
    localStorage.removeItem('rag-chatbot-conversation-id');
  };

  const handleFeedback = async (messageId: string, rating: 'positive' | 'negative') => {
    try {
      await fetch(`${apiBaseUrl}/chat/feedback`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-Session-ID': sessionId
        },
        body: JSON.stringify({
          message_id: messageId,
          rating
        })
      });
    } catch (err) {
      console.error('Failed to submit feedback:', err);
    }
  };

  return (
    <div className={`${styles.chatInterface} ${styles[theme]}`}>
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.headerContent}>
          <h3 className={styles.title}>Physical AI Assistant</h3>
          <p className={styles.subtitle}>Ask me about robotics and humanoid AI</p>
        </div>
        <div className={styles.headerActions}>
          <button
            className={styles.iconButton}
            onClick={handleNewConversation}
            aria-label="New conversation"
            title="Start new conversation"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M12 5v14M5 12h14" />
            </svg>
          </button>
          <button
            className={styles.iconButton}
            onClick={onClose}
            aria-label="Close"
            title="Close chatbot"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div className={styles.emptyState}>
            <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
              <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
            </svg>
            <h4>Welcome to the Physical AI Assistant!</h4>
            <p>Ask me anything about robotics, ROS2, kinematics, or humanoid AI.</p>
            <div className={styles.suggestions}>
              <button onClick={() => handleSendMessage('What is forward kinematics?')}>
                What is forward kinematics?
              </button>
              <button onClick={() => handleSendMessage('How do I create a ROS2 node?')}>
                How do I create a ROS2 node?
              </button>
              <button onClick={() => handleSendMessage('Explain the DH convention')}>
                Explain the DH convention
              </button>
            </div>
          </div>
        ) : (
          <MessageList
            messages={messages}
            onFeedback={handleFeedback}
            theme={theme}
          />
        )}
      </div>

      {/* Error */}
      {error && (
        <ErrorMessage
          message={error}
          onDismiss={() => setError(null)}
        />
      )}

      {/* Input */}
      <div className={styles.inputContainer}>
        <InputBox
          onSend={handleSendMessage}
          disabled={isLoading}
          placeholder="Ask a question about the textbook..."
          theme={theme}
        />
      </div>
    </div>
  );
};

export default ChatInterface;
