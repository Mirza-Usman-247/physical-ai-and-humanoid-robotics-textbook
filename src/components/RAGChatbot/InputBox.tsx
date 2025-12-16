/**
 * InputBox Component
 * Text input with send button and selection detection.
 * Implements FR-002, FR-006, FR-014.
 */

import React, { useState, useEffect, useRef, KeyboardEvent } from 'react';
import styles from './InputBox.module.css';

interface InputBoxProps {
  onSend: (query: string, selectedText?: string) => void;
  disabled?: boolean;
  placeholder?: string;
  theme?: 'light' | 'dark';
}

/**
 * Input box with text selection detection and send functionality.
 *
 * Features:
 * - Multi-line textarea
 * - Enter to send, Shift+Enter for newline
 * - Selected text detection from page
 * - Character count
 * - Clear button
 */
const InputBox: React.FC<InputBoxProps> = ({
  onSend,
  disabled = false,
  placeholder = 'Ask a question...',
  theme = 'light'
}) => {
  const [query, setQuery] = useState('');
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const lastSelectionRef = useRef<string | null>(null);

  // Auto-resize textarea based on content
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 120)}px`;
    }
  }, [query]);

  // Store selection immediately when user selects text
  useEffect(() => {
    const captureSelection = () => {
      const selection = window.getSelection();
      if (!selection || selection.rangeCount === 0) return;

      const text = selection.toString().trim();
      if (!text || text.length === 0 || text.length > 2000) {
        lastSelectionRef.current = null;
        return;
      }

      try {
        const range = selection.getRangeAt(0);
        const container = range.commonAncestorContainer;
        const parentElement = container.nodeType === Node.TEXT_NODE
          ? container.parentElement
          : container as Element;

        // Only store if outside chatbot
        const isChatbotSelection = parentElement?.closest('[class*="chatWidget"], [class*="chatInterface"], [class*="inputBox"]');

        if (!isChatbotSelection) {
          lastSelectionRef.current = text;
        }
      } catch (e) {
        console.error('Selection capture error:', e);
      }
    };

    // Capture on mouseup (when selection is complete)
    const handleMouseUp = () => {
      // Small delay to ensure selection is finalized
      setTimeout(captureSelection, 10);
    };

    // Capture periodically while text is selected
    const handleSelectionChange = () => {
      captureSelection();
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('selectionchange', handleSelectionChange);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  const handleSend = () => {
    if (query.trim() && !disabled) {
      onSend(query.trim(), selectedText || undefined);
      setQuery('');
      // Don't clear selected text immediately - let user send multiple queries with same context
      // setSelectedText(null);
      // Don't clear page selection - user might want to keep it
      // window.getSelection()?.removeAllRanges();
    }
  };

  const handleCaptureSelection = () => {
    // Try current selection first
    const selection = window.getSelection();
    const text = selection?.toString().trim();
    if (text && text.length > 0 && text.length <= 2000) {
      setSelectedText(text);
      lastSelectionRef.current = text;
    } else if (lastSelectionRef.current) {
      // Fall back to last stored selection
      setSelectedText(lastSelectionRef.current);
    }
  };

  // Capture selection from ref when textarea gets focus or user clicks capture button
  const handleTextareaFocus = () => {
    if (!selectedText && lastSelectionRef.current) {
      setSelectedText(lastSelectionRef.current);
    }
  };

  // Also trigger on mousedown (before focus clears selection)
  const handleTextareaMouseDown = () => {
    if (!selectedText && lastSelectionRef.current) {
      setSelectedText(lastSelectionRef.current);
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleClear = () => {
    setQuery('');
    setSelectedText(null);
  };

  return (
    <div className={`${styles.inputBox} ${styles[theme]}`}>
      {/* Selected text indicator */}
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
          </svg>
          <span>Context: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</span>
          <button
            onClick={() => setSelectedText(null)}
            aria-label="Remove selected text"
            title="Remove context"
          >
            <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      )}

      {/* Manual selection capture button */}
      {!selectedText && (
        <div className={styles.selectionHint}>
          <button
            className={styles.captureButton}
            onClick={handleCaptureSelection}
            aria-label="Use selected text as context"
            title="Click to use selected text from page as context"
          >
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
            </svg>
            <span>Use selected text as context</span>
          </button>
        </div>
      )}

      {/* Input area */}
      <div className={styles.inputWrapper}>
        <textarea
          ref={textareaRef}
          className={styles.textarea}
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          onKeyDown={handleKeyDown}
          onFocus={handleTextareaFocus}
          onMouseDown={handleTextareaMouseDown}
          placeholder={placeholder}
          disabled={disabled}
          rows={1}
          maxLength={2000}
        />

        {/* Actions */}
        <div className={styles.actions}>
          {query && (
            <button
              className={styles.clearButton}
              onClick={handleClear}
              aria-label="Clear input"
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="18" y1="6" x2="6" y2="18" />
                <line x1="6" y1="6" x2="18" y2="18" />
              </svg>
            </button>
          )}

          <button
            className={styles.sendButton}
            onClick={handleSend}
            disabled={!query.trim() || disabled}
            aria-label="Send message"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <line x1="22" y1="2" x2="11" y2="13" />
              <polygon points="22 2 15 22 11 13 2 9 22 2" />
            </svg>
          </button>
        </div>
      </div>

      {/* Character count */}
      <div className={styles.footer}>
        <span className={styles.charCount}>
          {query.length} / 2000
        </span>
        <span className={styles.hint}>
          Press Enter to send, Shift+Enter for new line
        </span>
      </div>
    </div>
  );
};

export default InputBox;
