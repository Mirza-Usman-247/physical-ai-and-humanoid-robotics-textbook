/**
 * TranslateButton component for chapter translation to Urdu.
 *
 * Features:
 * - Translates chapter to Urdu with Focus Mode
 * - Stores result in IndexedDB
 * - Shows loading state and success/error feedback
 * - Displays translated content with RTL support
 */
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { translateToUrdu, type TranslateRequest } from '../../api/translation';
import { saveTranslatedChapter } from '../../lib/storage';
import './TranslateButton.css';

interface TranslateButtonProps {
  chapterId: string;
  chapterContent: string;
  onTranslated?: (content: string, metadata?: any) => void;
  disabled?: boolean;
  focusAreas?: string[];
}

export function TranslateButton({
  chapterId,
  chapterContent,
  onTranslated,
  disabled = false,
  focusAreas = [],
}: TranslateButtonProps) {
  const { user, tokens, isAuthenticated } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  const handleTranslate = async () => {
    if (!isAuthenticated || !tokens) {
      setError('Please sign in to translate content');
      return;
    }

    if (!user) {
      setError('User profile not loaded');
      return;
    }

    setIsLoading(true);
    setError(null);
    setSuccess(false);

    try {
      const response = await translateToUrdu(
        chapterId,
        chapterContent,
        tokens.access_token,
        focusAreas
      );

      // Save to IndexedDB
      await saveTranslatedChapter(
        response.chapter_id,
        response.translated_content,
        response.transformation_metadata,
        user.email,
        'ur' // Urdu language code
      );

      // Notify parent component
      if (onTranslated) {
        onTranslated(response.translated_content, response.transformation_metadata);
      }

      setSuccess(true);
      setTimeout(() => setSuccess(false), 3000);
    } catch (err: any) {
      setError(err.message || 'Translation failed');
      console.error('Translation error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isAuthenticated) {
    return (
      <button
        className="translate-button translate-button--disabled"
        disabled
        title="Sign in to translate content"
      >
        🌐 Translate (Sign in required)
      </button>
    );
  }

  return (
    <div className="translate-button-container">
      <button
        onClick={handleTranslate}
        className={`translate-button ${isLoading ? 'translate-button--loading' : ''} ${
          success ? 'translate-button--success' : ''
        } ${error ? 'translate-button--error' : ''}`}
        disabled={isLoading || disabled}
        title="Translate chapter to Urdu (Focus Mode: faithful translation)"
      >
        {isLoading ? (
          <>
            <span className="translate-button__icon">⚙️</span>
            <span className="translate-button__text">Translating...</span>
          </>
        ) : success ? (
          <>
            <span className="translate-button__icon">✓</span>
            <span className="translate-button__text">Translated!</span>
          </>
        ) : (
          <>
            <span className="translate-button__icon">🌐</span>
            <span className="translate-button__text">Translate to Urdu</span>
          </>
        )}
      </button>

      {error && (
        <div className="translate-error" role="alert">
          <span className="translate-error__icon">⚠️</span>
          <span className="translate-error__text">{error}</span>
        </div>
      )}

      {success && (
        <div className="translate-success" role="status">
          <span className="translate-success__icon">✓</span>
          <span className="translate-success__text">
            Chapter translated to Urdu! (Stored locally)
          </span>
        </div>
      )}
    </div>
  );
}
