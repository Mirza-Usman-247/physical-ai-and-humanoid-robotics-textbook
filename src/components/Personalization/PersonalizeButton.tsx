/**
 * PersonalizeButton component for chapter content personalization.
 *
 * Features:
 * - Personalizes chapter based on user's skill profile
 * - Stores result in IndexedDB
 * - Shows loading state and success/error feedback
 * - Displays personalized content in modal or replaces current content
 */
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { personalizeChapter, type PersonalizeRequest } from '../../api/personalize';
import { savePersonalizedChapter } from '../../lib/storage';
import './PersonalizeButton.css';

interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent: string;
  onPersonalized?: (content: string, metadata?: any) => void;
  focusAreas?: string[];
  disabled?: boolean;
}

export function PersonalizeButton({
  chapterId,
  chapterContent,
  onPersonalized,
  focusAreas,
  disabled = false,
}: PersonalizeButtonProps) {
  const { user, tokens, isAuthenticated } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  const handlePersonalize = async () => {
    if (!isAuthenticated || !tokens) {
      setError('Please sign in to personalize content');
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
      const request: PersonalizeRequest = {
        chapter_id: chapterId,
        chapter_content: chapterContent,
        focus_areas: focusAreas,
      };

      const response = await personalizeChapter(request, tokens.access_token);

      // Save to IndexedDB
      await savePersonalizedChapter(
        response.chapter_id,
        response.personalized_content,
        response.transformation_metadata,
        user.email
      );

      // Notify parent component
      if (onPersonalized) {
        onPersonalized(response.personalized_content, response.transformation_metadata);
      }

      setSuccess(true);
      setTimeout(() => setSuccess(false), 3000);
    } catch (err: any) {
      setError(err.message || 'Personalization failed');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isAuthenticated) {
    return (
      <button
        className="personalize-button personalize-button--disabled"
        disabled
        title="Sign in to personalize content"
      >
        🎯 Personalize (Sign in required)
      </button>
    );
  }

  return (
    <div className="personalize-button-container">
      <button
        onClick={handlePersonalize}
        className={`personalize-button ${isLoading ? 'personalize-button--loading' : ''} ${
          success ? 'personalize-button--success' : ''
        } ${error ? 'personalize-button--error' : ''} ${disabled ? 'personalize-button--disabled' : ''}`}
        disabled={isLoading || disabled}
        title={disabled ? "Translation active - disable to personalize" : "Adapt content to your skill level and hardware"}
      >
        {isLoading ? (
          <>
            <span className="personalize-button__icon">⚙️</span>
            <span className="personalize-button__text">Personalizing...</span>
          </>
        ) : success ? (
          <>
            <span className="personalize-button__icon">✓</span>
            <span className="personalize-button__text">Personalized!</span>
          </>
        ) : (
          <>
            <span className="personalize-button__icon">🎯</span>
            <span className="personalize-button__text">Personalize Chapter</span>
          </>
        )}
      </button>

      {error && (
        <div className="personalize-error" role="alert">
          <span className="personalize-error__icon">⚠️</span>
          <span className="personalize-error__text">{error}</span>
        </div>
      )}

      {success && (
        <div className="personalize-success" role="status">
          <span className="personalize-success__icon">✓</span>
          <span className="personalize-success__text">
            Content adapted to your profile! (Stored locally)
          </span>
        </div>
      )}
    </div>
  );
}
