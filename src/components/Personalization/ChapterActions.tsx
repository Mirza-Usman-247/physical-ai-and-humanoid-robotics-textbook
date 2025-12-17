/**
 * ChapterActions component for chapter-level actions.
 *
 * Features:
 * - Displays PersonalizeButton for logged-in users
 * - FocusAreasSelector for targeted personalization
 * - TranslateButton for Urdu translation
 * - PersonalizedContentViewer for displaying transformed content
 * - Enforces one-transformation-at-a-time constraint
 * - Handles chapter content extraction and personalized/translated content display
 */
import React, { useState, useEffect } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { PersonalizeButton } from './PersonalizeButton';
import { TranslateButton } from './TranslateButton';
import { FocusAreasSelector } from './FocusAreasSelector';
import { PersonalizedContentViewer } from './PersonalizedContentViewer';
import { getPersonalizedChapter, getTranslatedChapter } from '../../lib/storage';
import './ChapterActions.css';
import './rtl-support.css';

interface ChapterActionsProps {
  /** Unique identifier for the chapter (e.g., "module-0-chapter-1") */
  chapterId: string;
  /** Placement of actions (top or bottom of chapter) */
  position?: 'top' | 'bottom';
}

type TransformationType = 'personalized' | 'translated' | null;

export function ChapterActions({
  chapterId,
  position = 'top',
}: ChapterActionsProps) {
  const { user, isAuthenticated } = useAuth();

  console.log('🔵 ChapterActions mounted:', { chapterId, isAuthenticated, userEmail: user?.email });
  const [chapterContent, setChapterContent] = useState<string>('');
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [personalizedMetadata, setPersonalizedMetadata] = useState<any>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [translatedMetadata, setTranslatedMetadata] = useState<any>(null);
  const [activeTransformation, setActiveTransformation] = useState<TransformationType>(null);
  const [showTransformed, setShowTransformed] = useState(false);
  const [focusAreas, setFocusAreas] = useState<string[]>([]);

  // Extract chapter content from the DOM on mount
  useEffect(() => {
    extractChapterContent();
    checkForStoredContent();
  }, [chapterId, user]);

  const extractChapterContent = () => {
    // Get the main content area (Docusaurus article element)
    const articleElement = document.querySelector('article');
    if (!articleElement) {
      console.warn('ChapterActions: Could not find article element');
      return;
    }

    // Extract markdown content (simplified - just get text content)
    // In production, you might want to use a markdown serializer
    const content = articleElement.textContent || '';
    setChapterContent(content);
  };

  const checkForStoredContent = async () => {
    if (!isAuthenticated || !user) return;

    try {
      // Check for personalized content
      const storedPersonalized = await getPersonalizedChapter(chapterId, user.email);
      if (storedPersonalized) {
        setPersonalizedContent(storedPersonalized.content);
        setPersonalizedMetadata(storedPersonalized.metadata);
      }

      // Check for translated content
      const storedTranslated = await getTranslatedChapter(chapterId, user.email);
      if (storedTranslated) {
        setTranslatedContent(storedTranslated.content);
        setTranslatedMetadata(storedTranslated.metadata);
      }
    } catch (err) {
      console.error('Error checking for stored content:', err);
    }
  };

  const handlePersonalized = (content: string, metadata?: any) => {
    setPersonalizedContent(content);
    setPersonalizedMetadata(metadata);
    setActiveTransformation('personalized');
    setShowTransformed(true);
  };

  const handleTranslated = (content: string, metadata?: any) => {
    setTranslatedContent(content);
    setTranslatedMetadata(metadata);
    setActiveTransformation('translated');
    setShowTransformed(true);
  };

  const handleFocusAreasChange = (areas: string[]) => {
    setFocusAreas(areas);
  };

  const toggleView = () => {
    setShowTransformed(!showTransformed);
  };

  const switchTransformation = (type: TransformationType) => {
    setActiveTransformation(type);
    setShowTransformed(type !== null);
  };

  // Don't render if not authenticated
  if (!isAuthenticated) {
    console.log('🔴 ChapterActions: User not authenticated');
    return null;
  }

  console.log('✅ ChapterActions: User authenticated', user?.email);

  // Don't render if no chapter content
  if (!chapterContent) {
    console.log('🔴 ChapterActions: No chapter content found');
    return null;
  }

  console.log('✅ ChapterActions: Rendering with content length:', chapterContent.length);

  return (
    <div className={`chapter-actions chapter-actions--${position}`}>
      <div className="chapter-actions__header">
        <h4 className="chapter-actions__title">
          {position === 'top' ? '📚 Chapter Tools' : '💡 Personalize Your Learning'}
        </h4>
        <p className="chapter-actions__description">
          Adapt this content to match your skill level and available hardware
        </p>
      </div>

      {/* Focus Areas Selector */}
      <FocusAreasSelector
        selectedAreas={focusAreas}
        onChange={handleFocusAreasChange}
        disabled={!chapterContent}
      />

      <div className="chapter-actions__buttons">
        {/* Personalize Button */}
        <PersonalizeButton
          chapterId={chapterId}
          chapterContent={chapterContent}
          onPersonalized={handlePersonalized}
          focusAreas={focusAreas}
          disabled={activeTransformation === 'translated'}
        />

        {/* Translate Button */}
        <TranslateButton
          chapterId={chapterId}
          chapterContent={chapterContent}
          onTranslated={handleTranslated}
          disabled={activeTransformation === 'personalized'}
          focusAreas={focusAreas}
        />

        {/* Switch Transformation Buttons */}
        {personalizedContent && translatedContent && (
          <div className="chapter-actions__switch">
            <button
              onClick={() => switchTransformation('personalized')}
              className={`chapter-actions__switch-btn ${
                activeTransformation === 'personalized' ? 'active' : ''
              }`}
              title="View personalized content"
            >
              🎯 Personalized
            </button>
            <button
              onClick={() => switchTransformation('translated')}
              className={`chapter-actions__switch-btn ${
                activeTransformation === 'translated' ? 'active' : ''
              }`}
              title="View translated content"
            >
              🌐 اردو
            </button>
          </div>
        )}

        {/* Toggle Original/Transformed */}
        {(personalizedContent || translatedContent) && (
          <button
            onClick={toggleView}
            className="chapter-actions__toggle"
            title={showTransformed ? 'Show original content' : 'Show transformed content'}
          >
            {showTransformed ? '📄 Show Original' : '✨ View Transformed'}
          </button>
        )}
      </div>

      {/* One-Transformation-at-a-Time Notice */}
      {(activeTransformation === 'personalized' || activeTransformation === 'translated') && (
        <div className="chapter-actions__notice">
          <span className="chapter-actions__notice-icon">ℹ️</span>
          <span className="chapter-actions__notice-text">
            {activeTransformation === 'personalized'
              ? 'Personalization active. Disable to translate.'
              : 'Translation active. Disable to personalize.'}
          </span>
        </div>
      )}

      {/* Transformed Content Viewer */}
      {showTransformed && activeTransformation === 'personalized' && personalizedContent && (
        <PersonalizedContentViewer
          content={personalizedContent}
          metadata={personalizedMetadata}
          onClose={() => setShowTransformed(false)}
          showMetadata={true}
          title="Personalized Chapter Content"
        />
      )}

      {showTransformed && activeTransformation === 'translated' && translatedContent && (
        <div className={`rtl-content ${activeTransformation === 'translated' ? 'rtl-content' : ''}`}>
          <div className="language-badge language-badge--urdu">
            Urdu Translation (اردو ترجمہ)
          </div>
          <PersonalizedContentViewer
            content={translatedContent}
            metadata={translatedMetadata}
            onClose={() => setShowTransformed(false)}
            showMetadata={true}
            title="Urdu Translation (اردو ترجمہ)"
          />
        </div>
      )}
    </div>
  );
}
