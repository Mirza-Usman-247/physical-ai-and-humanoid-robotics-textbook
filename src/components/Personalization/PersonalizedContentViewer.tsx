/**
 * PersonalizedContentViewer component for displaying personalized chapter content.
 *
 * Features:
 * - Renders markdown content with proper formatting
 * - Shows personalization metadata (skill levels, hardware, focus areas)
 * - Provides toggle between personalized and original content
 * - Supports printing and copying content
 */
import React, { useState } from 'react';
import './PersonalizedContentViewer.css';

interface PersonalizedContentViewerProps {
  /** Personalized markdown content (can be HTML or markdown) */
  content: string;
  /** Metadata about the personalization */
  metadata?: {
    ai_level?: number;
    ml_level?: number;
    ros_level?: number;
    python_level?: number;
    linux_level?: number;
    has_gpu?: boolean;
    has_jetson?: boolean;
    has_robot?: boolean;
    focus_areas?: string[];
    personalized_at?: string;
  };
  /** Callback when user closes the viewer */
  onClose?: () => void;
  /** Whether to show metadata section */
  showMetadata?: boolean;
  /** Optional title for the viewer */
  title?: string;
}

export function PersonalizedContentViewer({
  content,
  metadata,
  onClose,
  showMetadata = true,
  title = 'Personalized Content',
}: PersonalizedContentViewerProps) {
  const [isCopied, setIsCopied] = useState(false);

  const handleCopy = async () => {
    try {
      // Create a temporary div to extract text from HTML
      const tempDiv = document.createElement('div');
      tempDiv.innerHTML = content;
      const textContent = tempDiv.textContent || tempDiv.innerText || '';

      await navigator.clipboard.writeText(textContent);
      setIsCopied(true);
      setTimeout(() => setIsCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy content:', err);
    }
  };

  const handlePrint = () => {
    const printWindow = window.open('', '_blank');
    if (printWindow) {
      printWindow.document.write(`
        <!DOCTYPE html>
        <html>
          <head>
            <title>${title}</title>
            <style>
              body {
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
                line-height: 1.7;
                max-width: 800px;
                margin: 0 auto;
                padding: 2rem;
                color: #333;
              }
              h1, h2, h3, h4, h5, h6 {
                margin-top: 1.5rem;
                margin-bottom: 0.75rem;
                font-weight: 600;
              }
              p { margin-bottom: 1rem; }
              pre {
                background: #f5f5f5;
                padding: 1rem;
                border-radius: 4px;
                overflow-x: auto;
              }
              code {
                background: #f5f5f5;
                padding: 0.2rem 0.4rem;
                border-radius: 3px;
                font-size: 0.9em;
              }
              ul, ol {
                margin-bottom: 1rem;
                padding-left: 2rem;
              }
              blockquote {
                border-left: 4px solid #667eea;
                padding-left: 1rem;
                margin: 1rem 0;
                color: #555;
                font-style: italic;
              }
            </style>
          </head>
          <body>
            ${content}
          </body>
        </html>
      `);
      printWindow.document.close();
      printWindow.print();
    }
  };

  const getSkillLevelLabel = (level?: number): string => {
    if (!level) return 'N/A';
    const labels = ['N/A', 'Beginner', 'Basic', 'Intermediate', 'Advanced', 'Expert'];
    return labels[level] || 'N/A';
  };

  return (
    <div className="personalized-viewer">
      {/* Header */}
      <div className="personalized-viewer__header">
        <div className="personalized-viewer__title-section">
          <h3 className="personalized-viewer__title">
            <span className="personalized-viewer__icon">✨</span>
            {title}
          </h3>
          {metadata?.personalized_at && (
            <span className="personalized-viewer__timestamp">
              {new Date(metadata.personalized_at).toLocaleString()}
            </span>
          )}
        </div>
        <div className="personalized-viewer__actions">
          <button
            onClick={handleCopy}
            className="personalized-viewer__action-btn"
            title="Copy content"
          >
            {isCopied ? '✓ Copied!' : '📋 Copy'}
          </button>
          <button
            onClick={handlePrint}
            className="personalized-viewer__action-btn"
            title="Print content"
          >
            🖨️ Print
          </button>
          {onClose && (
            <button
              onClick={onClose}
              className="personalized-viewer__close-btn"
              aria-label="Close viewer"
            >
              ✕
            </button>
          )}
        </div>
      </div>

      {/* Metadata Section */}
      {showMetadata && metadata && (
        <div className="personalized-viewer__metadata">
          <details className="personalized-viewer__metadata-details">
            <summary className="personalized-viewer__metadata-summary">
              📊 Personalization Details
            </summary>
            <div className="personalized-viewer__metadata-content">
              {/* Skill Levels */}
              <div className="metadata-section">
                <h4 className="metadata-section__title">Skill Levels</h4>
                <div className="metadata-grid">
                  <div className="metadata-item">
                    <span className="metadata-item__label">AI:</span>
                    <span className="metadata-item__value">
                      {getSkillLevelLabel(metadata.ai_level)}
                    </span>
                  </div>
                  <div className="metadata-item">
                    <span className="metadata-item__label">ML:</span>
                    <span className="metadata-item__value">
                      {getSkillLevelLabel(metadata.ml_level)}
                    </span>
                  </div>
                  <div className="metadata-item">
                    <span className="metadata-item__label">ROS:</span>
                    <span className="metadata-item__value">
                      {getSkillLevelLabel(metadata.ros_level)}
                    </span>
                  </div>
                  <div className="metadata-item">
                    <span className="metadata-item__label">Python:</span>
                    <span className="metadata-item__value">
                      {getSkillLevelLabel(metadata.python_level)}
                    </span>
                  </div>
                  <div className="metadata-item">
                    <span className="metadata-item__label">Linux:</span>
                    <span className="metadata-item__value">
                      {getSkillLevelLabel(metadata.linux_level)}
                    </span>
                  </div>
                </div>
              </div>

              {/* Hardware Access */}
              {(metadata.has_gpu || metadata.has_jetson || metadata.has_robot) && (
                <div className="metadata-section">
                  <h4 className="metadata-section__title">Hardware Access</h4>
                  <div className="metadata-tags">
                    {metadata.has_gpu && (
                      <span className="metadata-tag metadata-tag--gpu">🎮 GPU</span>
                    )}
                    {metadata.has_jetson && (
                      <span className="metadata-tag metadata-tag--jetson">🤖 Jetson</span>
                    )}
                    {metadata.has_robot && (
                      <span className="metadata-tag metadata-tag--robot">🦾 Robot</span>
                    )}
                  </div>
                </div>
              )}

              {/* Focus Areas */}
              {metadata.focus_areas && metadata.focus_areas.length > 0 && (
                <div className="metadata-section">
                  <h4 className="metadata-section__title">Focus Areas</h4>
                  <div className="metadata-tags">
                    {metadata.focus_areas.map((area) => (
                      <span key={area} className="metadata-tag metadata-tag--focus">
                        {area.replace(/-/g, ' ').replace(/custom /g, '')}
                      </span>
                    ))}
                  </div>
                </div>
              )}
            </div>
          </details>
        </div>
      )}

      {/* Content */}
      <div
        className="personalized-viewer__content markdown"
        dangerouslySetInnerHTML={{ __html: content }}
      />
    </div>
  );
}
