/**
 * FocusAreasSelector component for targeted personalization.
 *
 * Features:
 * - Multi-select checkboxes for focus areas
 * - Predefined focus areas relevant to AI/robotics education
 * - Custom focus area input
 * - Collapsible/expandable UI
 */
import React, { useState } from 'react';
import './FocusAreasSelector.css';

interface FocusAreasSelectorProps {
  /** Currently selected focus areas */
  selectedAreas?: string[];
  /** Callback when selection changes */
  onChange: (areas: string[]) => void;
  /** Whether the selector is disabled */
  disabled?: boolean;
}

// Predefined focus areas for AI/robotics education
const PREDEFINED_FOCUS_AREAS = [
  {
    id: 'practical-examples',
    label: 'Practical Examples',
    description: 'Real-world code examples and implementations',
  },
  {
    id: 'theoretical-depth',
    label: 'Theoretical Depth',
    description: 'Mathematical foundations and theory',
  },
  {
    id: 'hardware-specific',
    label: 'Hardware Recommendations',
    description: 'Hardware-specific guidance and optimizations',
  },
  {
    id: 'step-by-step',
    label: 'Step-by-Step Guides',
    description: 'Detailed, beginner-friendly walkthroughs',
  },
  {
    id: 'advanced-topics',
    label: 'Advanced Topics',
    description: 'Cutting-edge techniques and research',
  },
  {
    id: 'troubleshooting',
    label: 'Troubleshooting Tips',
    description: 'Common issues and solutions',
  },
];

export function FocusAreasSelector({
  selectedAreas = [],
  onChange,
  disabled = false,
}: FocusAreasSelectorProps) {
  const [isExpanded, setIsExpanded] = useState(false);
  const [customArea, setCustomArea] = useState('');

  const handleToggle = (areaId: string) => {
    if (disabled) return;

    const newAreas = selectedAreas.includes(areaId)
      ? selectedAreas.filter((id) => id !== areaId)
      : [...selectedAreas, areaId];

    onChange(newAreas);
  };

  const handleAddCustom = () => {
    if (!customArea.trim() || disabled) return;

    const customId = `custom-${customArea.toLowerCase().replace(/\s+/g, '-')}`;
    if (!selectedAreas.includes(customId)) {
      onChange([...selectedAreas, customId]);
    }
    setCustomArea('');
  };

  const handleRemoveCustom = (areaId: string) => {
    if (disabled) return;
    onChange(selectedAreas.filter((id) => id !== areaId));
  };

  const customAreas = selectedAreas.filter(
    (area) => area.startsWith('custom-') && !PREDEFINED_FOCUS_AREAS.find((p) => p.id === area)
  );

  return (
    <div className={`focus-areas-selector ${disabled ? 'focus-areas-selector--disabled' : ''}`}>
      <button
        type="button"
        onClick={() => setIsExpanded(!isExpanded)}
        className="focus-areas-selector__toggle"
        disabled={disabled}
        aria-expanded={isExpanded}
        title={isExpanded ? 'Collapse focus areas' : 'Click to select focus areas (optional)'}
      >
        <span className="focus-areas-selector__toggle-icon">{isExpanded ? '▼' : '▶'}</span>
        <span className="focus-areas-selector__toggle-text">
          {isExpanded ? '▼ ' : '▶ '}Focus Areas {selectedAreas.length > 0 ? `(${selectedAreas.length} selected)` : '(Click to select)'}
        </span>
      </button>

      {isExpanded && (
        <div className="focus-areas-selector__content">
          <p className="focus-areas-selector__description">
            Select specific topics to emphasize in personalization and translation (optional):
          </p>

          <div className="focus-areas-selector__grid">
            {PREDEFINED_FOCUS_AREAS.map((area) => (
              <label
                key={area.id}
                className={`focus-area-item ${
                  selectedAreas.includes(area.id) ? 'focus-area-item--selected' : ''
                }`}
              >
                <input
                  type="checkbox"
                  checked={selectedAreas.includes(area.id)}
                  onChange={() => handleToggle(area.id)}
                  disabled={disabled}
                  className="focus-area-item__checkbox"
                />
                <div className="focus-area-item__content">
                  <span className="focus-area-item__label">{area.label}</span>
                  <span className="focus-area-item__description">{area.description}</span>
                </div>
              </label>
            ))}
          </div>

          {/* Custom Focus Areas */}
          {customAreas.length > 0 && (
            <div className="focus-areas-selector__custom-list">
              <p className="focus-areas-selector__custom-title">Custom Focus Areas:</p>
              <div className="focus-areas-selector__custom-tags">
                {customAreas.map((area) => (
                  <span key={area} className="focus-area-tag">
                    {area.replace('custom-', '').replace(/-/g, ' ')}
                    <button
                      type="button"
                      onClick={() => handleRemoveCustom(area)}
                      className="focus-area-tag__remove"
                      disabled={disabled}
                      aria-label="Remove"
                    >
                      ✕
                    </button>
                  </span>
                ))}
              </div>
            </div>
          )}

          {/* Add Custom Focus Area */}
          <div className="focus-areas-selector__custom-input">
            <input
              type="text"
              value={customArea}
              onChange={(e) => setCustomArea(e.target.value)}
              onKeyPress={(e) => {
                if (e.key === 'Enter') {
                  e.preventDefault();
                  handleAddCustom();
                }
              }}
              placeholder="Add custom focus area..."
              className="focus-areas-selector__input"
              disabled={disabled}
              maxLength={50}
            />
            <button
              type="button"
              onClick={handleAddCustom}
              className="focus-areas-selector__add-button"
              disabled={disabled || !customArea.trim()}
            >
              + Add
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
