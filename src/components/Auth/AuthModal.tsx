/**
 * AuthModal component for signup/signin modals.
 *
 * Features:
 * - Modal dialog for authentication forms
 * - Switch between signup and signin modes
 * - Integration with SignupForm and SigninForm
 * - Closes on successful authentication
 */
import React, { useState, useEffect } from 'react';
import { SignupForm } from './SignupForm';
import { SigninForm } from './SigninForm';
import './AuthModal.css';

export type AuthModalMode = 'signin' | 'signup' | null;

interface AuthModalProps {
  isOpen: boolean;
  mode: AuthModalMode;
  onClose: () => void;
  onModeChange: (mode: AuthModalMode) => void;
}

export function AuthModal({ isOpen, mode, onClose, onModeChange }: AuthModalProps) {
  // Close modal on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
      // Prevent body scroll when modal is open
      document.body.style.overflow = 'hidden';
    }

    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = '';
    };
  }, [isOpen, onClose]);

  const handleSuccess = () => {
    // Close modal on successful authentication
    onClose();
  };

  const handleSwitchMode = () => {
    onModeChange(mode === 'signin' ? 'signup' : 'signin');
  };

  if (!isOpen || !mode) {
    return null;
  }

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div
        className="auth-modal-content"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-labelledby="auth-modal-title"
      >
        <button
          className="auth-modal-close"
          onClick={onClose}
          aria-label="Close modal"
        >
          ×
        </button>

        <div className="auth-modal-body">
          {mode === 'signup' ? (
            <SignupForm
              onSuccess={handleSuccess}
              onSwitchToSignin={handleSwitchMode}
            />
          ) : (
            <SigninForm
              onSuccess={handleSuccess}
              onSwitchToSignup={handleSwitchMode}
            />
          )}
        </div>
      </div>
    </div>
  );
}
