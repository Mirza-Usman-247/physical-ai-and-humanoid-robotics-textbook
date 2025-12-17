/**
 * SigninForm component for user authentication.
 *
 * Features:
 * - Email/password authentication
 * - Client-side validation with Zod
 * - Integration with AuthProvider
 * - Error handling and user feedback
 */
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { signinSchema, type SigninFormData } from '../../lib/validation';

interface SigninFormProps {
  onSuccess?: () => void;
  onSwitchToSignup?: () => void;
}

export function SigninForm({ onSuccess, onSwitchToSignup }: SigninFormProps) {
  const { signin, isLoading } = useAuth();
  const [error, setError] = useState<string | null>(null);
  const [validationErrors, setValidationErrors] = useState<Record<string, string>>({});

  // Form state
  const [formData, setFormData] = useState<SigninFormData>({
    email: '',
    password: '',
  });

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));

    // Clear validation error for this field
    if (validationErrors[name]) {
      setValidationErrors((prev) => {
        const newErrors = { ...prev };
        delete newErrors[name];
        return newErrors;
      });
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setValidationErrors({});

    // Validate with Zod
    const result = signinSchema.safeParse(formData);

    if (!result.success) {
      const errors: Record<string, string> = {};
      if (result.error?.errors) {
        result.error.errors.forEach((err) => {
          if (err.path.length > 0) {
            errors[err.path[0] as string] = err.message;
          }
        });
      }
      setValidationErrors(errors);
      return;
    }

    try {
      await signin(formData.email, formData.password);

      // Success callback
      if (onSuccess) {
        onSuccess();
      }
    } catch (err: any) {
      setError(err.message || 'Invalid email or password. Please try again.');
    }
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form signin-form">
      <h2>Sign In to Your Account</h2>

      {error && (
        <div className="alert alert-error" role="alert">
          {error}
        </div>
      )}

      <div className="form-group">
        <label htmlFor="email">
          Email <span className="required">*</span>
        </label>
        <input
          type="email"
          id="email"
          name="email"
          value={formData.email}
          onChange={handleChange}
          required
          placeholder="you@example.com"
          autoComplete="email"
          className={validationErrors.email ? 'error' : ''}
        />
        {validationErrors.email && (
          <span className="error-message">{validationErrors.email}</span>
        )}
      </div>

      <div className="form-group">
        <label htmlFor="password">
          Password <span className="required">*</span>
        </label>
        <input
          type="password"
          id="password"
          name="password"
          value={formData.password}
          onChange={handleChange}
          required
          placeholder="Enter your password"
          autoComplete="current-password"
          className={validationErrors.password ? 'error' : ''}
        />
        {validationErrors.password && (
          <span className="error-message">{validationErrors.password}</span>
        )}
      </div>

      <div className="form-actions">
        <button
          type="submit"
          className="button button--primary button--lg"
          disabled={isLoading}
        >
          {isLoading ? 'Signing In...' : 'Sign In'}
        </button>

        {onSwitchToSignup && (
          <p className="form-switch">
            Don't have an account?{' '}
            <button
              type="button"
              onClick={onSwitchToSignup}
              className="link-button"
            >
              Sign Up
            </button>
          </p>
        )}
      </div>

      <div className="form-footer">
        <p className="help-text">
          <strong>Note:</strong> Your authentication tokens are stored in session storage
          and will be cleared when you close your browser.
        </p>
      </div>
    </form>
  );
}
