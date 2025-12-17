/**
 * SignupForm component for user registration.
 *
 * Features:
 * - Email/password registration
 * - Skill level selection (AI, ML, ROS, Python, Linux)
 * - Hardware access checkboxes (GPU, Jetson, Robot)
 * - Client-side validation with Zod
 * - Integration with AuthProvider
 */
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import {
  signupSchema,
  type SignupFormData,
  skillLevelLabels,
  skillLevelDescriptions,
  hardwareDescriptions,
} from '../../lib/validation';

interface SignupFormProps {
  onSuccess?: () => void;
  onSwitchToSignin?: () => void;
}

export function SignupForm({ onSuccess, onSwitchToSignin }: SignupFormProps) {
  const { signup, isLoading } = useAuth();
  const [error, setError] = useState<string | null>(null);
  const [validationErrors, setValidationErrors] = useState<Record<string, string>>({});

  // Form state
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    ai_level: 3,
    ml_level: 3,
    ros_level: 3,
    python_level: 3,
    linux_level: 3,
    has_gpu: false,
    has_jetson: false,
    has_robot: false,
  });

  // Password strength indicators
  const passwordRequirements = {
    minLength: formData.password.length >= 8,
    hasUppercase: /[A-Z]/.test(formData.password),
    hasLowercase: /[a-z]/.test(formData.password),
    hasNumber: /[0-9]/.test(formData.password),
  };

  const handleChange = (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    const { name, value, type } = e.target;

    if (type === 'checkbox') {
      const checked = (e.target as HTMLInputElement).checked;
      setFormData((prev) => ({ ...prev, [name]: checked }));
    } else if (type === 'number' || name.includes('_level')) {
      setFormData((prev) => ({ ...prev, [name]: parseInt(value, 10) }));
    } else {
      setFormData((prev) => ({ ...prev, [name]: value }));
    }

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
    console.log('🔵 Signup form submitted');
    setError(null);
    setValidationErrors({});

    // Validate with Zod
    console.log('🔵 Validating form data:', formData);
    const result = signupSchema.safeParse(formData);

    if (!result.success) {
      console.log('🔴 Validation failed:', result.error);
      const errors: Record<string, string> = {};
      result.error.errors.forEach((err) => {
        if (err.path.length > 0) {
          errors[err.path[0] as string] = err.message;
        }
      });
      console.log('🔴 Validation errors:', errors);
      setValidationErrors(errors);
      setError('Please fix the errors below and try again.');
      return;
    }

    console.log('✅ Validation passed');

    try {
      // Remove confirmPassword before sending to API
      const { confirmPassword, ...signupData } = formData;
      console.log('🔵 Calling signup API...');
      await signup(signupData);

      console.log('✅ Signup successful!');
      // Success callback
      if (onSuccess) {
        onSuccess();
      }
    } catch (err: any) {
      console.error('🔴 Signup error:', err);
      setError(err.message || 'Signup failed. Please try again.');
    }
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form signup-form">
      <h2>Create Your Account</h2>

      {error && (
        <div className="alert alert-error" role="alert">
          {error}
        </div>
      )}

      {/* Email and Password */}
      <div className="form-section">
        <h3>Account Information</h3>

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
            placeholder="Min 8 chars, 1 uppercase, 1 lowercase, 1 number"
            className={validationErrors.password ? 'error' : ''}
          />
          {formData.password && (
            <div className="password-requirements">
              <div className={`requirement ${passwordRequirements.minLength ? 'met' : 'unmet'}`}>
                {passwordRequirements.minLength ? '✓' : '✗'} At least 8 characters
              </div>
              <div className={`requirement ${passwordRequirements.hasUppercase ? 'met' : 'unmet'}`}>
                {passwordRequirements.hasUppercase ? '✓' : '✗'} One uppercase letter
              </div>
              <div className={`requirement ${passwordRequirements.hasLowercase ? 'met' : 'unmet'}`}>
                {passwordRequirements.hasLowercase ? '✓' : '✗'} One lowercase letter
              </div>
              <div className={`requirement ${passwordRequirements.hasNumber ? 'met' : 'unmet'}`}>
                {passwordRequirements.hasNumber ? '✓' : '✗'} One number
              </div>
            </div>
          )}
          {validationErrors.password && (
            <span className="error-message">{validationErrors.password}</span>
          )}
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">
            Confirm Password <span className="required">*</span>
          </label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            required
            placeholder="Re-enter your password"
            className={validationErrors.confirmPassword ? 'error' : ''}
          />
          {validationErrors.confirmPassword && (
            <span className="error-message">{validationErrors.confirmPassword}</span>
          )}
        </div>
      </div>

      {/* Skill Levels */}
      <div className="form-section">
        <h3>Your Skill Levels</h3>
        <p className="section-description">
          Help us personalize content to your experience level (1 = Beginner, 5 = Expert)
        </p>

        {(['ai_level', 'ml_level', 'ros_level', 'python_level', 'linux_level'] as const).map(
          (skillKey) => (
            <div key={skillKey} className="form-group">
              <label htmlFor={skillKey}>
                {skillKey.replace('_level', '').toUpperCase()} Level{' '}
                <span className="required">*</span>
              </label>
              <select
                id={skillKey}
                name={skillKey}
                value={formData[skillKey]}
                onChange={handleChange}
                required
                className={validationErrors[skillKey] ? 'error' : ''}
              >
                {[1, 2, 3, 4, 5].map((level) => (
                  <option key={level} value={level}>
                    {level} - {skillLevelLabels[level]}
                    {skillLevelDescriptions[skillKey]?.[level]
                      ? ` (${skillLevelDescriptions[skillKey][level]})`
                      : ''}
                  </option>
                ))}
              </select>
              {validationErrors[skillKey] && (
                <span className="error-message">{validationErrors[skillKey]}</span>
              )}
            </div>
          )
        )}
      </div>

      {/* Hardware Access */}
      <div className="form-section">
        <h3>Hardware Access</h3>
        <p className="section-description">
          Select the hardware you have access to (optional - affects exercise recommendations)
        </p>

        {(['has_gpu', 'has_jetson', 'has_robot'] as const).map((hardwareKey) => (
          <div key={hardwareKey} className="form-group form-checkbox">
            <label>
              <input
                type="checkbox"
                name={hardwareKey}
                checked={formData[hardwareKey]}
                onChange={handleChange}
              />
              <span>{hardwareDescriptions[hardwareKey]}</span>
            </label>
          </div>
        ))}
      </div>

      {/* Submit Button */}
      <div className="form-actions">
        <button
          type="submit"
          className="button button--primary button--lg"
          disabled={isLoading}
        >
          {isLoading ? 'Creating Account...' : 'Sign Up'}
        </button>

        {onSwitchToSignin && (
          <p className="form-switch">
            Already have an account?{' '}
            <button
              type="button"
              onClick={onSwitchToSignin}
              className="link-button"
            >
              Sign In
            </button>
          </p>
        )}
      </div>
    </form>
  );
}
