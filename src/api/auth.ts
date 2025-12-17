/**
 * Authentication API client.
 * Handles signup, signin, refresh, and logout with the backend.
 */

import type {
  SignupRequest,
  SigninRequest,
  UserSession,
  AuthTokens,
  AuthError,
} from '../types/auth';

// API base URL - browser-safe (no process.env in browser)
const API_BASE_URL = typeof window !== 'undefined' && (window as any).API_BASE_URL
  ? (window as any).API_BASE_URL
  : 'http://localhost:8000';

/**
 * Signup a new user with email, password, and skill profile.
 *
 * @param request - Signup request payload
 * @returns User session with profile and JWT tokens
 * @throws AuthError if signup fails
 */
export async function signup(request: SignupRequest): Promise<UserSession> {
  const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.message || 'Signup failed');
  }

  const data: UserSession = await response.json();
  return data;
}

/**
 * Sign in an existing user with email and password.
 *
 * @param request - Signin request payload
 * @returns User session with profile and JWT tokens
 * @throws AuthError if signin fails
 */
export async function signin(request: SigninRequest): Promise<UserSession> {
  const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.message || 'Signin failed');
  }

  const data: UserSession = await response.json();
  return data;
}

/**
 * Refresh access token using refresh token.
 *
 * @param refreshToken - Valid refresh token
 * @returns New JWT tokens (access + refresh)
 * @throws AuthError if refresh fails
 */
export async function refreshTokens(refreshToken: string): Promise<AuthTokens> {
  const response = await fetch(`${API_BASE_URL}/api/auth/refresh`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ refresh_token: refreshToken }),
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.message || 'Token refresh failed');
  }

  const data: AuthTokens = await response.json();
  return data;
}

/**
 * Logout the current user (client-side token deletion).
 *
 * Note: Backend is stateless, so logout is handled client-side by clearing tokens.
 */
export async function logout(): Promise<void> {
  // Clear tokens from storage (handled by AuthProvider)
  // Optionally call backend /api/auth/logout for audit logging
  try {
    await fetch(`${API_BASE_URL}/api/auth/logout`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
    });
  } catch (error) {
    // Ignore logout endpoint errors (client-side logout is sufficient)
    console.warn('Logout endpoint call failed (ignoring):', error);
  }
}

/**
 * Get current user profile using access token.
 *
 * @param accessToken - Valid access token
 * @returns User profile data
 * @throws AuthError if profile fetch fails
 */
export async function getUserProfile(accessToken: string) {
  const response = await fetch(`${API_BASE_URL}/api/profile`, {
    method: 'GET',
    headers: {
      'Authorization': `Bearer ${accessToken}`,
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.message || 'Failed to fetch user profile');
  }

  const data = await response.json();
  return data;
}
