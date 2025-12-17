/**
 * AuthProvider: React Context Provider for authentication state.
 * Wraps the entire Docusaurus app to provide auth state to all components.
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import type { UserSession, UserProfile, AuthTokens } from '../../types/auth';
import * as authAPI from '../../api/auth';

/**
 * Authentication context value type.
 */
interface AuthContextValue {
  // Auth state
  user: UserProfile | null;
  tokens: AuthTokens | null;
  isAuthenticated: boolean;
  isLoading: boolean;

  // Auth actions
  signin: (email: string, password: string) => Promise<void>;
  signup: (signupData: authAPI.SignupRequest) => Promise<void>;
  logout: () => Promise<void>;
  refreshAccessToken: () => Promise<void>;
}

/**
 * Authentication context.
 */
const AuthContext = createContext<AuthContextValue | undefined>(undefined);

/**
 * AuthProvider component props.
 */
interface AuthProviderProps {
  children: ReactNode;
}

/**
 * AuthProvider component.
 *
 * Manages authentication state and provides auth methods to child components.
 * Stores JWT tokens in memory (React state) for security (not localStorage).
 * Automatically refreshes access token on mount if refresh token exists.
 *
 * Usage:
 * ```tsx
 * <AuthProvider>
 *   <App />
 * </AuthProvider>
 * ```
 */
export function AuthProvider({ children }: AuthProviderProps) {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [tokens, setTokens] = useState<AuthTokens | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Load tokens from sessionStorage on mount (optional: use memory-only for max security)
  useEffect(() => {
    const loadSession = async () => {
      try {
        // Only run in browser, not during SSR
        if (typeof window === 'undefined') {
          setIsLoading(false);
          return;
        }

        const storedTokens = sessionStorage.getItem('auth_tokens');
        if (storedTokens) {
          const parsedTokens: AuthTokens = JSON.parse(storedTokens);
          setTokens(parsedTokens);

          // Fetch user profile with access token
          const profile = await authAPI.getUserProfile(parsedTokens.access_token);
          setUser(profile);
        }
      } catch (error) {
        console.error('Failed to restore session:', error);
        if (typeof window !== 'undefined') {
          sessionStorage.removeItem('auth_tokens');
        }
      } finally {
        setIsLoading(false);
      }
    };

    loadSession();
  }, []);

  // Save tokens to sessionStorage when they change
  useEffect(() => {
    // Only run in browser, not during SSR
    if (typeof window === 'undefined') {
      return;
    }

    if (tokens) {
      sessionStorage.setItem('auth_tokens', JSON.stringify(tokens));
    } else {
      sessionStorage.removeItem('auth_tokens');
    }
  }, [tokens]);

  /**
   * Sign in with email and password.
   */
  const signin = async (email: string, password: string): Promise<void> => {
    try {
      const session: UserSession = await authAPI.signin({ email, password });
      setUser(session.user);
      setTokens(session.tokens);
    } catch (error) {
      console.error('Signin failed:', error);
      throw error;
    }
  };

  /**
   * Sign up with email, password, and skill profile.
   */
  const signup = async (signupData: authAPI.SignupRequest): Promise<void> => {
    try {
      const session: UserSession = await authAPI.signup(signupData);
      setUser(session.user);
      setTokens(session.tokens);
    } catch (error) {
      console.error('Signup failed:', error);
      throw error;
    }
  };

  /**
   * Logout and clear session.
   */
  const logout = async (): Promise<void> => {
    try {
      await authAPI.logout();
    } finally {
      setUser(null);
      setTokens(null);
      if (typeof window !== 'undefined') {
        sessionStorage.removeItem('auth_tokens');
      }
    }
  };

  /**
   * Refresh access token using refresh token.
   */
  const refreshAccessToken = async (): Promise<void> => {
    if (!tokens || !tokens.refresh_token) {
      throw new Error('No refresh token available');
    }

    try {
      const newTokens = await authAPI.refreshTokens(tokens.refresh_token);
      setTokens(newTokens);
    } catch (error) {
      console.error('Token refresh failed:', error);
      // Clear session on refresh failure
      await logout();
      throw error;
    }
  };

  const value: AuthContextValue = {
    user,
    tokens,
    isAuthenticated: !!user && !!tokens,
    isLoading,
    signin,
    signup,
    logout,
    refreshAccessToken,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * Hook to access authentication context.
 *
 * Must be used within AuthProvider.
 *
 * @throws Error if used outside AuthProvider
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const { user, isAuthenticated, signin, logout } = useAuth();
 *
 *   if (!isAuthenticated) {
 *     return <button onClick={() => signin('user@example.com', 'password')}>Sign In</button>;
 *   }
 *
 *   return <div>Welcome, {user.email}!</div>;
 * }
 * ```
 */
export function useAuthContext(): AuthContextValue {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within AuthProvider');
  }
  return context;
}
