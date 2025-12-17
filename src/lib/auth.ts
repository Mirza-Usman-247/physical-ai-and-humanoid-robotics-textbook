/**
 * Better Auth configuration for Physical AI & Humanoid Robotics Textbook.
 * Configures email/password authentication with JWT tokens.
 */

import { createAuth } from 'better-auth';

/**
 * Better Auth client instance.
 *
 * Configuration:
 * - Email/password authentication enabled
 * - JWT tokens for session management (handled by backend)
 * - Base URL from environment variable (defaults to http://localhost:8000)
 *
 * Usage:
 * ```typescript
 * import { authClient } from '@/lib/auth';
 *
 * // Sign up
 * const result = await authClient.signUp({
 *   email: 'user@example.com',
 *   password: 'SecurePass123',
 *   ai_level: 3,
 *   ml_level: 2,
 *   ros_level: 1,
 *   python_level: 4,
 *   linux_level: 3,
 *   has_gpu: true,
 *   has_jetson: false,
 *   has_robot: false
 * });
 *
 * // Sign in
 * const session = await authClient.signIn({
 *   email: 'user@example.com',
 *   password: 'SecurePass123'
 * });
 * ```
 */
export const authClient = createAuth({
  baseURL: typeof window !== 'undefined' && (window as any).API_BASE_URL
    ? (window as any).API_BASE_URL
    : 'http://localhost:8000',

  // Enable email/password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },

  // Session configuration (JWT tokens)
  session: {
    cookieCache: {
      enabled: true,
      maxAge: 60 * 15, // 15 minutes (matches JWT access token expiry)
    },
  },
});

/**
 * Auth client type for TypeScript inference.
 */
export type AuthClient = typeof authClient;
