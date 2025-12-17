/**
 * useAuth hook: Convenience hook for accessing authentication context.
 *
 * Re-exports useAuthContext from AuthProvider for cleaner imports.
 */

export { useAuthContext as useAuth } from '../components/Auth';

/**
 * Usage example:
 * ```tsx
 * import { useAuth } from '@/hooks/useAuth';
 *
 * function MyComponent() {
 *   const { user, isAuthenticated, signin, signup, logout } = useAuth();
 *
 *   if (isAuthenticated) {
 *     return (
 *       <div>
 *         <p>Welcome, {user.email}!</p>
 *         <p>AI Level: {user.ai_level}/5</p>
 *         <button onClick={logout}>Logout</button>
 *       </div>
 *     );
 *   }
 *
 *   return (
 *     <button onClick={() => signin('user@example.com', 'password')}>
 *       Sign In
 *     </button>
 *   );
 * }
 * ```
 */
