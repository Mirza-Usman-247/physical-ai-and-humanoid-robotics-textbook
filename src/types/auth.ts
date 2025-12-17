/**
 * TypeScript types for authentication and user profiles.
 * Matches backend Pydantic models for type safety.
 */

/**
 * User profile data including skill levels and hardware access.
 */
export interface UserProfile {
  user_id: string;
  email: string;
  ai_level: number; // 1-5
  ml_level: number; // 1-5
  ros_level: number; // 1-5
  python_level: number; // 1-5
  linux_level: number; // 1-5
  has_gpu: boolean;
  has_jetson: boolean;
  has_robot: boolean;
  cloud_only: boolean; // Computed
  created_at: string; // ISO 8601 timestamp
}

/**
 * JWT tokens response from authentication endpoints.
 */
export interface AuthTokens {
  access_token: string;
  refresh_token: string;
  token_type: 'bearer';
}

/**
 * Complete user session including profile and tokens.
 */
export interface UserSession {
  user: UserProfile;
  tokens: AuthTokens;
}

/**
 * Signup request payload.
 */
export interface SignupRequest {
  email: string;
  password: string;
  ai_level: number; // 1-5
  ml_level: number; // 1-5
  ros_level: number; // 1-5
  python_level: number; // 1-5
  linux_level: number; // 1-5
  has_gpu?: boolean; // Optional, defaults to false
  has_jetson?: boolean; // Optional, defaults to false
  has_robot?: boolean; // Optional, defaults to false
}

/**
 * Signin request payload.
 */
export interface SigninRequest {
  email: string;
  password: string;
}

/**
 * Refresh token request payload.
 */
export interface RefreshTokenRequest {
  refresh_token: string;
}

/**
 * Personalize chapter request payload.
 */
export interface PersonalizeRequest {
  chapter_id: string;
  chapter_content: string;
}

/**
 * Translate chapter request payload.
 */
export interface TranslateRequest {
  chapter_id: string;
  chapter_content: string;
  focus_mode?: boolean; // Optional, defaults to true
}

/**
 * Personalization response.
 */
export interface PersonalizeResponse {
  chapter_id: string;
  personalized_content: string;
  transformation_metadata: {
    llm_model: string;
    profile_snapshot: Partial<UserProfile>;
    request_timestamp: string;
    response_timestamp: string;
    token_usage: {
      input_tokens: number;
      output_tokens: number;
    };
  };
}

/**
 * Translation response.
 */
export interface TranslateResponse {
  chapter_id: string;
  translated_content: string;
  transformation_metadata: {
    llm_model: string;
    focus_mode: boolean;
    request_timestamp: string;
    response_timestamp: string;
    token_usage: {
      input_tokens: number;
      output_tokens: number;
    };
  };
}

/**
 * Authentication error response.
 */
export interface AuthError {
  error: string;
  message: string;
  details?: Record<string, any>;
  timestamp: string;
}
