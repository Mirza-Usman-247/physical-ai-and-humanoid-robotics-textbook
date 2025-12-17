/**
 * Personalization API client for chapter content adaptation.
 */

// API base URL - browser-safe (no process.env in browser)
const API_BASE_URL = typeof window !== 'undefined' && (window as any).API_BASE_URL
  ? (window as any).API_BASE_URL
  : 'http://localhost:8000';

export interface PersonalizeRequest {
  chapter_id: string;
  chapter_content: string;
  focus_areas?: string[];
}

export interface PersonalizeResponse {
  chapter_id: string;
  personalized_content: string;
  transformation_metadata: {
    skill_adjustments: {
      ai_level: number;
      ml_level: number;
      ros_level: number;
      python_level: number;
      linux_level: number;
    };
    hardware_recommendations: string[];
    model: string;
    focus_areas?: string[];
    usage?: {
      prompt_tokens: number;
      completion_tokens: number;
      total_tokens: number;
    };
  };
}

export interface AuthError {
  detail: string;
}

/**
 * Personalize chapter content based on user profile.
 *
 * @param request - Personalization request with chapter ID and content
 * @param accessToken - JWT access token
 * @returns Personalized chapter content and metadata
 * @throws Error if personalization fails
 */
export async function personalizeChapter(
  request: PersonalizeRequest,
  accessToken: string
): Promise<PersonalizeResponse> {
  const response = await fetch(`${API_BASE_URL}/api/personalize`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${accessToken}`,
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error: AuthError = await response.json();
    throw new Error(error.detail || 'Personalization failed');
  }

  return await response.json();
}
