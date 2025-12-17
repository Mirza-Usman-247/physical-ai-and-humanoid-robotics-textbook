/**
 * Translation API Client
 *
 * Handles communication with the translation backend endpoint
 */

import type {
  TranslateRequest,
  TranslateResponse,
  SupportedLanguagesResponse,
} from '../types/translation';

// API base URL - browser-safe (no process.env in browser)
const API_BASE_URL = typeof window !== 'undefined' && (window as any).API_BASE_URL
  ? (window as any).API_BASE_URL
  : 'http://localhost:8000';

/**
 * Translate chapter content to Urdu
 *
 * @param request Translation request with chapter content
 * @param accessToken JWT access token for authentication
 * @returns Promise resolving to translation response
 * @throws Error if translation fails or user is not authenticated
 */
export async function translateChapter(
  request: TranslateRequest,
  accessToken: string
): Promise<TranslateResponse> {
  if (!accessToken) {
    throw new Error('Authentication required to translate chapters');
  }

  const response = await fetch(`${API_BASE_URL}/api/translate`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${accessToken}`,
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Translation failed' }));
    throw new Error(error.detail || `Translation failed: ${response.statusText}`);
  }

  const data: TranslateResponse = await response.json();
  return data;
}

/**
 * Get list of supported translation languages
 *
 * @returns Promise resolving to supported languages configuration
 * @throws Error if request fails
 */
export async function getSupportedLanguages(): Promise<SupportedLanguagesResponse> {
  const response = await fetch(`${API_BASE_URL}/api/supported-languages`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    throw new Error(`Failed to fetch supported languages: ${response.statusText}`);
  }

  const data: SupportedLanguagesResponse = await response.json();
  return data;
}

/**
 * Translate with default Focus Mode settings
 *
 * Convenience function for translating with Focus Mode enabled (default)
 *
 * @param chapterId Unique identifier for the chapter
 * @param chapterContent Original chapter markdown content
 * @param accessToken JWT access token
 * @param focusAreas Optional list of topics to emphasize in translation
 * @returns Promise resolving to translation response
 */
export async function translateToUrdu(
  chapterId: string,
  chapterContent: string,
  accessToken: string,
  focusAreas?: string[]
): Promise<TranslateResponse> {
  return translateChapter(
    {
      chapter_id: chapterId,
      chapter_content: chapterContent,
      focus_mode: true, // Enable Focus Mode by default
      focus_areas: focusAreas,
    },
    accessToken
  );
}
