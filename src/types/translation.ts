/**
 * Translation TypeScript Types
 *
 * Type definitions for chapter translation to Urdu
 */

/**
 * Translation request payload
 */
export interface TranslateRequest {
  /** Unique identifier for the chapter */
  chapter_id: string;
  /** Original chapter markdown content */
  chapter_content: string;
  /** Enable Focus Mode (technical faithfulness, no extra commentary) */
  focus_mode?: boolean;
  /** Optional list of topics to emphasize in translation */
  focus_areas?: string[];
}

/**
 * Translation metadata
 */
export interface TranslationMetadata {
  /** Source language code */
  source_language: string;
  /** Target language code */
  target_language: string;
  /** Target language name */
  target_language_name: string;
  /** Translation mode used */
  mode: string;
  /** LLM model used for translation */
  model_used: string;
  /** Number of technical terms preserved */
  preserved_terms_count: number;
  /** Sample of preserved technical terms */
  preserved_terms_sample: string[];
  /** Original content length */
  original_length: number;
  /** Translated content length */
  translated_length: number;
}

/**
 * Translation response from API
 */
export interface TranslateResponse {
  /** Chapter identifier */
  chapter_id: string;
  /** Translated markdown content in Urdu */
  translated_content: string;
  /** Translation metadata */
  transformation_metadata: TranslationMetadata;
}

/**
 * Supported language configuration
 */
export interface SupportedLanguage {
  /** Language code (ISO 639-1) */
  code: string;
  /** English name */
  name: string;
  /** Native name */
  native_name: string;
  /** Text direction (ltr or rtl) */
  direction: 'ltr' | 'rtl';
  /** Availability status */
  status: 'active' | 'coming_soon';
}

/**
 * Translation mode configuration
 */
export interface TranslationMode {
  /** Mode code */
  code: string;
  /** Mode name */
  name: string;
  /** Mode description */
  description: string;
}

/**
 * Supported languages response
 */
export interface SupportedLanguagesResponse {
  /** List of supported languages */
  supported_languages: SupportedLanguage[];
  /** Default translation mode */
  default_mode: string;
  /** Available translation modes */
  modes: TranslationMode[];
}

/**
 * Stored translation in IndexedDB
 */
export interface StoredTranslation {
  /** Chapter identifier */
  chapter_id: string;
  /** Translated content */
  content: string;
  /** Translation metadata */
  metadata: TranslationMetadata;
  /** Timestamp when translation was stored */
  timestamp: number;
  /** User email */
  user_email: string;
  /** Target language code */
  language: string;
}
