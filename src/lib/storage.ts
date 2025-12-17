/**
 * IndexedDB storage for personalized and translated chapter content.
 *
 * Stores transformed content client-side per ADR-006 (session-only storage).
 */
import { openDB, DBSchema, IDBPDatabase } from 'idb';

// Database schema
interface TextbookDB extends DBSchema {
  'personalized-chapters': {
    key: string; // chapter_id
    value: {
      chapter_id: string;
      content: string;
      metadata: any;
      timestamp: number;
      user_email: string;
    };
  };
  'translated-chapters': {
    key: string; // chapter_id
    value: {
      chapter_id: string;
      content: string;
      metadata: any;
      timestamp: number;
      user_email: string;
      language: string;
    };
  };
}

const DB_NAME = 'textbook-personalization';
const DB_VERSION = 1;

let dbPromise: Promise<IDBPDatabase<TextbookDB>> | null = null;

/**
 * Initialize IndexedDB database.
 */
async function getDB(): Promise<IDBPDatabase<TextbookDB>> {
  if (!dbPromise) {
    dbPromise = openDB<TextbookDB>(DB_NAME, DB_VERSION, {
      upgrade(db) {
        // Create personalized chapters store
        if (!db.objectStoreNames.contains('personalized-chapters')) {
          const personalizedStore = db.createObjectStore('personalized-chapters', {
            keyPath: 'chapter_id',
          });
          personalizedStore.createIndex('user_email', 'user_email');
          personalizedStore.createIndex('timestamp', 'timestamp');
        }

        // Create translated chapters store
        if (!db.objectStoreNames.contains('translated-chapters')) {
          const translatedStore = db.createObjectStore('translated-chapters', {
            keyPath: 'chapter_id',
          });
          translatedStore.createIndex('user_email', 'user_email');
          translatedStore.createIndex('language', 'language');
          translatedStore.createIndex('timestamp', 'timestamp');
        }
      },
    });
  }
  return dbPromise;
}

/**
 * Save personalized chapter content.
 */
export async function savePersonalizedChapter(
  chapter_id: string,
  content: string,
  metadata: any,
  user_email: string
): Promise<void> {
  const db = await getDB();
  await db.put('personalized-chapters', {
    chapter_id,
    content,
    metadata,
    timestamp: Date.now(),
    user_email,
  });
}

/**
 * Get personalized chapter content.
 */
export async function getPersonalizedChapter(
  chapter_id: string
): Promise<{ content: string; metadata: any; timestamp: number } | null> {
  const db = await getDB();
  const result = await db.get('personalized-chapters', chapter_id);
  return result || null;
}

/**
 * Delete personalized chapter content.
 */
export async function deletePersonalizedChapter(chapter_id: string): Promise<void> {
  const db = await getDB();
  await db.delete('personalized-chapters', chapter_id);
}

/**
 * Get all personalized chapters for current user.
 */
export async function getAllPersonalizedChapters(
  user_email: string
): Promise<Array<{ chapter_id: string; timestamp: number }>> {
  const db = await getDB();
  const tx = db.transaction('personalized-chapters', 'readonly');
  const index = tx.store.index('user_email');
  const results = await index.getAll(user_email);

  return results.map((item) => ({
    chapter_id: item.chapter_id,
    timestamp: item.timestamp,
  }));
}

/**
 * Clear all personalized content (called on logout).
 */
export async function clearAllPersonalizedContent(): Promise<void> {
  const db = await getDB();
  await db.clear('personalized-chapters');
}

/**
 * Save translated chapter content.
 */
export async function saveTranslatedChapter(
  chapter_id: string,
  content: string,
  metadata: any,
  user_email: string,
  language: string
): Promise<void> {
  const db = await getDB();
  await db.put('translated-chapters', {
    chapter_id,
    content,
    metadata,
    timestamp: Date.now(),
    user_email,
    language,
  });
}

/**
 * Get translated chapter content.
 */
export async function getTranslatedChapter(
  chapter_id: string
): Promise<{ content: string; metadata: any; language: string; timestamp: number } | null> {
  const db = await getDB();
  const result = await db.get('translated-chapters', chapter_id);
  return result || null;
}

/**
 * Delete translated chapter content.
 */
export async function deleteTranslatedChapter(chapter_id: string): Promise<void> {
  const db = await getDB();
  await db.delete('translated-chapters', chapter_id);
}

/**
 * Clear all translated content (called on logout).
 */
export async function clearAllTranslatedContent(): Promise<void> {
  const db = await getDB();
  await db.clear('translated-chapters');
}

/**
 * Clear all stored content (personalized + translated).
 * Called on user logout.
 */
export async function clearAllStoredContent(): Promise<void> {
  await Promise.all([
    clearAllPersonalizedContent(),
    clearAllTranslatedContent(),
  ]);
}
