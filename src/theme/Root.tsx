import React, { useState, useEffect } from 'react';
import { AuthProvider } from '../components/Auth';
import { ChatWidget } from '../components/RAGChatbot';

// Default implementation, that you can customize
// Wrapped with AuthProvider to provide auth context to entire app
export default function Root({children}) {
  const [isBrowser, setIsBrowser] = useState(false);

  // Only render ChatWidget in browser, not during SSR
  useEffect(() => {
    setIsBrowser(true);
  }, []);

  return (
    <AuthProvider>
      {children}
      {isBrowser && (
        <ChatWidget
          apiBaseUrl={typeof window !== 'undefined' && (window as any).API_BASE_URL
            ? (window as any).API_BASE_URL
            : 'http://localhost:8000'
          }
        />
      )}
    </AuthProvider>
  );
}
