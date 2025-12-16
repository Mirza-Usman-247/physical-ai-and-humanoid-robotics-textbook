import React from 'react';
import ReadingProgress from '@site/src/components/ReadingProgress';
import { ChatWidget } from '@site/src/components/RAGChatbot';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      <ReadingProgress />
      {children}
      <ChatWidget apiBaseUrl="http://localhost:8000" />
    </>
  );
}
