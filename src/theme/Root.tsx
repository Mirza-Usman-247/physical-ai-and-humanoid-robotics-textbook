import React from 'react';
import ReadingProgress from '@site/src/components/ReadingProgress';

// Default implementation, that you can customize
export default function Root({children}) {
  return (
    <>
      <ReadingProgress />
      {children}
    </>
  );
}
