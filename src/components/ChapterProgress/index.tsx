import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

interface ProgressData {
  [path: string]: number; // path -> percentage read
}

export default function ChapterProgress(): JSX.Element {
  const location = useLocation();
  const [overallProgress, setOverallProgress] = useState(0);
  const [chaptersCompleted, setChaptersCompleted] = useState(0);

  useEffect(() => {
    const updateChapterProgress = () => {
      // Get current scroll percentage
      const scrollTop = window.scrollY;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const scrollPercent = Math.min(100, Math.max(0, (scrollTop / docHeight) * 100));

      // Get saved progress from localStorage
      const savedProgress: ProgressData = JSON.parse(
        localStorage.getItem('chapterProgress') || '{}'
      );

      // Update progress for current page
      const currentPath = location.pathname;
      if (currentPath.includes('/docs/')) {
        savedProgress[currentPath] = Math.max(
          savedProgress[currentPath] || 0,
          scrollPercent
        );

        // Mark as completed if > 90% read
        if (scrollPercent > 90) {
          savedProgress[currentPath] = 100;
        }

        localStorage.setItem('chapterProgress', JSON.stringify(savedProgress));
      }

      // Calculate overall statistics
      const paths = Object.keys(savedProgress);
      const totalChapters = 21; // Total chapters in textbook
      const completed = paths.filter(p => savedProgress[p] >= 90).length;
      const totalProgress = paths.reduce((sum, p) => sum + savedProgress[p], 0);
      const avgProgress = paths.length > 0 ? totalProgress / totalChapters : 0;

      setChaptersCompleted(completed);
      setOverallProgress(Math.round(avgProgress));
    };

    // Update on scroll
    const handleScroll = () => {
      updateChapterProgress();
    };

    window.addEventListener('scroll', handleScroll);
    updateChapterProgress(); // Initial update

    return () => window.removeEventListener('scroll', handleScroll);
  }, [location.pathname]);

  // Only show on chapter pages
  if (!location.pathname.includes('/docs/module-') &&
      !location.pathname.includes('/docs/capstone')) {
    return null;
  }

  return (
    <div className={styles.chapterProgressBadge}>
      <div className={styles.badgeContent}>
        <span className={styles.icon}>📚</span>
        <div className={styles.stats}>
          <div className={styles.completedCount}>
            {chaptersCompleted}/21 Chapters
          </div>
          <div className={styles.overallProgress}>
            {overallProgress}% Complete
          </div>
        </div>
      </div>
    </div>
  );
}
