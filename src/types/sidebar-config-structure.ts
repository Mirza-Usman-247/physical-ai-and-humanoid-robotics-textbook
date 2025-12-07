/**
 * Sidebar Configuration Structure
 *
 * TypeScript interfaces for Docusaurus 3.x sidebar navigation.
 * Supports 3-level nested structure: Modules > Chapters > Sections
 *
 * Generated from: sidebar-config-structure.ts
 * Date: 2025-12-06
 * Spec: Physical AI & Humanoid Robotics Textbook
 */

/**
 * Top-level sidebar configuration
 * Maps sidebar IDs to their item arrays
 */
export interface SidebarConfig {
  tutorialSidebar: SidebarItem[];
  [key: string]: SidebarItem[];
}

/**
 * Base sidebar item (discriminated union)
 * Can be a category (module), doc (chapter), or link
 */
export type SidebarItem = SidebarCategory | SidebarDoc | SidebarLink;

/**
 * Sidebar category (module level)
 * Contains nested chapters as children
 */
export interface SidebarCategory {
  type: 'category';
  label: string;
  link?: {
    type: 'generated-index' | 'doc';
    slug?: string;
    id?: string;
    title?: string;
    description?: string;
    keywords?: string[];
  };
  items: SidebarItem[];
  collapsible?: boolean;
  collapsed?: boolean;
  className?: string;
  customProps?: Record<string, unknown>;
}

/**
 * Sidebar doc (chapter level)
 * Direct link to MDX document
 */
export interface SidebarDoc {
  type: 'doc';
  id: string;
  label?: string;
  className?: string;
  customProps?: {
    module?: ModuleId;
    weekMapping?: string;
    difficultyLevel?: DifficultyLevel;
    estimatedTime?: number;
  };
}

/**
 * Sidebar link (external or internal)
 * Used for special pages (glossary, hardware lab)
 */
export interface SidebarLink {
  type: 'link';
  label: string;
  href: string;
  className?: string;
  customProps?: Record<string, unknown>;
}

/**
 * Module identifier enum
 */
export type ModuleId =
  | 'module-0'
  | 'module-1'
  | 'module-2'
  | 'module-3'
  | 'module-4'
  | 'capstone';

/**
 * Difficulty level enum
 */
export type DifficultyLevel = 'beginner' | 'intermediate' | 'advanced';

/**
 * Assessment type enum
 */
export type AssessmentType = 'conceptual' | 'computational' | 'implementation';

/**
 * Example sidebar configuration for Physical AI Textbook
 */
export const exampleSidebarConfig: SidebarConfig = {
  tutorialSidebar: [
    {
      type: 'link',
      label: 'Building Your Physical AI Lab',
      href: '/hardware-lab',
      className: 'sidebar-hardware-lab'
    },
    {
      type: 'category',
      label: 'Module 0: Physical AI Foundations',
      link: {
        type: 'generated-index',
        title: 'Module 0: Physical AI Foundations',
        description: 'Introduction to embodied intelligence, sensorimotor systems, and physical AI principles.',
        keywords: ['physical ai', 'embodied intelligence', 'foundations']
      },
      items: [
        {
          type: 'doc',
          id: 'module-0-foundations/chapter-1-intro-physical-ai/index',
          label: 'Introduction to Physical AI',
          customProps: {
            module: 'module-0',
            weekMapping: 'Week 1',
            difficultyLevel: 'beginner',
            estimatedTime: 3
          }
        },
        {
          type: 'doc',
          id: 'module-0-foundations/chapter-2-embodied-intelligence/index',
          label: 'Embodied Intelligence',
          customProps: {
            module: 'module-0',
            weekMapping: 'Week 1',
            difficultyLevel: 'beginner',
            estimatedTime: 4
          }
        },
        {
          type: 'doc',
          id: 'module-0-foundations/chapter-3-sensorimotor-foundations/index',
          label: 'Sensorimotor Foundations',
          customProps: {
            module: 'module-0',
            weekMapping: 'Week 2',
            difficultyLevel: 'intermediate',
            estimatedTime: 4
          }
        }
      ],
      collapsible: true,
      collapsed: false,
      className: 'sidebar-module-0'
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 for Physical AI',
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2 for Physical AI',
        description: 'Master ROS 2 Humble architecture, communication patterns, and robot control.',
        keywords: ['ros2', 'middleware', 'robot operating system']
      },
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/chapter-1-ros2-architecture/index',
          label: 'ROS 2 Architecture Overview',
          customProps: {
            module: 'module-1',
            weekMapping: 'Week 3',
            difficultyLevel: 'intermediate',
            estimatedTime: 3
          }
        }
        // ... additional chapters
      ],
      collapsible: true,
      collapsed: true,
      className: 'sidebar-module-1'
    },
    // ... additional modules
    {
      type: 'link',
      label: 'Glossary',
      href: '/glossary',
      className: 'sidebar-glossary'
    },
    {
      type: 'link',
      label: 'Mathematical Notation',
      href: '/notation',
      className: 'sidebar-notation'
    }
  ]
};

/**
 * Utility type guards for runtime type checking
 */
export function isSidebarCategory(item: SidebarItem): item is SidebarCategory {
  return item.type === 'category';
}

export function isSidebarDoc(item: SidebarItem): item is SidebarDoc {
  return item.type === 'doc';
}

export function isSidebarLink(item: SidebarItem): item is SidebarLink {
  return item.type === 'link';
}

/**
 * Helper function to extract all doc IDs from sidebar config
 * Useful for validation and build checks
 */
export function extractDocIds(sidebar: SidebarItem[]): string[] {
  const docIds: string[] = [];

  function traverse(items: SidebarItem[]) {
    for (const item of items) {
      if (isSidebarDoc(item)) {
        docIds.push(item.id);
      } else if (isSidebarCategory(item)) {
        traverse(item.items);
      }
    }
  }

  traverse(sidebar);
  return docIds;
}

/**
 * Helper function to validate sidebar structure
 * Ensures all required modules and chapters are present
 */
export function validateSidebarStructure(config: SidebarConfig): {
  valid: boolean;
  errors: string[];
} {
  const errors: string[] = [];
  const requiredModules: ModuleId[] = [
    'module-0',
    'module-1',
    'module-2',
    'module-3',
    'module-4',
    'capstone'
  ];

  const expectedChapterCounts: Record<ModuleId, number> = {
    'module-0': 3,
    'module-1': 5,
    'module-2': 4,
    'module-3': 5,
    'module-4': 3,
    'capstone': 1
  };

  const foundModules = new Set<ModuleId>();
  const moduleChapterCounts: Record<string, number> = {};

  for (const item of config.tutorialSidebar) {
    if (isSidebarCategory(item)) {
      const moduleMatch = item.className?.match(/sidebar-module-(\d+|capstone)/);
      if (moduleMatch) {
        const moduleId = moduleMatch[1] === 'capstone'
          ? 'capstone'
          : `module-${moduleMatch[1]}` as ModuleId;
        foundModules.add(moduleId);

        const chapterCount = item.items.filter(isSidebarDoc).length;
        moduleChapterCounts[moduleId] = chapterCount;

        if (chapterCount !== expectedChapterCounts[moduleId]) {
          errors.push(
            `${moduleId} has ${chapterCount} chapters, expected ${expectedChapterCounts[moduleId]}`
          );
        }
      }
    }
  }

  for (const requiredModule of requiredModules) {
    if (!foundModules.has(requiredModule)) {
      errors.push(`Missing required module: ${requiredModule}`);
    }
  }

  return {
    valid: errors.length === 0,
    errors
  };
}
