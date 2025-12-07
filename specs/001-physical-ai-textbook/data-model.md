# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2025-12-06
**Status**: Phase 1 Design Complete

## Overview

This document defines the canonical data model for the Physical AI & Humanoid Robotics textbook. All entities, relationships, and validation rules are specified here to ensure consistency across the Docusaurus site, code examples, and CI/CD validation.

## Core Entities

### 1. Module

**Description**: Top-level organizational unit representing a major topic area in the 13-week course.

**Attributes**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `id` | string | ✅ | Unique module identifier | Pattern: `module-[0-4]\|capstone` |
| `title` | string | ✅ | Full module name | 5-100 characters |
| `weekRange` | string | ✅ | Week span in course | Pattern: `Week [1-13]` or `Weeks [1-13]-[1-13]` |
| `description` | string | ✅ | Module overview (2-3 sentences) | 50-500 characters |
| `learningOutcomes` | string[] | ✅ | High-level module objectives | 3-7 outcomes, active voice |
| `chapters` | Chapter[] | ✅ | Ordered list of chapters | 3-5 chapters (1 for capstone) |
| `capstoneIntegration` | string | ❌ | How module contributes to capstone | 50-300 characters |
| `estimatedTime` | number | ✅ | Total study hours for module | Range: 6-40 hours |

**Example**:
```json
{
  "id": "module-1",
  "title": "ROS 2: The Robotic Nervous System",
  "weekRange": "Weeks 3-5",
  "description": "Master ROS 2 Humble architecture, pub/sub patterns, services/actions, and tf transforms for robot control.",
  "learningOutcomes": [
    "Explain ROS 2 architecture and DDS middleware",
    "Implement pub/sub communication patterns",
    "Create services and actions for task coordination",
    "Apply tf transforms for spatial reasoning",
    "Control simulated and physical robots via ROS 2"
  ],
  "chapters": ["chapter-1", "chapter-2", "chapter-3", "chapter-4", "chapter-5"],
  "capstoneIntegration": "ROS 2 provides the communication backbone for voice-to-action pipeline integration.",
  "estimatedTime": 18
}
```

**Relationships**:
- 1:N with Chapter (each module contains 3-5 chapters, 1 for capstone)
- 1:N with Assessment (each module has 3+ assessments)

---

### 2. Chapter

**Description**: Individual learning unit within a module, following strict 12-section template.

**Attributes**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `id` | string | ✅ | Unique chapter identifier | Pattern: `chapter-[1-5]` |
| `module` | string | ✅ | Parent module ID | Must match existing module ID |
| `title` | string | ✅ | Chapter title | 5-100 characters |
| `description` | string | ✅ | Brief chapter summary for SEO | 50-300 characters |
| `weekMapping` | string | ✅ | Specific week assignment | Pattern: `Week [1-13] – [Day(s)]` |
| `sidebarPosition` | number | ✅ | Order in sidebar | Positive integer |
| `keywords` | string[] | ✅ | SEO keywords | 5-15 keywords |
| `prerequisites` | Prerequisite[] | ✅ | Required prior knowledge | Internal chapters + external topics |
| `learningObjectives` | string[] | ✅ | Chapter-specific objectives | 3-7 objectives, active voice |
| `assessmentType` | enum | ✅ | Exercise category | `conceptual \| computational \| implementation` |
| `difficultyLevel` | enum | ✅ | Reader skill level | `beginner \| intermediate \| advanced` |
| `capstoneComponent` | string | ❌ | Capstone integration point | 50-200 characters |
| `codeDependencies` | string[] | ✅ | Version-pinned packages | Pattern: `package==version` |
| `estimatedTime` | number | ✅ | Study hours for chapter | Range: 2-8 hours |
| `content` | ChapterContent | ✅ | 12-section structure | See ChapterContent entity |

**Prerequisite Sub-Entity**:
```typescript
interface Prerequisite {
  type: 'internal' | 'external';
  title: string;
  link?: string;  // Required for internal, optional for external
  description?: string;  // Optional context
}
```

**ChapterContent Sub-Entity** (12 Sections):
```typescript
interface ChapterContent {
  frontmatter: object;  // Docusaurus metadata
  learningObjectives: string;  // Markdown content
  prerequisites: string;  // Markdown content
  weeklyMapping: string;  // Markdown content
  motivatingScenario: string;  // Markdown content
  coreTheory: string;  // Markdown content (equations, derivations)
  workedExample: string;  // Markdown content
  handsOnCode: string;  // Markdown content (embedded code snippets)
  applicationToHumanoids: string;  // Markdown content
  commonPitfalls: string;  // Markdown content
  exercises: Exercise[];  // Structured exercises
  references: Reference[];  // Citations
}
```

**Exercise Sub-Entity**:
```typescript
interface Exercise {
  type: 'conceptual' | 'computational' | 'implementation';
  title: string;
  description: string;
  estimatedTime: number;  // minutes
  difficulty: 'beginner' | 'intermediate' | 'advanced';
}
```

**Reference Sub-Entity**:
```typescript
interface Reference {
  type: 'paper' | 'book' | 'documentation' | 'website';
  title: string;
  authors?: string[];
  year?: number;
  url?: string;
  doi?: string;
  citation: string;  // Full formatted citation
}
```

**Example**:
```json
{
  "id": "chapter-2",
  "module": "module-1",
  "title": "Publisher-Subscriber Communication in ROS 2",
  "description": "Learn ROS 2 pub/sub architecture, create publishers and subscribers, and understand DDS Quality of Service policies.",
  "weekMapping": "Week 3 – Tuesday & Thursday",
  "sidebarPosition": 2,
  "keywords": ["ros2", "publisher", "subscriber", "dds", "qos", "middleware"],
  "prerequisites": [
    {
      "type": "internal",
      "title": "ROS 2 Architecture Overview",
      "link": "/docs/module-1/chapter-1"
    },
    {
      "type": "external",
      "title": "Python basics",
      "description": "Functions, classes, decorators"
    }
  ],
  "learningObjectives": [
    "Explain ROS 2 pub/sub communication pattern",
    "Create custom message types",
    "Implement publishers and subscribers in Python",
    "Configure QoS policies for reliable communication"
  ],
  "assessmentType": "implementation",
  "difficultyLevel": "intermediate",
  "capstoneComponent": "Sensor data streaming for perception pipeline",
  "codeDependencies": ["rclpy==3.3.11", "std_msgs==4.2.3"],
  "estimatedTime": 4
}
```

**Relationships**:
- N:1 with Module (each chapter belongs to one module)
- N:M with GlossaryEntry (chapters reference multiple terms)
- 1:N with CodeExample (chapters have embedded + /examples/ code)

---

### 3. GlossaryEntry

**Description**: Standardized definition of technical term used across the textbook.

**Attributes**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `term` | string | ✅ | Canonical term | 2-50 characters, unique |
| `definition` | string | ✅ | Clear, concise definition | 20-500 characters |
| `relatedTerms` | string[] | ❌ | Associated concepts | References to other glossary terms |
| `chapters` | string[] | ✅ | Chapters using this term | Chapter IDs |
| `aliases` | string[] | ❌ | Alternative names | Synonyms, acronyms |
| `category` | string | ❌ | Term classification | e.g., "robotics", "ai", "mathematics" |

**Example**:
```json
{
  "term": "Inverse Kinematics",
  "definition": "The mathematical process of calculating joint angles required to position a robot's end-effector at a desired location and orientation in space.",
  "relatedTerms": ["Forward Kinematics", "Jacobian Matrix", "End-Effector"],
  "chapters": ["module-0/chapter-3", "module-1/chapter-5", "capstone/chapter-1"],
  "aliases": ["IK"],
  "category": "robotics"
}
```

**Relationships**:
- N:M with Chapter (terms used across multiple chapters, chapters reference multiple terms)

**FlexSearch Index**:
- Indexed fields: `term`, `definition`, `aliases`
- Searchable with typo tolerance
- Index rebuilt automatically during Docusaurus build

---

### 4. HardwareSetup

**Description**: Hardware configuration option for readers to run code examples.

**Attributes**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `id` | string | ✅ | Setup identifier | Pattern: `setup-[type]` |
| `name` | string | ✅ | Configuration name | 5-100 characters |
| `type` | enum | ✅ | Setup category | `workstation \| embedded \| cloud \| robot` |
| `requirements` | Requirement[] | ✅ | Component specifications | Hardware/software requirements |
| `cost` | Cost | ✅ | Budget information | USD estimates |
| `steps` | string[] | ✅ | Setup instructions | Ordered installation steps |
| `latencyWarning` | string | ❌ | Performance considerations | For cloud setups |

**Requirement Sub-Entity**:
```typescript
interface Requirement {
  component: string;  // e.g., "GPU", "RAM", "Storage"
  specification: string;  // e.g., "NVIDIA RTX 3060 or better"
  mandatory: boolean;
}
```

**Cost Sub-Entity**:
```typescript
interface Cost {
  oneTime: number;  // USD
  monthly?: number;  // USD (for cloud)
  notes: string;
}
```

**Example**:
```json
{
  "id": "setup-jetson-economy",
  "name": "Economy Jetson Student Kit",
  "type": "embedded",
  "requirements": [
    {
      "component": "Jetson Orin Nano Developer Kit",
      "specification": "8GB RAM variant",
      "mandatory": true
    },
    {
      "component": "microSD Card",
      "specification": "128GB UHS-I",
      "mandatory": true
    },
    {
      "component": "Power Supply",
      "specification": "USB-C PD 15W",
      "mandatory": true
    }
  ],
  "cost": {
    "oneTime": 699,
    "notes": "Includes Jetson, SD card, power supply. Does not include robot platform."
  },
  "steps": [
    "Flash JetPack 5.x to microSD card using Etcher",
    "Insert SD card and connect power",
    "Complete initial Ubuntu setup",
    "Install ROS 2 Humble: sudo apt install ros-humble-desktop",
    "Configure workspace: mkdir -p ~/ros2_ws/src && cd ~/ros2_ws && colcon build"
  ]
}
```

**Relationships**:
- Referenced by "Building Your Physical AI Lab" section (hardware-lab.md)

---

### 5. Assessment

**Description**: Evaluation artifact for measuring student learning outcomes.

**Attributes**:

| Field | Type | Required | Description | Validation Rules |
|-------|------|----------|-------------|------------------|
| `id` | string | ✅ | Assessment identifier | Pattern: `assessment-module-[0-4]-[type]` |
| `module` | string | ✅ | Parent module ID | Must match existing module |
| `title` | string | ✅ | Assessment name | 5-100 characters |
| `type` | enum | ✅ | Assessment category | `conceptual \| computational \| implementation` |
| `estimatedTime` | number | ✅ | Completion time (minutes) | Range: 15-180 minutes |
| `rubric` | Rubric | ✅ | Grading criteria | 3-level proficiency |

**Rubric Sub-Entity**:
```typescript
interface Rubric {
  novice: string;  // Emerging understanding
  proficient: string;  // Meets learning objectives
  advanced: string;  // Exceeds expectations
}
```

**Example**:
```json
{
  "id": "assessment-module-1-implementation",
  "module": "module-1",
  "title": "ROS 2 Multi-Node Communication System",
  "type": "implementation",
  "estimatedTime": 120,
  "rubric": {
    "novice": "Creates single publisher-subscriber pair with hardcoded topics; basic functionality works but lacks error handling or QoS configuration.",
    "proficient": "Implements multi-node system with custom messages, proper QoS policies, and error handling; demonstrates understanding of ROS 2 communication patterns.",
    "advanced": "Extends system with parameter configuration, lifecycle management, and comprehensive testing; shows mastery of ROS 2 best practices and real-world robustness."
  }
}
```

**Relationships**:
- N:1 with Module (each assessment belongs to one module)

---

## Entity Relationships Diagram

```
Module (6 total)
├── 1:N → Chapter (3-5 per module, 1 for capstone)
│   ├── N:M → GlossaryEntry (chapters reference terms, terms used across chapters)
│   └── 1:N → CodeExample (embedded + /examples/)
└── 1:N → Assessment (3+ per module)

HardwareSetup (standalone, referenced by hardware-lab.md)

FlexSearch Index ← builds from GlossaryEntry
Algolia DocSearch ← indexes all Chapter content
```

## Validation Rules Summary

**Global Constraints**:
- Total modules: Exactly 6 (Module 0-4 + Capstone)
- Total chapters: Exactly 21 (distributed as: 3, 5, 4, 5, 3, 1)
- Chapter template: Exactly 12 sections (enforced via JSON Schema)
- Code dependencies: Must use `==` version pinning (e.g., `numpy==1.24.3`)
- Images: ≤500KB each (enforced via CI image-optimization.yml)
- Glossary: 100+ terms minimum for comprehensive coverage

**Frontmatter Schema Validation**:
- All chapter frontmatter validated against `chapter-metadata-schema.json` during build
- `ajv` validator fails build if any field violates schema
- TypeScript types generated from schemas ensure compile-time safety

**Content Validation**:
- Learning objectives: 3-7 per chapter/module, active voice verbs
- Prerequisites: At least 1 prerequisite per chapter (except Module 0 Chapter 1)
- Exercises: Minimum 3 per chapter (1 conceptual, 1 computational, 1+ implementation)
- References: Minimum 5 per chapter, proper citation format

## Data Flow

**Content Creation**:
1. Author creates chapter MDX with frontmatter
2. Build-time `ajv` validates frontmatter against schema
3. TypeScript React components consume validated data
4. FlexSearch indexes glossary terms
5. Algolia indexes full chapter content

**CI/CD Validation**:
1. JSON Schema validation (frontmatter compliance)
2. Link checker (internal/external references)
3. Docker validation (code examples in /examples/)
4. Image optimization (≤500KB enforcement)
5. Lighthouse (performance/accessibility ≥90)

**User Experience**:
1. Reader navigates via sidebar (3-level nested)
2. Dashboard shows module progress
3. GlossarySearch enables instant term lookup (FlexSearch <50ms)
4. Algolia DocSearch provides full-text search with facets (module, week, difficulty)
5. ChapterMeta displays prerequisites, learning objectives, estimated time

---

**Data Model Version**: 1.0.0
**Last Updated**: 2025-12-06
**Status**: Complete (ready for contract generation)
