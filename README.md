# Physical AI & Humanoid Robotics Textbook

**From Simulated Brains to Embodied Intelligence**

A comprehensive university-level textbook covering Physical AI and Humanoid Robotics, built with Docusaurus 3.x.

## Overview

This project is a complete educational resource for learning Physical AI and humanoid robotics through structured, progressive content with hands-on code examples. The textbook covers 21 chapters across 6 modules, following a 13-week course structure.

### Modules

- **Module 0: Foundations** (3 chapters, Weeks 1-2)
- **Module 1: ROS 2 Fundamentals** (5 chapters, Weeks 3-5)
- **Module 2: Digital Twin & Simulation** (4 chapters, Weeks 6-7)
- **Module 3: NVIDIA Isaac Sim** (5 chapters, Weeks 8-10)
- **Module 4: VLA Robotics** (3 chapters, Weeks 11-12)
- **Capstone Project** (1 chapter, Week 13)

## Quick Start

### Prerequisites

- Node.js 20.0 or higher
- npm or yarn package manager
- Git

### Installation

```bash
# Clone the repository
git clone https://github.com/Mirza-Usman-247/physical-ai-and-humanoid-robotics-textbook.git
cd physical-ai-and-humanoid-robotics-textbook

# Install dependencies
npm install

# Start development server
npm start
```

The site will be available at `http://localhost:3000`.

### Build for Production

```bash
# Build static site
npm run build

# Serve built site locally
npm run serve
```

## Project Structure

```
physical-ai-textbook/
├── docs/                   # MDX content (6 modules)
├── examples/               # Runnable code examples (per chapter)
├── src/                    # React components
│   └── css/               # Custom styles
├── static/                # Static assets
│   └── img/               # Images (≤500KB each)
├── .github/workflows/     # CI/CD pipelines
├── specs/                 # Specifications and plans
└── history/               # ADRs and PHRs
```

## Development Workflow

For detailed development guidelines, see [Developer Quickstart Guide](specs/001-physical-ai-textbook/quickstart.md).

### Available Scripts

- `npm start` - Start development server
- `npm run build` - Build production site
- `npm run serve` - Serve production build locally
- `npm run typecheck` - Run TypeScript type checking
- `npm run lint` - Lint JavaScript/TypeScript files
- `npm run format` - Format code with Prettier
- `npm run check-links` - Validate internal/external links
- `npm run lighthouse` - Run Lighthouse performance audit

## Tech Stack

- **Docusaurus 3.x** - Static site generator
- **React 18** - UI components
- **TypeScript 5.x** - Type-safe development
- **MDX 3** - Markdown with React components
- **FlexSearch** - Local glossary search
- **Algolia DocSearch** - Full-text search
- **Lighthouse CI** - Performance monitoring

## Code Examples

All code examples are:
- **Runnable** - Validated in Docker containers via CI
- **Version-pinned** - Dependencies locked to specific versions
- **Safe** - Hardware-related code includes safety notes

### Technologies Covered

- ROS 2 Humble LTS
- NVIDIA Isaac Sim 4.x
- Unity 2022.x LTS
- Gazebo Classic 11 / Gazebo Harmonic
- Whisper (speech recognition)

## Contributing

This project follows Spec-Driven Development (SDD) workflow:

1. `/sp.specify` - Create feature specification
2. `/sp.clarify` - Resolve ambiguities
3. `/sp.plan` - Generate implementation plan
4. `/sp.tasks` - Break down into tasks
5. `/sp.implement` - Execute implementation

## Constitution

All development adheres to the project constitution v1.1.0:

1. 100% Spec-Driven Development
2. Technical Accuracy Only (citations/derivations required)
3. Deterministic, Reproducible Output
4. Single Source of Truth
5. Runnable, Version-Pinned Code Only
6. Zero Hallucinations

See [Constitution](.specify/memory/constitution.md) for full details.

## License

MIT License - see LICENSE file for details.

## Acknowledgments

Built with [Docusaurus](https://docusaurus.io/) and powered by spec-driven development principles.
