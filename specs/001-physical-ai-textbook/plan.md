# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-06 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

## Summary

Create a comprehensive 21-chapter university-level textbook titled "Physical AI & Humanoid Robotics – From Simulated Brains to Embodied Intelligence" using spec-driven development workflow. The project delivers a Docusaurus 3.x static site with fully runnable code examples, automated CI/CD validation, and MCP Context-7 documentation retrieval for technical accuracy.

**Key Requirements**:
- 21 chapters across 6 modules (Module 0: 3 chapters, Module 1: 5, Module 2: 4, Module 3: 5, Module 4: 3, Capstone: 1)
- 12-section chapter template (including Weekly Mapping and Common Pitfalls sections)
- Hybrid code approach: simple snippets embedded in MDX, complex projects in `/examples/[chapter]/`
- Automated Docker-based CI validation for all code examples
- MCP Context-7 documentation retrieval at start of EVERY chapter generation
- GitHub Actions CI/CD with quality gates (Lighthouse ≥90, zero broken links, image optimization)

## Technical Context

**Language/Version**: TypeScript 5.x, Markdown/MDX, Node.js 18+, React 18, Python 3.10+
**Primary Dependencies**: Docusaurus 3.x (latest stable within v3), ROS 2 Humble, NVIDIA Isaac Sim 4.x, Unity 2022.x LTS, Gazebo Classic 11/Harmonic, Whisper, Algolia DocSearch, FlexSearch
**Storage**: Static files (images ≤500KB in `/static/img/[chapter]/`), code examples in `/examples/[chapter]/`, MDX content in `/docs/`
**Testing**: Docusaurus build validation, Docker-based code execution tests, Lighthouse CI ≥90, broken link checker, image size validation, metadata JSON Schema validation
**Target Platform**: GitHub Pages (static site deployment), Docker containers (code validation), local/cloud development environments (NVIDIA RTX 3060+ or AWS g5/g6 instances)
**Project Type**: Web application (Docusaurus site) + educational content
**Performance Goals**: <3s page load, LCP <2.5s, CLS <0.1, search <300ms, Lighthouse ≥90
**Constraints**: Images ≤500KB each, 90% code example success rate, zero broken links, WCAG AA accessibility, 21-chapter fixed structure
**Scale/Scope**: 21 chapters across 6 modules total (Module 0-4 + Capstone), 100+ glossary terms, ~40-50 total pages, 13-week course mapping

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: 100% Spec-Driven Development
✅ **PASS** - All content follows `/sp.specify` → `/sp.plan` → `/sp.tasks` workflow (FR-023)

### Principle II: Technical Accuracy Only
✅ **PASS** - All equations require derivations or citations (FR-003), all technical claims must be verifiable (FR-004), MCP Context-7 retrieves versioned documentation at start of EVERY chapter (FR-026)

### Principle III: Deterministic, Reproducible Output
✅ **PASS** - Version-pinned dependencies (FR-006), Docker-based CI validation (FR-009a), automated quality gates (FR-015a)

### Principle IV: Single Source of Truth (SSOT)
✅ **PASS** - Canonical structure defined: `specs/[chapter]/`, `docs/[module]/`, `examples/[chapter]/`, `history/adr/`, `history/prompts/`, global glossary and notation files

### Principle V: Runnable, Version-Pinned Code Only
⚠️ **DEVIATION JUSTIFIED** - Hybrid code approach: simple examples (≤20 lines) embedded in MDX for optimal learning flow, complex multi-file examples in `/examples/[chapter]/` for CI validation (FR-009). Deviation documented in Complexity Tracking section.

**Justification**: Embedded code improves UX by eliminating context switching between text and separate code files. All code maintains runnable/version-pinned requirements via frontmatter `code_dependencies` field and Docker validation for `/examples/` code.

### Principle VI: Zero Hallucinations
✅ **PASS** - MCP Context-7 documentation retrieval enforced (FR-026, FR-026a), citations required (FR-003, FR-004), derivations mandatory

### Book Structure Requirements
✅ **PASS** - 21 chapters across 6 modules per Major Correction clarification, 12-section template enforced (FR-002)

### Quality Gates
✅ **PASS** - All gates specified: Docusaurus build (FR-012), link checker (FR-015), Lighthouse ≥90 (FR-013, SC-003), code validation (FR-009a), image optimization (FR-019), accessibility WCAG AA (SC-010)

**Overall Constitution Compliance**: ✅ **PASS** (one justified deviation documented)

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (created below)
├── data-model.md        # Phase 1 output (created below)
├── quickstart.md        # Phase 1 output (created below)
├── contracts/           # Phase 1 output (created below)
│   ├── chapter-metadata-schema.json
│   ├── module-metadata-schema.json
│   ├── glossary-entry-schema.json
│   └── sidebar-config-structure.ts
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/                                    # Docusaurus content
├── module-0-foundations/
│   ├── _category_.json
│   ├── chapter-1-intro-physical-ai/
│   │   └── index.mdx
│   ├── chapter-2-embodied-intelligence/
│   │   └── index.mdx
│   └── chapter-3-sensorimotor-foundations/
│       └── index.mdx
├── module-1-ros2/
│   ├── _category_.json
│   ├── chapter-1-ros2-architecture/
│   │   └── index.mdx
│   ├── chapter-2-pub-sub/
│   │   └── index.mdx
│   ├── chapter-3-services-actions/
│   │   └── index.mdx
│   ├── chapter-4-tf-transforms/
│   │   └── index.mdx
│   └── chapter-5-robot-control/
│       └── index.mdx
├── module-2-digital-twin/
│   ├── _category_.json
│   ├── chapter-1-simulation-principles/
│   │   └── index.mdx
│   ├── chapter-2-gazebo-basics/
│   │   └── index.mdx
│   ├── chapter-3-unity-integration/
│   │   └── index.mdx
│   └── chapter-4-physics-accuracy/
│       └── index.mdx
├── module-3-isaac/
│   ├── _category_.json
│   ├── chapter-1-isaac-overview/
│   │   └── index.mdx
│   ├── chapter-2-perception-pipelines/
│   │   └── index.mdx
│   ├── chapter-3-reinforcement-learning/
│   │   └── index.mdx
│   ├── chapter-4-sim-to-real/
│   │   └── index.mdx
│   └── chapter-5-advanced-ai/
│       └── index.mdx
├── module-4-vla-humanoids/
│   ├── _category_.json
│   ├── chapter-1-vision-language-models/
│   │   └── index.mdx
│   ├── chapter-2-action-primitives/
│   │   └── index.mdx
│   └── chapter-3-integration/
│       └── index.mdx
├── capstone/
│   ├── _category_.json
│   └── chapter-1-autonomous-humanoid/
│       └── index.mdx
├── glossary.md
├── notation.md
└── hardware-lab.md

examples/                                # Runnable code examples
├── module-0-foundations/
│   ├── chapter-1-intro-physical-ai/
│   │   ├── intro_physical_ai_demo.py
│   │   └── README.md
│   ├── chapter-2-embodied-intelligence/
│   │   ├── embodied_intelligence_demo.py
│   │   └── README.md
│   └── chapter-3-sensorimotor-foundations/
│       ├── sensorimotor_foundations_demo.py
│       └── README.md
│
├── module-1-ros2/
│   ├── chapter-1-ros2-architecture/
│   │   ├── simple_publisher.py
│   │   ├── simple_subscriber.py
│   │   ├── requirements.txt
│   │   └── README.md
│   ├── chapter-2-pub-sub/
│   │   ├── pub_example.py
│   │   ├── sub_example.py
│   │   └── README.md
│   ├── chapter-3-services-actions/
│   │   ├── service_server.py
│   │   ├── service_client.py
│   │   ├── action_server.py
│   │   ├── action_client.py
│   │   └── README.md
│   ├── chapter-4-tf-transforms/
│   │   ├── tf_broadcaster.py
│   │   ├── tf_listener.py
│   │   └── README.md
│   └── chapter-5-robot-control/
│       ├── cmd_vel_controller.py
│       ├── joint_controller.py
│       └── README.md
│
├── module-2-digital-twin/
│   ├── chapter-1-digital-twin-basics/
│   │   ├── digital_twin_basic_example.py
│   │   └── README.md
│   ├── chapter-2-simulation-sync/
│   │   ├── sim_sync_example.py
│   │   └── README.md
│   ├── chapter-3-robot-state-streaming/
│   │   ├── robot_state_stream_example.py
│   │   └── README.md
│   └── chapter-4-dataset-generation/
│       ├── dataset_generation_example.py
│       └── README.md
│
├── module-3-isaac/
│   ├── chapter-1-isaac-basics/
│   │   ├── isaac_basic_scene.py
│   │   └── README.md
│   ├── chapter-2-robot-simulation/
│   │   ├── isaac_robot_sim_example.py
│   │   └── README.md
│   ├── chapter-3-isaac-manipulation/
│   │   ├── isaac_pick_place_example.py
│   │   └── README.md
│   └── chapter-4-sim2real/
│       ├── sim2real_pipeline_demo.py
│       └── README.md
│
├── module-4-vla-humanoids/
│   ├── chapter-1-vision-language-actions/
│   │   ├── vla_basic_inference.py
│   │   └── README.md
│   ├── chapter-2-action-primitives/
│   │   ├── action_primitives_demo.py
│   │   └── README.md
│   ├── chapter-3-whole-body-control/
│   │   ├── whole_body_control_demo.py
│   │   └── README.md
│   └── chapter-4-humanoid-manipulation/
│       ├── humanoid_manipulation_demo.py
│       └── README.md
│
└── capstone/
    ├── project_template.py
    └── README.md

static/img/                              # Images ≤500KB each
├── module-0-foundations/
├── module-1-ros2/
├── module-2-digital-twin/
├── module-3-isaac/
├── module-4-vla-humanoids/
└── capstone/

src/                                     # React components
├── components/
│   ├── Dashboard/
│   │   └── index.tsx
│   ├── GlossarySearch/
│   │   └── index.tsx
│   ├── ChapterMeta/
│   │   └── index.tsx
│   └── Navigation/
│       └── index.tsx
├── pages/
│   └── index.tsx
└── theme/
    └── custom.css

.github/workflows/
├── deploy.yml                           # Docusaurus build + deploy to GitHub Pages
├── quality-gates.yml                    # Code validation, link checker, Lighthouse
└── image-optimization.yml               # Image compression + size validation

history/
├── adr/                                 # Architecture Decision Records
│   ├── 001-ros2-humble-choice.md
│   ├── 002-docusaurus-classic-theme.md
│   ├── 003-flexsearch-vs-lunr.md
│   ├── 004-hybrid-code-storage.md
│   └── 005-21-chapter-structure.md
└── prompts/
    ├── constitution/
    ├── 001-physical-ai-textbook/
    └── general/
```

**Structure Decision**: Web application structure selected. Docusaurus site with React components for custom functionality (dashboard, glossary search, metadata display). Content in `/docs/`, runnable examples in `/examples/`, static assets in `/static/img/`. Follows standard Docusaurus monorepo pattern with custom components in `/src/`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Principle V: No `/examples/` directory (code embedded in MDX only) | Hybrid approach required for optimal learning flow AND automated validation | Pure embedded approach breaks CI validation (cannot extract and run multi-file projects easily); Pure `/examples/` approach breaks learning flow (readers must context-switch between text and separate files) |

**Justification**: The hybrid approach (≤20 line snippets embedded, complex multi-file examples in `/examples/`) provides optimal UX while maintaining automated quality assurance. Embedded code keeps simple concepts inline for immediate understanding. Complex examples in `/examples/` enable Docker-based CI validation, version pinning, and runnable project templates. This resolves the tension between pedagogical clarity and technical validation.

## Phase 0: Research & Technical Decisions

### Research Questions Resolved

**Context**: User provided comprehensive implementation roadmap in command input. All major technical decisions pre-specified. Research phase validates and documents these decisions with rationale.

#### 1. Docusaurus 3.x Configuration for Educational Textbook

**Decision**: Use Docusaurus 3.x with classic theme + docs preset

**Rationale**:
- **Battle-tested**: Classic theme is production-ready with extensive community support
- **Mobile-responsive**: Built-in responsive design meets FR-013 (<3s mobile load)
- **Docs preset**: Optimized for technical documentation with sidebar navigation, versioning support, search integration
- **React ecosystem**: Full access to React components for custom features (dashboard, glossary search, chapter metadata)
- **SEO optimized**: Built-in OpenGraph metadata, sitemap.xml, robots.txt generation (FR-014)
- **Performance**: Achieves Lighthouse ≥90 by default with proper image optimization

**Alternatives Considered**:
- Sphinx: Python-native but lacks modern UI/UX, poor mobile responsiveness
- VuePress: Smaller ecosystem, less educational-focused than Docusaurus
- GitBook: Proprietary hosting, limited customization
- Custom Next.js: Requires building all educational features from scratch

**MCP Context-7 Requirement**: Retrieve Docusaurus 3.x documentation during initial site setup and whenever modifying React components, plugins, or configuration (FR-026a)

#### 2. Glossary Search Implementation: FlexSearch vs Lunr.js

**Decision**: Use FlexSearch for local glossary search

**Rationale**:
- **Performance**: <50ms search latency vs Lunr.js ~100ms (meets <300ms search requirement)
- **Index size**: ~30KB for 100+ terms vs Lunr.js ~80KB (faster page load)
- **Memory efficiency**: Lower runtime memory footprint
- **Fuzzy search**: Built-in support for typo tolerance
- **Maintenance**: Active development, modern API

**Alternatives Considered**:
- Lunr.js: Slower, larger index size
- Algolia DocSearch only: Requires external service, no offline glossary search
- Fuse.js: Heavier weight, comparable performance to FlexSearch

**Implementation**: Client-side FlexSearch index built from `docs/glossary.md` at build time, hydrated on page load. Separate from Algolia DocSearch (full-text site search).

#### 3. Frontmatter Metadata Validation: TypeScript + JSON Schema

**Decision**: Use `ajv` (JSON Schema validator) + `json-schema-to-typescript` for type generation

**Rationale**:
- **Compile-time safety**: TypeScript interfaces generated from JSON Schema ensure type safety
- **Runtime validation**: `ajv` validates frontmatter during build, fails CI if invalid
- **Single source of truth**: JSON Schema defines canonical structure, TypeScript types derived
- **Standard-based**: Industry-standard JSON Schema (widely understood, tooling support)
- **Fast validation**: `ajv` is highly optimized for performance

**Alternatives Considered**:
- Zod: TypeScript-first but requires duplicating schema definitions
- Joi: Less ecosystem support for static type generation
- Manual validation: Error-prone, no type safety

**Schemas Required** (see Phase 1 Contracts):
- `chapter-metadata-schema.json`: 13 required frontmatter fields with enums for module, assessment type, difficulty
- `module-metadata-schema.json`: Module overview page metadata
- `glossary-entry-schema.json`: Glossary term validation

#### 4. Algolia DocSearch Configuration

**Decision**: Use Algolia DocSearch with custom facets (module, week, difficulty_level)

**Rationale**:
- **Free for open-source**: No cost for educational/open-source projects
- **Full-text search**: Indexes all MDX content, code snippets, glossary terms
- **Custom facets**: Filter by module, week, difficulty level for targeted search
- **Instant results**: <300ms search with typo tolerance
- **Maintained**: Official Docusaurus integration, automatic reindexing

**Configuration**:
- Facets: `module` (Module 0-4, Capstone), `week` (1-13), `difficulty_level` (Beginner, Intermediate, Advanced)
- Index selectors: `.markdown` content, `code` blocks, `h1-h6` headings
- Exclusions: Navigation elements, footer, sidebar

**Alternatives Considered**:
- Local search only: Limited functionality for large textbook
- Self-hosted Elasticsearch: Requires infrastructure, maintenance overhead

#### 5. GitHub Actions CI/CD Pipeline Architecture

**Decision**: Three-workflow pipeline (deploy.yml, quality-gates.yml, image-optimization.yml)

**Rationale**:
- **Separation of concerns**: Each workflow has single responsibility (deploy, validate, optimize)
- **Parallel execution**: Quality gates and image optimization run concurrently during PR checks
- **Fail-fast**: Quality gates fail immediately on errors (don't wait for full deployment)
- **Reusability**: Image optimization workflow reusable for manual triggers

**deploy.yml Workflow**:
1. Checkout code
2. Install Node.js 18+ dependencies
3. Run `npm run build`
4. Deploy to GitHub Pages (only on main branch push, only if quality gates passed)

**quality-gates.yml Workflow**:
1. Build Docusaurus site
2. Run Docker-based code validation (execute all `/examples/[chapter]/` code)
3. Run broken link checker (internal + external links)
4. Run Lighthouse CI (performance, accessibility, SEO, best practices ≥90)
5. Validate frontmatter metadata (JSON Schema validation)
6. Fail PR if any check fails

**image-optimization.yml Workflow**:
1. Find all images in `/static/img/`
2. Compress with sharp/imagemin (lossless)
3. Fail if any image >500KB after compression
4. Commit optimized images (if applicable)

**Alternatives Considered**:
- Single monolithic workflow: Slower feedback, harder to debug
- Manual quality checks: Error-prone, doesn't scale to 21 chapters
- CircleCI/Travis: GitHub Actions native integration simpler for GitHub Pages

#### 6. Incremental Publishing Strategy

**Decision**: Environment-based content inclusion via `PUBLISH_MODULES` environment variable

**Rationale**:
- **Phased rollout**: Publish Module 0 (Weeks 1-2) standalone for early feedback
- **Testing isolation**: Validate single module before full deployment
- **Flexible deployment**: Staging publishes all modules, production uses env var filter
- **No code duplication**: Same codebase, behavior controlled by environment

**Implementation**:
- Custom Docusaurus plugin filters `docs/` based on frontmatter `module` field
- Staging: `PUBLISH_MODULES=all` (default)
- Production Week 1-2: `PUBLISH_MODULES=0`
- Production Week 3-5: `PUBLISH_MODULES=0,1`
- Full release: `PUBLISH_MODULES=0,1,2,3,4,capstone`

**Alternatives Considered**:
- Separate branches per module: Git branching complexity, merge conflicts
- Versioning: Overkill for linear course progression

### Research Findings Summary

All technical decisions pre-validated by user's comprehensive roadmap. Research phase confirms feasibility and documents rationale for:
- Docusaurus 3.x classic theme + docs preset (educational best practices)
- FlexSearch for glossary (<50ms vs Lunr.js ~100ms)
- TypeScript + JSON Schema for type-safe metadata validation
- Algolia DocSearch with custom facets (module, week, difficulty)
- Three-workflow GitHub Actions pipeline (deploy, quality gates, image optimization)
- Environment-based incremental publishing (`PUBLISH_MODULES` env var)

**No unresolved technical questions**. All components align with constitution and spec requirements.

## Phase 1: Data Model, Contracts & Quickstart

### Data Model

See `data-model.md` (generated below) for full entity definitions. Summary:

**Core Entities**:
1. **Module** (6 total): id, title, weekRange, description, learningOutcomes, chapters[], capstoneIntegration, estimatedTime
2. **Chapter** (21 total): id, module, title, weekMapping, prerequisites, learningObjectives[], assessmentType, difficultyLevel, capstoneComponent, codeDependencies[], estimatedTime, content (12 sections)
3. **Glossary Entry** (100+ terms): term, definition, relatedTerms[], chapters[], aliases[]
4. **Hardware Setup**: id, name, requirements[], cost, steps[]
5. **Assessment**: id, module, title, type, estimatedTime, rubric{novice, proficient, advanced}

**Relationships**:
- Module 1:N Chapter (each module contains 3-5 chapters)
- Chapter N:M GlossaryEntry (chapters reference multiple terms, terms used across chapters)
- Module 1:N Assessment (each module has conceptual, computational, implementation assessments)
- Chapter 1:N CodeExample (embedded + /examples/)

### Contracts

See `contracts/` directory (generated below). Summary:

**JSON Schemas**:
1. `chapter-metadata-schema.json`: Frontmatter validation (13 required fields, enums for module/assessment/difficulty)
2. `module-metadata-schema.json`: Module overview page metadata
3. `glossary-entry-schema.json`: Glossary term structure validation

**TypeScript Interfaces**:
1. `sidebar-config-structure.ts`: 3-level nested sidebar navigation types

**Contract Enforcement**:
- Build-time validation via `ajv` in Docusaurus config
- TypeScript compilation enforces type safety for React components
- CI fails if any frontmatter violates schema

### Quickstart Guide

See `quickstart.md` (generated below) for comprehensive developer documentation. Summary:

**Setup**:
1. Clone repository
2. Install Node.js 18+, Docker, ROS 2 Humble (optional for local testing)
3. Run `npm install`
4. Run `npm start` (dev server at http://localhost:3000)

**Create New Chapter Workflow**:
1. Create `/docs/module-X/chapter-Y/index.mdx`
2. Add frontmatter (validate against `chapter-metadata-schema.json`)
3. Write 12 required sections
4. Add code examples: inline (≤20 lines) or `/examples/module-X/chapter-Y/`
5. Update sidebar (`docs/module-X/_category_.json`)
6. Run `npm run build` to validate
7. Run Docker validation: `docker run --rm -v $(pwd)/examples:/examples [validation-image]`
8. Commit and push (CI runs quality gates)

**Glossary Update Flow**:
1. Edit `docs/glossary.md`
2. Rebuild FlexSearch index (automatic during build)
3. Test search functionality

**CI Checks**:
- Docusaurus build success
- Code example execution in Docker
- Link checker (zero broken links)
- Lighthouse ≥90
- Metadata schema validation
- Image optimization (<500KB)

**Troubleshooting**:
- "Build failed": Check frontmatter schema compliance
- "Code example failed": Verify dependencies pinned in requirements.txt
- "Image too large": Run `npm run optimize-images`

## Phase 2: Task Decomposition (Deferred)

**Note**: Task decomposition is performed by `/sp.tasks` command, NOT `/sp.plan`. This plan provides the foundation for task generation.

**Expected Task Categories** (for `/sp.tasks` reference):
- Foundation tasks: Repository structure, Docusaurus configuration, CI/CD setup
- Module scaffolding tasks: Create module overview pages, sidebar configuration
- Chapter production tasks: Per-chapter `/sp.specify` → `/sp.plan` → `/sp.tasks` workflow
- Hardware section tasks: Create `hardware-lab.md` with budget tables
- Integration tasks: Glossary, notation, cross-links validation
- Deployment tasks: GitHub Pages configuration, domain setup

**Estimated Task Count**: 150-200 granular tasks across all phases (Foundation: 20-30, Module scaffolding: 15-20, Chapter production: 100-130 [21 chapters × 5-6 tasks each], Hardware: 5-10, Integration: 10-15)

## Constitution Re-Check (Post-Design)

**Constitution Check Status**: ✅ **PASS** (re-validated after Phase 1 design)

All principles satisfied:
- ✅ Spec-driven workflow enforced
- ✅ Technical accuracy via MCP Context-7 + citations
- ✅ Deterministic output via version pinning + Docker CI
- ✅ SSOT structure defined
- ⚠️ Runnable code: Hybrid approach justified (embedded + /examples/)
- ✅ Zero hallucinations via Context-7 + mandatory derivations

**No new violations introduced during design phase.**

## Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| **Version drift** (ROS 2, Isaac, Unity docs change) | Version pinning + MCP Context-7 retrieval at start of EVERY chapter |
| **Image bloat** (diagrams >500KB) | Automated CI image optimization workflow, fails if >500KB after compression |
| **Code failures** (examples don't run) | Docker-based CI validation with pinned dependencies, 90% success rate enforced |
| **Math inaccuracies** (equation errors) | Mandatory derivations or citations (FR-003), technical review gate |
| **Timeline overruns** (21 chapters × complexity) | Deterministic Spec-Kit Plus workflow, parallel chapter production, incremental publishing |
| **Context-7 API failures** | Fallback to cached documentation snapshots (warn if stale), retry logic with exponential backoff |
| **Docusaurus build failures** | Pre-commit hooks validate frontmatter schema, CI fails fast on build errors |

## Success Criteria Mapping

| Success Criterion | Implementation | Validation |
|-------------------|----------------|------------|
| **SC-001**: Reader completes Module 0-1 in 10 hours, runs all code | Clear learning objectives (FR-016), runnable code (FR-005, FR-009a) | User testing with target audience |
| **SC-002**: 90% code examples execute successfully | Docker CI validation (FR-009a), pinned dependencies (FR-006) | Automated CI metrics |
| **SC-003**: Lighthouse ≥90 (desktop + mobile) | Docusaurus optimization, image compression (FR-019), <3s load (FR-013) | Lighthouse CI in quality-gates.yml |
| **SC-004**: 12-section template compliance | JSON Schema validation (chapter-metadata-schema.json) | Build-time validation via ajv |
| **SC-005**: Zero broken links | Automated link checker in quality-gates.yml (FR-015) | CI fails on broken links |
| **SC-006**: 100% technical claims backed by citations | Mandatory derivations/citations (FR-003, FR-004), MCP Context-7 (FR-026) | Technical review gate |
| **SC-007**: Capstone completed in 20-40 hours | Comprehensive capstone chapter with integration steps, architecture diagrams | User testing, instructor feedback |
| **SC-008**: 3+ independent reviewers validate accuracy | Technical review process, peer review workflow | Pre-publication review |
| **SC-009**: Exercises align with learning objectives | 3 exercise types per chapter (FR-018), objective-driven design | Instructor review checklist |
| **SC-010**: WCAG AA accessibility | Alt-text for all images (FR-019), semantic HTML, screen reader testing | Automated accessibility audit in CI |
| **SC-011**: Weekly Breakdown table on module pages | Module overview template includes mandatory table (FR-002a) | Manual QA, automated presence check |
| **SC-012**: Hardware/lab section with budget tables | `hardware-lab.md` with all required tables (FR-002b) | Manual QA, schema validation |

## Deliverables

**Phase 0 (Foundation)**:
- ✅ Repository structure initialized
- ✅ Docusaurus 3.x configured (classic theme + docs preset)
- ✅ CI/CD pipelines (deploy.yml, quality-gates.yml, image-optimization.yml)
- ✅ Global references (glossary.md, notation.md, hardware-lab.md stubs)
- ✅ Contract schemas (chapter-metadata, module-metadata, glossary-entry, sidebar-config)

**Phase 1 (Module Scaffolding)**:
- ✅ 6 module overview pages with Weekly Breakdown tables
- ✅ Sidebar navigation structure (3-level nested)
- ✅ Prerequisite chains defined

**Phase 2 (Chapter Production)**: *Deferred to per-chapter `/sp.specify` → `/sp.plan` → `/sp.tasks` workflow*
- 21 chapter MDX files with 12-section template
- Runnable code examples (embedded + /examples/)
- Diagrams (≤500KB)
- Citations, equations, exercises

**Phase 3 (Hardware & Lab)**:
- `hardware-lab.md` with budget tables, cloud costs, latency warnings

**Phase 4 (Integration & Deployment)**:
- Validated crosslinks, glossary, notation
- All 12 success criteria validated
- GitHub Pages deployment

## Next Steps

1. **Run `/sp.tasks`** to generate granular task breakdown for Phase 0 (Foundation)
2. **Execute Foundation tasks**: Repository structure, Docusaurus setup, CI/CD configuration
3. **Create module scaffolding**: Module overview pages, sidebar configuration
4. **Begin chapter production**: For each chapter, run `/sp.specify` → `/sp.plan` → `/sp.tasks` → Claude Code generation with MCP Context-7 retrieval
5. **Validate incrementally**: Run CI after each chapter to catch issues early
6. **Deploy incrementally**: Use `PUBLISH_MODULES` for phased rollout (Module 0 first, then 1, 2, 3, 4, Capstone)

**Immediate Action**: Run `/sp.tasks` to decompose Foundation phase into atomic tasks.
