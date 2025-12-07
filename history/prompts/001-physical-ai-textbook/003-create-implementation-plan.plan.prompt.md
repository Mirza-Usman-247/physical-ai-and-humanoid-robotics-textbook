---
id: 003
title: Create implementation plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: /sp.plan
labels: ["plan", "architecture", "docusaurus", "textbook", "technical-design"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/plan.md
tests:
  - N/A (planning document)
---

## Prompt

Create an implementation plan for the following project specification.

Summary:
Create the foundational Docusaurus-based textbook structure for a 13-week Physical AI & Humanoid Robotics course. The book will contain 4 modules, a dashboard-style homepage, nested sidebar navigation, hybrid search (Algolia + glossary), TypeScript-based metadata validation, and rich chapter metadata including prerequisites, time estimates, and assessment types.

[Full detailed requirements provided including Technical Approach, Technical Context, Performance Goals, Constraints, Scale/Scope, Constitution Check, Repository Structure, Complexity Tracking, Phase 0 Research Tasks, Phase 1 Design & Contracts, and Success Criteria Mapping]

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics Textbook Docusaurus site.

**Plan Structure**:

**Technical Context** (fully specified):
- Languages: TypeScript 5.x, Markdown/MDX, Node.js 18+, React 18
- Stack: Docusaurus 3.x (classic theme + docs preset), Algolia DocSearch, FlexSearch (glossary), JSON Schema validation
- Testing: Docusaurus build, broken link checker, Lighthouse CI ≥90, metadata validation
- Performance: <3s page load, LCP <2.5s, CLS <0.1, search <300ms
- Scale: 20-25 chapters, 5 modules, 100+ glossary terms, ~40-50 pages

**Constitution Check** (all gates passed):
- ✅ Principle I-VI: All requirements satisfied
- ⚠️ Deviation: No `/examples` directory (code embedded in chapters) - JUSTIFIED in Complexity Tracking
- ✅ 11-section chapter template enforced via FR-002
- ✅ Quality gates: build, links, Lighthouse ≥90, accessibility, metadata validation

**Project Structure**:
- `/docs`: 5 modules (module-0 through module-4-vla-humanoids), setup guides, capstone, assessments, references, instructors
- `/src`: React components (Dashboard, GlossarySearch, ChapterMeta, Navigation, common utilities)
- `/static`: Images (≤500KB), glossary search index
- `/.github/workflows`: deploy.yml, quality-gates.yml, image-optimization.yml
- Contracts directory with JSON Schemas and TypeScript interfaces

**Complexity Tracking**: Documented deviation from Principle V (no `/examples` directory) - embedded code improves UX, maintains runnable/version-pinned requirements via frontmatter `code_dependencies` field.

**Phase 0 Research** (6 tasks outlined):
1. Docusaurus 3.x educational best practices for multi-module textbook
2. FlexSearch vs Lunr.js comparison for glossary search
3. TypeScript + JSON Schema integration for frontmatter validation
4. Algolia DocSearch configuration with custom facets
5. GitHub Actions workflows (Docusaurus + Lighthouse CI + link checker)
6. Incremental publishing strategies (enable Module 0 standalone)

**Research Decisions** (embedded in plan.md):
- Classic theme + docs preset (battle-tested, mobile-responsive)
- FlexSearch for glossary (<50ms vs Lunr.js ~100ms, smaller index)
- `ajv` + `json-schema-to-typescript` for metadata validation
- Algolia DocSearch with custom facets (module, week, difficulty)
- Three-workflow GitHub Actions pipeline (deploy, quality gates, image optimization)
- Environment-based content inclusion (`PUBLISH_MODULES` env var)

**Phase 1 Design**:

**Data Model** (5 entities fully specified in plan.md):
- Module: id, title, weekRange, description, learningOutcomes, chapters, capstoneIntegration, estimatedTime
- Chapter: 13 frontmatter fields including prerequisites, learning_objectives, assessment_type, difficulty_level, capstone_component, code_dependencies
- Glossary Entry: term, definition, relatedTerms, chapters, aliases
- Hardware Setup: id, name, requirements, cost, steps
- Assessment: id, module, title, type, estimatedTime, rubric (3-level)

**Contracts** (4 JSON Schemas + 1 TypeScript interface created):
1. `chapter-metadata-schema.json`: Frontmatter validation (13 required fields, enums for module/assessment/difficulty)
2. `sidebar-config-structure.ts`: TypeScript interfaces for nested sidebar
3. `module-metadata-schema.json`: Module metadata validation
4. `glossary-entry-schema.json`: Glossary term validation

**Quickstart Guide** (comprehensive developer documentation):
- Install, run, build, deploy instructions
- Create new chapter workflow (file → frontmatter → validation → sidebar update → test)
- Glossary update flow (edit → rebuild index → test)
- CI checks (build, links, Lighthouse, metadata, image optimization)
- Incremental publishing (`PUBLISH_MODULES` env var)
- Troubleshooting guide

**Impact**: Plan provides complete architectural blueprint for Docusaurus textbook site. All technical unknowns resolved via research phase. Data model and contracts enable type-safe, schema-validated content. Quickstart enables efficient chapter creation workflow. Ready for `/sp.tasks` to generate granular task breakdown.

## Outcome

- ✅ Impact: Comprehensive implementation plan with technical context, constitution compliance, research findings, data model, contracts, and quickstart guide
- 🧪 Tests: N/A (planning document, but quality gate requirements specified: Lighthouse ≥90, link checker, metadata validation)
- 📁 Files: 1 primary file (plan.md with embedded research, data model, contracts, quickstart)
- 🔁 Next prompts: `/sp.tasks` to break plan into granular, executable tasks organized by phase
- 🧠 Reflection: Plan successfully balances user's detailed requirements with constitution compliance. Deviation from `/examples` directory documented and justified. FlexSearch vs Lunr.js research informs glossary implementation. Three-workflow GitHub Actions pipeline ensures quality without blocking deployment.

## Evaluation notes (flywheel)

- Failure modes observed: None - all user requirements incorporated, constitution gates passed with documented deviation
- Graders run and results (PASS/FAIL): PASS (Constitution Check: 6/6 principles satisfied with justified deviation, all quality gates specified)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
