---
id: 003
title: Implementation Planning Complete
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: 003-auth-personalization-translation
branch: 003-auth-personalization-translation
user: Mirza-Usman-247
command: /sp.plan
labels: ["planning", "authentication", "personalization", "translation", "rag-integration", "better-auth", "fastapi", "openrouter"]
links:
  spec: specs/003-auth-personalization-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-auth-personalization-translation/plan.md
  - specs/003-auth-personalization-translation/research.md
  - specs/003-auth-personalization-translation/data-model.md
  - specs/003-auth-personalization-translation/quickstart.md
  - specs/003-auth-personalization-translation/contracts/auth.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/personalization.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/translation.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/profile.openapi.yaml
tests:
  - Constitution Check passed (all gates)
  - Phase 0 research completed (all technical unknowns resolved)
  - Phase 1 design completed (data model, contracts, quickstart generated)
---

## Prompt

Execution Plan: Auth, Personalization, and Translation

Phase 1: Authentication
- Integrate Better Auth into Docusaurus frontend
- Implement signup/signin flows
- Store user profile in Neon Postgres

Phase 2: Personalization Engine
- Add "Personalize Chapter" button to chapters
- Backend endpoint: /personalize
- Use OpenRouter free LLM models
- Personalization logic based on user profile

Phase 3: Translation Engine
- Add "Translate to Urdu" button
- Backend endpoint: /translate
- Focus Mode Urdu translation via OpenRouter free model

Phase 4: RAG Integration
- Pass personalized/translated content to RAG pipeline
- Ensure rag-retriever and rag-answerer respect modified content

Phase 5: QA & Performance
- Validate auth security
- Validate personalization accuracy
- Validate Urdu translation quality
- Measure latency and cost

Output:
Step-by-step execution plan aligned with existing RAG and book infrastructure.

## Response snapshot

Created comprehensive implementation plan with Phase 0 (Research) and Phase 1 (Design) completed:

**plan.md**:
- Technical Context: TypeScript 5.x + React 18 (Docusaurus), Python 3.11+ (FastAPI), Better Auth, Neon Postgres, OpenRouter API
- Constitution Check: ✅ PASS (all gates passed, no violations detected)
- Project Structure: Documented frontend (Docusaurus) and backend (FastAPI) directory structure with all new files
- Performance Goals: Registration <3min, Personalization <10s, Translation <15s, RAG <3s
- Scale/Scope: 100 concurrent users, session-only storage, per-chapter transformations

**research.md**:
- Decision 1: Better Auth + FastAPI JWT validation (modern, lightweight, React-first auth library)
- Decision 2: SQLAlchemy ORM with Alembic migrations for Neon Postgres (version-controlled schema changes)
- Decision 3: OpenRouter free tier with graceful degradation (researched exact limits: 20 req/min, 200 req/day, 8K input tokens)
- Decision 4: IndexedDB for session-only storage (50MB+ capacity vs 5-10MB SessionStorage limit)
- Decision 5: Stateless RAG chatbot with frontend-sent context (simplifies backend, no session sync)
- Decision 6: Comprehensive security hardening (bcrypt cost factor 12, 15-min JWT expiry, rate limiting, CORS restrictions)
- Technology stack summary table with rationale for each component
- Performance optimization strategy (code splitting, connection pooling, async IO, parallel LLM requests)
- Deployment considerations (environment variables, infrastructure)
- Testing strategy (unit, integration, performance tests)

**data-model.md**:
- Database schema: `users` table (UUID PK, email UK, bcrypt password_hash, is_active flag)
- Database schema: `user_profiles` table (user_id FK, 5 mandatory skill levels 1-5, 3 optional hardware boolean flags, computed cloud_only column)
- Entity relationships: One-to-one User-UserProfile with CASCADE delete
- Session storage schema: IndexedDB TransformedContentStore with composite key (userId:chapterId:transformationType)
- API request/response models: Signup, Signin, Refresh, Personalize, Translate, RAG Chat Query (modified)
- Data flow diagrams: Authentication flow, Personalization flow, RAG chatbot with transformed content
- Validation rules: Backend (Pydantic models), Frontend (Zod schemas)
- Database migration strategy: Alembic migration with full SQL DDL

**quickstart.md**:
- Prerequisites: Node.js 18+, Python 3.11+, Neon account, OpenRouter account
- Backend setup: Python venv, requirements.txt additions, .env configuration, database migrations, server start
- Frontend setup: npm dependencies, Better Auth configuration, .env.local, Docusaurus start
- Development workflow: Create test user, test personalization API, test translation API
- Troubleshooting: Database connection, JWT tokens, OpenRouter API, Better Auth
- Testing checklist: 10 verification items before proceeding to implementation

**contracts/** (4 OpenAPI 3.0.3 specifications):
- auth.openapi.yaml: Signup, Signin, Refresh, Logout endpoints with JWT bearer auth
- personalization.openapi.yaml: /personalize POST endpoint with user profile and chapter content
- translation.openapi.yaml: /translate POST endpoint with Focus Mode Urdu translation
- profile.openapi.yaml: /profile GET endpoint for retrieving user profile

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation; all technical unknowns resolved; database schema designed; API contracts specified; development environment documented
- 🧪 Tests: Constitution Check passed; all design artifacts validated
- 📁 Files: Created 8 planning artifacts (plan.md, research.md, data-model.md, quickstart.md, 4 OpenAPI contracts)
- 🔁 Next prompts: `/sp.tasks` to generate granular task breakdown for implementation
- 🧠 Reflection: Research phase successfully resolved all technical unknowns (OpenRouter limits, Better Auth integration, session storage strategy, RAG integration pattern); Design phase produced complete data model and API contracts; Ready for task generation and implementation

## Evaluation notes (flywheel)

- Failure modes observed: None; planning completed successfully with all research findings documented and validated
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all 8 quality gates passed, no violations detected)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 (Research) and Phase 1 (Design) fully completed
- Next experiment (smallest change to try): For future planning work, consider creating architecture decision records (ADRs) for major decisions (Better Auth choice, session storage strategy, stateless RAG pattern) as identified in Constitution Check evaluation
