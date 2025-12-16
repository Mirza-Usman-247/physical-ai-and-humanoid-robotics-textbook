---
id: 003
title: Implementation Planning Complete
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4.5
feature: 003-auth-personalization-translation
branch: 003-auth-personalization-translation
user: developer
command: /sp.plan
labels: ["planning", "architecture", "research", "data-model", "contracts", "phase-0", "phase-1"]
links:
  spec: specs/003-auth-personalization-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-auth-personalization-translation/plan.md
  - specs/003-auth-personalization-translation/research.md
  - specs/003-auth-personalization-translation/data-model.md
  - specs/003-auth-personalization-translation/contracts/auth.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/profile.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/personalization.openapi.yaml
  - specs/003-auth-personalization-translation/contracts/translation.openapi.yaml
  - specs/003-auth-personalization-translation/quickstart.md
tests:
  - Constitution Check: PASS (2 WATCH items for LLM non-determinism and quality validation)
  - Phase 0 research: All 7 research areas resolved
  - Phase 1 design: Data model, contracts, and quickstart generated
  - Schema validation: 4 tables, 2 ENUMs, 9 indexes defined
  - API contracts: 4 OpenAPI 3.1 specs with all endpoints documented
---

## Prompt

```text
Execution Plan: Auth, Personalization & Translation

Phase 1: Authentication
- Integrate Better Auth with FastAPI
- Create Neon tables for users and profiles
- Implement signup questionnaire
- Implement signin and session handling

Phase 2: Skill Development (Reusable)
- user-profile-skill: read/write user background
- personalization-skill: adapt content using profile
- translation-skill: translate content to Urdu
- content-adapter-skill: apply AI-generated changes safely

Phase 3: Backend APIs
- /auth/signup
- /auth/signin
- /user/profile
- /content/personalize
- /content/translate

Phase 4: Frontend Integration
- Add "Personalize Content" button per chapter
- Add "Translate to Urdu" button per chapter
- Connect buttons to backend APIs

Phase 5: Testing & Validation
- Verify Neon DB persistence
- Validate personalization accuracy
- Validate Urdu translation quality
- Ensure RAG chatbot compatibility

Output:
Step-by-step implementation plan explicitly using reusable skills.
```

## Response snapshot

**Executed `/sp.plan` workflow**: Phases 0 and 1 complete

### Phase 0: Research & Architecture Decisions (research.md)

Resolved 7 technical unknowns:

1. **Better Auth Integration**: Decided on FastAPI-Users (Better Auth has no Python support)
   - Rationale: Native FastAPI support, session-based auth, SQLAlchemy integration
   - Alternatives rejected: Authlib (JWT-focused), custom session management (security risks)

2. **Database Schema**: 3NF normalized design with JSONB + ENUM
   - Tables: users, profiles, preferences, translation_logs
   - JSONB for flexible multi-select software_skills array
   - ENUM for type-safe hardware_access field (GPU/Jetson/Robot/CloudOnly)
   - Foreign keys, CHECK constraints, GIN indexes for JSONB

3. **Redis Caching**: 30-minute TTL with profile hash for cache invalidation
   - Key format: `personalized:{userId}:{chapterId}:{profileHash}`
   - Profile hash: MD5 of profile fields (sufficient for non-cryptographic use)
   - Namespace isolation from RAG chatbot cache

4. **LLM Integration**: DeepSeek R1 (free) with GPT-3.5-turbo fallback
   - 8K context window, 50 requests/minute
   - Prompt templates for personalization and translation
   - Chunking strategy for long chapters (>6K tokens)

5. **Theme Customization**: MDXContent swizzling + sessionStorage
   - Inject buttons at chapter start via React component wrapper
   - Session storage for view preferences (personalized vs original)
   - Translation never persists (refresh → English per FR-029)

6. **Translation QA**: Regex validation + manual spot checks
   - Automated: 95% technical term preservation, 100% code preservation
   - Manual: Grammar and fluency review (sample 3 chapters per release)
   - Quality metrics returned in API response

7. **Security**: CSRF protection, Redis sessions, HTTPS, parameterized queries
   - FastAPI-Users built-in CSRF protection
   - 1-hour session timeout
   - Redis-backed sessions for horizontal scaling
   - psycopg3 parameterized queries prevent SQL injection

### Phase 1: Design & Contracts

**Data Model (data-model.md)**:
- 4 tables with UUID primary keys, foreign key constraints
- 2 ENUMs: hardware_type, personalization_level_type
- 9 indexes including GIN index for JSONB software_skills
- State transitions documented (signup → profile → preferences)
- Cache invalidation trigger on profile.updated_at change
- Alembic migration script provided

**API Contracts (contracts/)**:
- **auth.openapi.yaml**: Signup, signin, logout, session validation (4 endpoints)
- **profile.openapi.yaml**: Questionnaire submission, get/update profile (3 endpoints)
- **personalization.openapi.yaml**: Personalize, get cached, invalidate cache (3 endpoints)
- **translation.openapi.yaml**: Translate with quality metrics (1 endpoint)
- All schemas include examples, error responses (400, 401, 404, 422, 504)
- Session-based authentication via HTTP-only cookies

**Quickstart Guide (quickstart.md)**:
- Prerequisites: Python 3.11+, Node 18+, Redis 7.x, Neon Postgres
- Backend setup: venv, requirements.txt, .env, migrations, uvicorn
- Frontend setup: npm install, .env.local, npm start
- Verification steps: curl commands for all API flows
- Troubleshooting: Database connection, Redis, OpenRouter rate limits

### Constitution Check Results

✅ **PASS** with 2 WATCH items:

1. **LLM Non-Determinism**: AI-generated personalization/translation is non-deterministic
   - **Mitigation**: Redis caching ensures same user sees consistent content within 30-min window
   - **Justification**: Core feature requirement; non-determinism is acceptable with caching

2. **Translation Quality Validation**: 95% term preservation requires empirical testing
   - **Mitigation**: Automated regex validation + manual spot checks
   - **Phase 2**: Comprehensive quality tests with sample chapters

### Project Structure

**Web application** (FastAPI backend + Docusaurus frontend):

```
backend/src/
├── models/ (users, profiles, preferences - SQLAlchemy)
├── services/ (auth, profile, personalization, translation)
├── api/ (FastAPI routes for 4 API groups)
├── skills/ (4 reusable AI skills - framework-agnostic)
├── cache/ (Redis abstraction with TTL)

frontend/src/
├── components/Auth/ (SignupForm, SigninForm, ProfileSettings, AuthContext)
├── components/Chapter/ (PersonalizeButton, TranslateButton, ContentToggle)
├── theme/ (MDXContent swizzling for button injection)
├── services/ (API clients for auth, profile, personalization, translation)
```

## Outcome

- ✅ Impact: Comprehensive implementation plan ready for /sp.tasks
- 🧪 Tests: Constitution Check passed, all research decisions documented
- 📁 Files: plan.md (1 file), research.md (1 file), data-model.md (1 file), contracts/ (4 OpenAPI specs), quickstart.md (1 file)
- 🔁 Next prompts: /sp.tasks (generate task breakdown for implementation)
- 🧠 Reflection: Phase 0 research resolved all technical unknowns (Better Auth → FastAPI-Users critical decision). Phase 1 design provides complete data model, API contracts, and developer onboarding. Ready for task breakdown and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: Better Auth Python integration not available (pivoted to FastAPI-Users)
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (2 WATCH items justified)
  - Research completeness: PASS (all 7 areas resolved with alternatives considered)
  - Data model normalization: PASS (3NF with JSONB flexibility)
  - API contract completeness: PASS (4 OpenAPI 3.1 specs with all CRUD operations)
  - Developer onboarding: PASS (quickstart.md with verification steps and troubleshooting)
- Prompt variant (if applicable): Standard /sp.plan workflow (Phase 0 → Phase 1)
- Next experiment (smallest change to try): /sp.tasks to generate granular implementation tasks with acceptance criteria
