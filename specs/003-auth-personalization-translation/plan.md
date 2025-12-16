# Implementation Plan: Auth, Personalization & Translation Integration

**Branch**: `003-auth-personalization-translation` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-auth-personalization-translation/spec.md`

## Summary

Extend the existing RAG-enabled Docusaurus textbook platform with authentication, content personalization, and Urdu translation capabilities. Users create accounts with background profiles (software skills, hardware access), then can personalize chapter content to their skill level or translate chapters to Urdu. All features use reusable AI skills and integrate with existing FastAPI backend without breaking the RAG chatbot.

**Technical Approach**: Implement Better Auth for session-based authentication, store user profiles in Neon Postgres (JSONB for skills, ENUM for hardware), develop four reusable AI skills (user-profile, personalization, translation, content-adapter), cache personalized content in Redis (30-min TTL with profile hash), and add React components to Docusaurus theme for auth UI and chapter buttons.

## Technical Context

**Language/Version**:
- Backend: Python 3.11+ (FastAPI)
- Frontend: TypeScript 5.x + React 18.x (Docusaurus 3.x)
- Database: PostgreSQL 15+ (Neon Serverless)

**Primary Dependencies**:
- Backend: FastAPI 0.104+, Better Auth (Python adapter), psycopg3, redis-py, openrouter-sdk
- Frontend: Docusaurus 3.x, @docusaurus/theme-classic, React 18.x
- AI: OpenRouter API (DeepSeek free model)
- Database: Neon Serverless Postgres
- Cache: Redis 7.x (shared with RAG chatbot)

**Storage**:
- Neon Postgres tables: users, profiles, preferences, translation_logs (optional)
- Redis cache: Personalized content with 30-min TTL
- File system: Chapter markdown files (unchanged)

**Testing**:
- Backend: pytest with fixtures for DB and Redis
- Frontend: Jest + React Testing Library
- Integration: Playwright for E2E auth flows
- Contract: OpenAPI schema validation

**Target Platform**:
- Backend: Linux server (deployment), Windows/macOS (development)
- Frontend: Web browsers (Chrome, Firefox, Safari, Edge)
- Database: Neon Serverless (cloud Postgres)

**Project Type**: Web application (FastAPI backend + Docusaurus frontend)

**Performance Goals**:
- Personalization: <10 seconds (FR-020), <100ms cached
- Translation: <15 seconds (FR-030)
- Auth endpoints: <200ms p95
- Cache hit rate: >80% for personalization
- Concurrent users: 500 without degradation (SC-005)

**Constraints**:
- Must not break existing RAG chatbot functionality (SC-009)
- Skills must be framework-agnostic and reusable (FR-042)
- Translation never persists to database (FR-029)
- Buttons hidden for anonymous users (FR-015, FR-023)
- Redis shared with RAG chatbot (namespace isolation)

**Scale/Scope**:
- Users: ~10,000 in first year (A-002)
- Chapters: All existing textbook chapters (~21 chapters per constitution)
- Personalization cache: ~1,000 cached variations at peak
- Translation: On-demand only (no pre-generation)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### 1. 100% Spec-Driven Development
- ✅ PASS: Feature has complete spec.md with 48 FRs and 7 clarifications
- ✅ PASS: This plan.md follows `/sp.specify` → `/sp.plan` workflow
- ✅ PASS: All requirements traceable to spec (FR-001 to FR-048)

### 2. Technical Accuracy Only
- ✅ PASS: Authentication uses industry-standard Better Auth library
- ✅ PASS: Caching strategy based on proven Redis patterns (TTL + hash invalidation)
- ✅ PASS: Database design follows normalization (users, profiles, preferences)
- ⚠️ WATCH: Translation quality (95% term preservation) requires empirical validation in Phase 2

### 3. Deterministic, Reproducible Output
- ✅ PASS: Better Auth provides deterministic session management
- ✅ PASS: Redis caching is deterministic (same profile hash → same cache key)
- ⚠️ WATCH: LLM outputs (personalization, translation) are non-deterministic by nature
  - **Mitigation**: Cache results to ensure same user sees consistent content within 30-min window
  - **Justification**: AI-generated personalization is core feature value; caching mitigates non-determinism

### 4. Single Source of Truth (SSOT)
- ✅ PASS: User profiles in Neon Postgres (profiles table)
- ✅ PASS: Chapter markdown files remain canonical source
- ✅ PASS: Personalized content cached but never overwrites originals
- ✅ PASS: Translation is temporary (session-only, no persistence)

### 5. Runnable, Version-Pinned Code Only
- ✅ PASS: Dependencies will be pinned in requirements.txt and package.json
- ✅ PASS: Code examples will include version constraints
- ✅ PASS: No hardware safety concerns (web application only)

### 6. Zero Hallucinations
- ✅ PASS: All technical decisions cited from spec.md and clarifications
- ✅ PASS: Better Auth, FastAPI, Redis patterns are documented standards
- ⚠️ WATCH: LLM-generated personalization/translation content requires quality gates
  - **Mitigation**: FR-048 defines measurable quality metrics (95% term preservation, 100% code preservation)

**Constitution Compliance**: ✅ PASS with 2 WATCH items (LLM non-determinism and quality validation)

**Justifications**:
- **LLM Non-Determinism**: Core feature requirement (personalization and translation). Mitigated with caching and quality metrics.
- **Translation Quality Validation**: Requires empirical testing with sample chapters. Phase 2 will include automated tests for term preservation.

## Project Structure

### Documentation (this feature)

```text
specs/003-auth-personalization-translation/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (completed)
├── clarifications.md    # [REMOVED] - integrated into spec.md
├── research.md          # Phase 0 output (to be created)
├── data-model.md        # Phase 1 output (to be created)
├── quickstart.md        # Phase 1 output (to be created)
├── contracts/           # Phase 1 output (to be created)
│   ├── auth.openapi.yaml
│   ├── profile.openapi.yaml
│   ├── personalization.openapi.yaml
│   └── translation.openapi.yaml
├── checklists/
│   └── requirements.md  # Quality validation (completed)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (FastAPI backend + Docusaurus frontend)

backend/                      # FastAPI application (NEW)
├── src/
│   ├── models/              # Database models (SQLAlchemy/psycopg3)
│   │   ├── user.py         # User authentication model
│   │   ├── profile.py      # User profile (software/hardware background)
│   │   └── preference.py   # Personalization preferences
│   ├── services/           # Business logic layer
│   │   ├── auth_service.py       # Better Auth integration
│   │   ├── profile_service.py    # Profile CRUD operations
│   │   ├── personalization_service.py  # Personalization orchestration
│   │   └── translation_service.py      # Translation orchestration
│   ├── api/                # FastAPI routes
│   │   ├── auth_routes.py        # /auth/signup, /auth/signin, /auth/logout
│   │   ├── profile_routes.py     # /user/profile (GET/PUT)
│   │   ├── personalization_routes.py  # /content/personalize
│   │   └── translation_routes.py      # /content/translate
│   ├── skills/             # Reusable AI skills (NEW)
│   │   ├── user_profile_skill.py      # Read/write user profile context
│   │   ├── personalization_skill.py   # LLM-based chapter personalization
│   │   ├── translation_skill.py       # LLM-based Urdu translation
│   │   └── content_adapter_skill.py   # Format and sanitize LLM outputs
│   ├── cache/              # Redis cache abstraction
│   │   └── personalization_cache.py  # Cache layer with TTL and invalidation
│   ├── config.py           # Environment configuration
│   └── main.py             # FastAPI app entry point
├── tests/
│   ├── unit/              # Unit tests for services and skills
│   ├── integration/       # Integration tests for DB and Redis
│   └── contract/          # OpenAPI contract validation
├── alembic/               # Database migrations
│   └── versions/          # Migration scripts
├── requirements.txt       # Python dependencies (pinned)
└── .env.example           # Environment variables template

frontend/                  # Docusaurus site (EXISTING - EXTEND)
├── src/
│   ├── components/
│   │   ├── Auth/          # Authentication UI components (NEW)
│   │   │   ├── SignupForm.tsx      # Signup with questionnaire
│   │   │   ├── SigninForm.tsx      # Signin form
│   │   │   ├── ProfileSettings.tsx # Profile update page
│   │   │   └── AuthContext.tsx     # React context for auth state
│   │   ├── Chapter/       # Chapter enhancements (NEW)
│   │   │   ├── PersonalizeButton.tsx    # "Personalize Content" button
│   │   │   ├── TranslateButton.tsx      # "Translate to Urdu" button
│   │   │   ├── ContentToggle.tsx        # "Show Original" toggle
│   │   │   └── ChapterWrapper.tsx       # Wraps chapter with buttons
│   │   └── RAGChatbot/    # Existing RAG chatbot (UNCHANGED)
│   │       └── ...
│   ├── theme/             # Docusaurus theme customization (EXTEND)
│   │   └── Root.tsx       # Add AuthProvider wrapper (EXISTING)
│   └── services/          # Frontend API clients (NEW)
│       ├── authService.ts           # Auth API calls
│       ├── profileService.ts        # Profile API calls
│       ├── personalizationService.ts # Personalization API calls
│       └── translationService.ts     # Translation API calls
├── tests/
│   └── e2e/               # Playwright E2E tests (NEW)
│       ├── auth.spec.ts          # Signup/signin flows
│       ├── personalization.spec.ts # Personalize chapter flow
│       └── translation.spec.ts    # Translate chapter flow
├── package.json           # Node dependencies (pinned)
└── .env.example           # Frontend environment variables

.claude/skills/            # Claude Code skills metadata (NEW)
├── user-profile-skill/
│   └── skill.yaml
├── personalization-skill/
│   └── skill.yaml
├── translation-skill/
│   └── skill.yaml
└── content-adapter-skill/
    └── skill.yaml

history/prompts/003-auth-personalization-translation/  # Prompt History Records
└── [PHR files for this feature]
```

**Structure Decision**: Web application with separate backend and frontend directories. Backend is new (FastAPI), frontend extends existing Docusaurus. Skills are framework-agnostic and stored in `.claude/skills/` for reusability. RAG chatbot components remain unchanged to satisfy SC-009 (zero breaking changes).

## Complexity Tracking

> **No violations requiring justification**

All complexity is justified by functional requirements:
- Better Auth library required for FR-001 to FR-007 (authentication)
- Redis caching required for FR-044 (30-min TTL performance)
- JSONB storage required for FR-011 (flexible multi-select software skills)
- Four AI skills required for FR-036 to FR-039 (reusable, framework-agnostic design)

## Phase 0: Research & Architecture Decisions

**Status**: To be executed
**Output**: `research.md`

### Research Tasks

1. **Better Auth Python Integration**
   - Research: How to integrate Better Auth with FastAPI (session-based, not JWT)
   - Decision needed: Python adapter library or custom integration
   - Alternatives: FastAPI-Users, Authlib, custom session management
   - Research output: Installation steps, session storage, middleware configuration

2. **Neon Postgres Schema Design**
   - Research: Best practices for user profile JSONB schemas
   - Research: ENUM vs. string types for hardware access field
   - Research: Foreign key constraints and cascade delete policies
   - Decision needed: Normalization level (3NF vs. denormalization for performance)
   - Research output: Complete DDL with indexes and constraints

3. **Redis Caching Strategy**
   - Research: Redis key naming conventions for multi-tenant caching
   - Research: Cache invalidation patterns (TTL vs. manual invalidation)
   - Research: Profile hash algorithms (MD5 vs. SHA256 vs. custom)
   - Decision needed: Namespace isolation from RAG chatbot cache
   - Research output: Cache key format, TTL values, invalidation logic

4. **OpenRouter DeepSeek Integration**
   - Research: DeepSeek free model capabilities (context window, Urdu support)
   - Research: Rate limits and token budgets for free tier
   - Research: Prompt engineering for technical term preservation (95% target)
   - Decision needed: Fallback model if DeepSeek unavailable
   - Research output: API configuration, prompt templates, error handling

5. **Docusaurus Theme Customization**
   - Research: How to inject React components at chapter start (before content)
   - Research: Theme swizzling vs. plugin approach
   - Research: Session storage vs. cookies for view preferences
   - Decision needed: Component lifecycle and state management
   - Research output: Implementation pattern, code examples

6. **Urdu Translation Quality Assurance**
   - Research: Automated testing for technical term preservation
   - Research: Regex patterns for detecting English terms in Urdu text
   - Research: Hybrid format validation ("term (اردو)...")
   - Decision needed: Manual review workflow or automated gates
   - Research output: Quality validation script, acceptance criteria

7. **Security and Session Management**
   - Research: CSRF protection for FastAPI endpoints
   - Research: Session timeout and refresh token strategies
   - Research: SQL injection prevention with psycopg3 parameterized queries
   - Decision needed: Session storage (Redis vs. database vs. signed cookies)
   - Research output: Security checklist, middleware configuration

**Research Deliverable**: `research.md` with all decisions documented, alternatives evaluated, and implementation guidance provided.

## Phase 1: Design & Contracts

**Status**: To be executed after Phase 0
**Outputs**: `data-model.md`, `contracts/`, `quickstart.md`

### 1. Data Model Design (`data-model.md`)

**Source**: Entities from spec.md (User, Profile, Preference, TranslationLog)

**Output**:
- Entity-relationship diagrams
- Table schemas with DDL
- Field validation rules
- Indexes and constraints
- State transitions (user signup → profile creation flow)

### 2. API Contracts (`contracts/`)

**Method**: Extract from functional requirements FR-001 to FR-043

**Endpoints to define**:

**Authentication** (auth.openapi.yaml):
- `POST /auth/signup` - FR-001, FR-002
- `POST /auth/signin` - FR-003
- `POST /auth/logout` - FR-007
- `GET /auth/session` - Session validation

**Profile Management** (profile.openapi.yaml):
- `GET /user/profile` - FR-013
- `PUT /user/profile` - FR-013
- `POST /user/questionnaire` - FR-008, FR-009, FR-010

**Content Personalization** (personalization.openapi.yaml):
- `POST /content/personalize` - FR-014, FR-016, FR-017
- `GET /content/personalize/{userId}/{chapterId}` - Cached retrieval
- `DELETE /content/personalize/cache/{userId}` - Cache invalidation (FR-047)

**Content Translation** (translation.openapi.yaml):
- `POST /content/translate` - FR-022, FR-024, FR-025
- Note: No GET endpoint (translation never persists per FR-029)

**Contract format**: OpenAPI 3.1 with:
- Request/response schemas
- Error responses (401, 403, 404, 422, 500)
- Security requirements (session cookies)
- Rate limits and timeouts

### 3. Quickstart Guide (`quickstart.md`)

**Audience**: Developers setting up local environment

**Contents**:
1. Prerequisites (Python 3.11+, Node 18+, Redis, Neon Postgres credentials)
2. Backend setup (install deps, configure .env, run migrations, start server)
3. Frontend setup (install deps, configure .env, start dev server)
4. Verification steps (auth flow, personalization, translation)
5. Troubleshooting (common errors, database connection issues)

### 4. Agent Context Update

**Action**: Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`

**Purpose**:
- Update `.claude/` configuration with new technologies
- Add Better Auth, Neon Postgres, Redis to known dependencies
- Preserve manual additions between markers

**Verification**: Check `.claude/settings.local.json` or equivalent

## Phase 2: Task Breakdown

**Status**: Deferred to `/sp.tasks` command
**Output**: `tasks.md`

**Scope**: `/sp.plan` ends here. User will run `/sp.tasks` to generate implementation tasks.

---

## Next Steps

1. ✅ Complete Phase 0: Generate `research.md` (resolve all NEEDS CLARIFICATION items)
2. ✅ Complete Phase 1: Generate `data-model.md`, `contracts/`, `quickstart.md`
3. ⏭️ Run `/sp.tasks` to generate `tasks.md` for implementation
4. ⏭️ Execute tasks incrementally (TDD: red → green → refactor)
5. ⏭️ Validate against Success Criteria (SC-001 to SC-010)

**Branch**: `003-auth-personalization-translation`
**Spec**: [spec.md](./spec.md)
**Blockers**: None - ready for Phase 0 research
