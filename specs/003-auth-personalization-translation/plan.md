# Implementation Plan: Authentication, Personalization, and Translation Integration

**Branch**: `003-auth-personalization-translation` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-auth-personalization-translation/spec.md`

## Summary

Extend the existing Physical AI & Humanoid Robotics Docusaurus platform with user authentication, personalized content rendering based on user skill profiles, and Urdu translation capabilities. The system integrates Better Auth for authentication, Neon Serverless Postgres for user profile storage, OpenRouter free LLM models for content transformations, and enhances the existing RAG chatbot to respect personalized/translated context sent from the frontend.

**Technical Approach**:
- **Authentication**: Better Auth library integrated into Docusaurus React components, JWT tokens validated by FastAPI backend
- **Personalization**: Frontend-initiated LLM transformation using user profile data, stored in browser session
- **Translation**: Frontend-initiated Urdu translation with Focus Mode (technical faithfulness), stored in browser session
- **RAG Integration**: Frontend sends transformed content chunks with each chatbot query (stateless chatbot approach)

## Technical Context

**Language/Version**:
- **Frontend**: TypeScript 5.x + React 18 (Docusaurus 3.x)
- **Backend**: Python 3.11+ (FastAPI)

**Primary Dependencies**:
- **Frontend**: Better Auth (auth library), Docusaurus 3.x, React 18, TypeScript
- **Backend**: FastAPI, SQLAlchemy, Alembic (migrations), python-jose (JWT), passlib (password hashing), Neon Serverless Postgres driver
- **LLM**: OpenRouter API (free tier models)
- **Existing**: RAG orchestrator (backend/src/services/rag_orchestrator.py)

**Storage**:
- **User Profiles**: Neon Serverless Postgres (persistent storage via SQLAlchemy ORM)
- **Personalized/Translated Content**: Browser session storage (IndexedDB or SessionStorage, no backend persistence)
- **Authentication**: JWT tokens (Better Auth issues, FastAPI validates)

**Testing**:
- **Frontend**: Jest + React Testing Library
- **Backend**: pytest (existing test structure in backend/tests/)
- **Integration**: End-to-end tests for auth flow, personalization, translation

**Target Platform**:
- **Frontend**: Modern browsers (Chrome, Firefox, Safari, Edge - latest 2 versions)
- **Backend**: Linux server (Docker container deployment)
- **Database**: Neon Serverless Postgres (cloud-hosted)

**Project Type**: Web application (separate frontend/backend)

**Performance Goals**:
- User registration completes in under 3 minutes (SC-001)
- Personalized chapter generation completes in under 10 seconds for chapters up to 5000 words (SC-002)
- Chapter translation to Urdu completes in under 15 seconds for chapters up to 5000 words (SC-003)
- RAG chatbot responds within 3 seconds when using personalized or translated context (SC-006)
- Authentication success rate exceeds 99% for valid credentials (SC-010)

**Constraints**:
- Session-only storage for personalized/translated content (regenerate on new session)
- OpenRouter free tier rate limits (exact limits to be researched in Phase 0)
- Cost per personalization/translation request remains under $0.01 (SC-012)
- JWT token validation required on all protected API endpoints
- Frontend sends transformed content chunks with each RAG chatbot query (stateless)
- Only one content transformation (personalization OR translation) active at a time per chapter

**Scale/Scope**:
- Expected users: 100 concurrent users without degradation (SC-009)
- Database: Single Neon Postgres instance with connection pooling
- Content transformation: Per-chapter basis (not entire book)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Mandatory Principles Compliance

✅ **100% Spec-Driven Development**: This plan follows `/sp.specify` → `/sp.plan` → `/sp.tasks` workflow. All features match approved specification exactly.

✅ **Technical Accuracy Only**: All authentication patterns, JWT flows, and database schemas will be verified against industry standards and documented with citations.

✅ **Deterministic, Reproducible Output**: All dependencies version-pinned (package.json, requirements.txt), database migrations versioned with Alembic, build processes documented.

✅ **Single Source of Truth (SSOT)**:
- Specifications: `specs/003-auth-personalization-translation/`
- Frontend code: `src/components/`, `src/api/`, `src/hooks/`
- Backend code: `backend/src/api/routes/`, `backend/src/db/models/`
- Migrations: `backend/src/db/migrations/versions/`

✅ **Runnable, Version-Pinned Code Only**: All code will include version constraints, validation tests, and safety notes for API key handling.

✅ **Zero Hallucinations**: All technical claims (JWT validation, OAuth patterns, database schema design) will be supported by official documentation citations (Better Auth docs, FastAPI docs, JWT.io, Neon docs).

### Book Structure Compliance

⚠️ **Not Applicable**: This is an infrastructure feature (authentication, personalization platform), not a book chapter. Does not impact 6-module, 21-chapter structure.

### Documentation & Site Rules

✅ **Docusaurus Page Requirements**: New components will integrate with existing Docusaurus theme without adding new pages. No impact on metadata, link integrity, or media optimization.

✅ **Performance**: Session storage and JWT validation add minimal latency (<100ms). OpenRouter API calls handled asynchronously with loading indicators.

✅ **Build Quality**: No Docusaurus build warnings or errors expected. TypeScript strict mode enforced.

### Workflow & Artifacts Compliance

✅ **Mandatory Workflow**: Currently in `/sp.plan` phase. Next: `/sp.tasks` for granular task breakdown.

✅ **Required Artifacts**:
- `specs/003-auth-personalization-translation/spec.md` ✅ (created)
- `specs/003-auth-personalization-translation/plan.md` ✅ (this file)
- `specs/003-auth-personalization-translation/research.md` ⏳ (Phase 0 output)
- `specs/003-auth-personalization-translation/data-model.md` ⏳ (Phase 1 output)
- `specs/003-auth-personalization-translation/quickstart.md` ⏳ (Phase 1 output)
- `specs/003-auth-personalization-translation/contracts/` ⏳ (Phase 1 output)
- `specs/003-auth-personalization-translation/tasks.md` ⏳ (`/sp.tasks` command)

✅ **Code Example Requirements**: All authentication flows, API endpoints, and frontend components will include validation tests and inline comments explaining security reasoning.

### ADR Policy

📋 **ADR Trigger Test** (run after Phase 1):

**Potential architecturally significant decisions**:
1. **Better Auth vs custom auth implementation** (framework choice)
2. **Session-only vs database-persisted personalized content** (storage architecture)
3. **Stateless RAG chatbot with frontend-sent context vs backend context management** (integration pattern)
4. **OpenRouter vs self-hosted LLM for personalization/translation** (cost/performance tradeoff)

**Evaluation (after Phase 1)**:
- All decisions have long-term structural consequences? ✅
- Multiple viable options considered? ✅
- Cross-cutting influence on system design? ✅

**Recommendation**: Create ADRs for decisions #1, #2, #3 (decision #4 specified by requirements, not a choice).

### Quality Gates

All gates MUST pass before deployment:

1. ✅ **Docusaurus Build**: `npm run build` = PASS (no errors)
2. ✅ **Broken Links**: Link checker = PASS
3. ✅ **Spell Check**: Spell check = PASS
4. ✅ **Accessibility**: WCAG AA compliance = PASS (auth forms must be keyboard-accessible, screen-reader compatible)
5. ✅ **Technical Accuracy**: JWT validation, password hashing, database schema reviewed against security best practices
6. ✅ **Code Tests**: All API endpoints, auth flows, personalization/translation tested = PASS
7. ✅ **Performance**: Lighthouse score ≥ 90 (auth flows must not degrade page performance)
8. ✅ **Completeness**: No TODO, TBD, or placeholder content

**Constitution Check Status**: ✅ **PASS** - Proceed to Phase 0 research

## Project Structure

### Documentation (this feature)

```text
specs/003-auth-personalization-translation/
├── spec.md              # Feature specification (✅ created)
├── plan.md              # This file (✅ created)
├── research.md          # Phase 0 output (research findings)
├── data-model.md        # Phase 1 output (database schema, entities)
├── quickstart.md        # Phase 1 output (setup instructions)
├── contracts/           # Phase 1 output (API contracts)
│   ├── auth.openapi.yaml
│   ├── personalization.openapi.yaml
│   ├── translation.openapi.yaml
│   └── profile.openapi.yaml
├── checklists/
│   └── requirements.md  # Spec quality checklist (✅ created)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

**Structure Decision**: Web application with separate frontend (Docusaurus) and backend (FastAPI). This matches the existing project structure with `src/` for frontend and `backend/` for backend services.

```text
# Frontend (Docusaurus React components)
src/
├── components/
│   ├── Auth/
│   │   ├── SignupForm.tsx           # User registration form
│   │   ├── SigninForm.tsx           # User login form
│   │   ├── AuthProvider.tsx         # Better Auth context provider
│   │   └── ProtectedRoute.tsx       # Route guard component
│   ├── ChapterActions/
│   │   ├── PersonalizeButton.tsx    # "Personalize Chapter" button
│   │   ├── TranslateButton.tsx      # "Translate to Urdu" button
│   │   └── ContentTransformer.tsx   # Handles transformation UI/logic
│   └── RAGChatbot/                  # Existing RAG chatbot component
│       └── ChatInterface.tsx        # (modify to send transformed content)
├── hooks/
│   ├── useAuth.ts                   # Auth state management hook
│   ├── usePersonalization.ts        # Personalization state/logic hook
│   └── useTranslation.ts            # Translation state/logic hook
├── api/
│   ├── auth.ts                      # Auth API client (Better Auth integration)
│   ├── personalization.ts           # Personalization API client
│   ├── translation.ts               # Translation API client
│   └── rag.ts                       # RAG API client (modify to send context)
├── types/
│   ├── auth.ts                      # Auth types (User, UserProfile, Token)
│   ├── personalization.ts           # Personalization types
│   └── translation.ts               # Translation types
├── theme/
│   └── Root.tsx                     # Docusaurus root wrapper (inject AuthProvider)
└── plugins/
    └── sessionStorage.ts            # Session storage utility for transformed content

# Backend (FastAPI)
backend/
├── src/
│   ├── api/
│   │   ├── routes/
│   │   │   ├── auth.py              # Auth endpoints (signup, signin, token validation)
│   │   │   ├── profile.py           # User profile endpoints (get, update)
│   │   │   ├── personalization.py   # Personalization endpoint (/personalize)
│   │   │   ├── translation.py       # Translation endpoint (/translate)
│   │   │   └── chat.py              # (modify to validate JWT, accept context)
│   │   ├── middleware/
│   │   │   ├── auth.py              # JWT validation middleware
│   │   │   └── rate_limit.py        # (existing, may extend for auth)
│   │   └── models/
│   │       ├── request.py           # (extend with auth, personalization, translation)
│   │       └── response.py          # (extend with auth, personalization, translation)
│   ├── db/
│   │   ├── models.py                # (extend with User, UserProfile models)
│   │   ├── connection.py            # (existing Neon Postgres connection)
│   │   └── migrations/
│   │       └── versions/
│   │           └── YYYYMMDD_HHMM_add_user_auth_tables.py
│   ├── services/
│   │   ├── auth_service.py          # Auth business logic (password hashing, JWT)
│   │   ├── profile_service.py       # User profile business logic
│   │   ├── personalization_service.py # Personalization LLM orchestration
│   │   ├── translation_service.py   # Translation LLM orchestration
│   │   └── rag_orchestrator.py      # (existing, modify to accept context)
│   ├── utils/
│   │   ├── jwt.py                   # JWT utilities (encode, decode, verify)
│   │   ├── password.py              # Password hashing utilities
│   │   └── openrouter.py            # OpenRouter API client
│   └── config.py                    # (extend with auth, OpenRouter config)
└── tests/
    ├── api/
    │   ├── test_auth.py
    │   ├── test_profile.py
    │   ├── test_personalization.py
    │   └── test_translation.py
    ├── services/
    │   ├── test_auth_service.py
    │   ├── test_personalization_service.py
    │   └── test_translation_service.py
    └── integration/
        └── test_auth_flow.py

# Database Migrations
backend/src/db/migrations/versions/
└── YYYYMMDD_HHMM_add_user_auth_tables.py  # Alembic migration
```

## Complexity Tracking

> **No violations detected** - Constitution Check passed all gates. No complexity justifications required.

---

**Phase 0 (Research) and Phase 1 (Design) outputs will follow in subsequent sections per `/sp.plan` command workflow.**
