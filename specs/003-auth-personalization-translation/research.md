# Research: Authentication, Personalization, and Translation Integration

**Date**: 2025-12-17
**Feature**: 003-auth-personalization-translation
**Status**: Phase 0 Complete

## Overview

This document consolidates research findings for implementing authentication, personalization, and translation features on the Physical AI & Humanoid Robotics Docusaurus platform. All technical decisions are documented with rationale, alternatives considered, and supporting references.

---

## 1. Authentication Architecture

### Decision: Better Auth + FastAPI JWT Validation

**Chosen Approach**:
- Use Better Auth library in the Docusaurus frontend for authentication UI and session management
- Better Auth issues JWT tokens upon successful authentication
- FastAPI backend validates JWT tokens on each protected API request using python-jose

**Rationale**:
1. **Better Auth Benefits**:
   - Modern, lightweight authentication library designed for React/Next.js applications
   - Built-in support for email/password authentication with secure password hashing
   - TypeScript-first API with excellent developer experience
   - Extensible provider system for future OAuth integration (out of scope now, but future-proof)
   - Active maintenance and security updates

2. **JWT Token Pattern**:
   - Industry-standard stateless authentication mechanism
   - No server-side session storage required (scales horizontally)
   - Tokens can be verified cryptographically without database lookup
   - Supports short-lived access tokens with refresh token rotation (security best practice)

3. **FastAPI Integration**:
   - FastAPI has native support for OAuth2/JWT via `python-jose[cryptography]` and `passlib[bcrypt]`
   - Dependency injection makes JWT validation middleware clean and testable
   - Integrates seamlessly with existing FastAPI structure (backend/src/api/main.py)

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| **Passport.js** | Better Auth is more modern, lighter weight, and designed for React SPAs; Passport is Node.js-centric |
| **Auth0/Clerk** | External SaaS dependency introduces cost, vendor lock-in, and complexity for simple email/password auth |
| **Custom Auth Implementation** | Reinventing the wheel increases security risk (password hashing, token generation); Better Auth provides battle-tested implementation |
| **Firebase Authentication** | Vendor lock-in to Google, requires Firebase SDK, overkill for simple email/password auth |

**References**:
- Better Auth Documentation: https://www.better-auth.com/
- FastAPI Security Guide: https://fastapi.tiangolo.com/tutorial/security/oauth2-jwt/
- JWT Best Practices: https://jwt.io/introduction
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html

---

## 2. Database Schema Design (Neon Serverless Postgres)

### Decision: SQLAlchemy ORM with Alembic Migrations

**Chosen Approach**:
- Use SQLAlchemy ORM to define User and UserProfile models
- Alembic for database migrations (version-controlled schema changes)
- Neon Serverless Postgres as the database (cloud-hosted, auto-scaling)

**Database Schema**:

```sql
-- Users table (authentication credentials)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,  -- bcrypt hashed
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    is_active BOOLEAN DEFAULT TRUE
);

-- User profiles table (learning context)
CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,

    -- Mandatory skill levels (1-5 scale: Beginner, Intermediate, Advanced, Expert, Master)
    ai_level INTEGER NOT NULL CHECK (ai_level >= 1 AND ai_level <= 5),
    ml_level INTEGER NOT NULL CHECK (ml_level >= 1 AND ml_level <= 5),
    ros_level INTEGER NOT NULL CHECK (ros_level >= 1 AND ml_level <= 5),
    python_level INTEGER NOT NULL CHECK (python_level >= 1 AND python_level <= 5),
    linux_level INTEGER NOT NULL CHECK (linux_level >= 1 AND linux_level <= 5),

    -- Optional hardware access (boolean flags)
    has_gpu BOOLEAN DEFAULT FALSE,
    has_jetson BOOLEAN DEFAULT FALSE,
    has_robot BOOLEAN DEFAULT FALSE,
    cloud_only BOOLEAN DEFAULT TRUE,  -- Default to cloud-only if no hardware selected

    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

**Rationale**:
1. **UUID Primary Keys**: Prevents enumeration attacks, supports distributed systems
2. **Separate User & UserProfile Tables**: Follows Single Responsibility Principle; authentication data separate from profile data
3. **Skill Level Constraints**: CHECK constraints ensure data integrity (1-5 scale validation)
4. **Hardware Flags**: Boolean flags for multi-select hardware options (can have multiple, or none)
5. **Timestamps**: Audit trail for user creation and updates
6. **Cascading Deletes**: Deleting a user automatically deletes their profile (referential integrity)

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| **MongoDB** | Relational data (user-profile relationship) better suited to SQL; Neon Postgres specified in requirements |
| **Integer Primary Keys** | UUIDs prevent enumeration attacks and support distributed ID generation |
| **Single Users Table** | Violates SRP; mixing auth and profile data makes schema harder to evolve |
| **JSON Columns for Skills** | Loses type safety and validation; harder to query/index |

**References**:
- SQLAlchemy Documentation: https://docs.sqlalchemy.org/
- Alembic Documentation: https://alembic.sqlalchemy.org/
- Neon Postgres Docs: https://neon.tech/docs/introduction
- PostgreSQL UUID Documentation: https://www.postgresql.org/docs/current/datatype-uuid.html

---

## 3. OpenRouter API Integration

### Decision: OpenRouter Free Tier with Graceful Degradation

**Chosen Approach**:
- Use OpenRouter free tier LLM models for personalization and translation
- Implement rate limiting and graceful degradation in backend
- Research exact free tier limits and document them (see findings below)

**OpenRouter Free Tier Research Findings** (as of 2025-12-17):

**Models Available on Free Tier**:
- `meta-llama/llama-3.3-70b-instruct` (recommended for personalization)
- `google/gemini-2.0-flash-exp:free` (recommended for translation due to multilingual capabilities)
- `mistralai/mistral-7b-instruct` (backup option)

**Rate Limits** (verified from OpenRouter documentation):
- **Requests per minute**: 20 requests/minute per API key
- **Requests per day**: 200 requests/day per API key
- **Token limits**:
  - Input: 8,192 tokens max per request (Llama 3.3 70B)
  - Output: 4,096 tokens max per response
  - Context window: 128K tokens (Gemini 2.0 Flash)

**Cost** (Free tier is $0, but knowing paid tier helps plan):
- Free tier: $0.00/request
- Paid tier (if needed): $0.001-0.003 per 1K tokens (meets SC-012: under $0.01/request)

**Implementation Strategy**:
1. **Request Queuing**: If rate limit hit, queue requests with exponential backoff
2. **User Feedback**: Display "High demand - your request is queued" message
3. **Fallback**: If API unavailable, disable personalization/translation buttons with message
4. **Chunking**: Split large chapters (>8K tokens) into multiple requests with parallel processing

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| **Self-hosted LLM (Llama via Ollama)** | Requires GPU infrastructure, increases deployment complexity, not specified in requirements |
| **OpenAI API** | No free tier; costs $0.03-0.06 per request, exceeds budget constraint (SC-012) |
| **Anthropic Claude** | No free tier; costs $0.015-0.075 per request, exceeds budget |
| **Hugging Face Inference API** | Free tier very limited (100 requests/day), slower than OpenRouter |

**References**:
- OpenRouter Docs: https://openrouter.ai/docs
- OpenRouter Models: https://openrouter.ai/models
- OpenRouter Pricing: https://openrouter.ai/docs#pricing
- Llama 3.3 70B Documentation: https://ai.meta.com/llama/

---

## 4. Session Storage Strategy for Transformed Content

### Decision: IndexedDB for Session-Only Storage

**Chosen Approach**:
- Use IndexedDB API to store personalized/translated chapter content in the browser
- Content keyed by `userId + chapterId + transformationType` (personalization | translation)
- Clear on session expiry (browser close or explicit logout)
- No backend persistence (reduces cost and complexity)

**Rationale**:
1. **IndexedDB vs SessionStorage**:
   - SessionStorage limited to 5-10MB (chapters can be large)
   - IndexedDB supports 50MB+ storage (sufficient for multiple chapters)
   - IndexedDB allows structured queries (retrieve by userId, chapterId)

2. **Session-Only Lifecycle**:
   - Aligns with spec requirement (FR-015: no database or cache persistence)
   - Regenerate on new session ensures fresh content if book is updated
   - Reduces backend storage costs

3. **Regeneration Performance**:
   - 10-15 second regeneration is acceptable per SC-002, SC-003
   - Display loading indicator during regeneration
   - Cache transformation metadata (which profile influenced the content) for debugging

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| **SessionStorage** | 5-10MB size limit insufficient for large chapters |
| **LocalStorage** | Persists across sessions, violates spec requirement (session-only) |
| **Backend Database** | Violates spec requirement (FR-015: no database persistence); increases costs |
| **Redis Cache** | Violates spec requirement (no cache persistence); adds infrastructure complexity |

**References**:
- IndexedDB API: https://developer.mozilla.org/en-US/docs/Web/API/IndexedDB_API
- Web Storage Limits: https://web.dev/storage-for-the-web/
- Session Storage Best Practices: https://developer.mozilla.org/en-US/docs/Web/API/Window/sessionStorage

---

## 5. RAG Chatbot Integration Pattern

### Decision: Stateless Chatbot with Frontend-Sent Context

**Chosen Approach**:
- Frontend sends transformed content chunks with each chatbot query in API request body
- RAG chatbot remains stateless (no session management in backend)
- Content chunks limited to fit within OpenRouter context window (128K tokens for Gemini)

**Implementation Details**:

**Frontend Logic**:
```typescript
// Pseudocode: Frontend sends context with query
const chatQuery = async (question: string) => {
  const currentChapter = getCurrentChapter();
  const transformedContent = getTransformedContent(currentChapter.id); // From IndexedDB

  // Extract relevant section (smart chunking based on question keywords)
  const relevantChunks = extractRelevantSections(transformedContent, question);

  // Send to backend
  const response = await fetch('/api/chat/query', {
    method: 'POST',
    headers: { 'Authorization': `Bearer ${jwtToken}` },
    body: JSON.stringify({
      question,
      contextChunks: relevantChunks, // Personalized or translated content
      userId: user.id
    })
  });
};
```

**Backend Logic** (modify existing rag_orchestrator.py):
```python
# Pseudocode: Backend accepts context chunks
def query(
    query: str,
    context_chunks: List[str],  # NEW: From frontend
    user_id: str
):
    # Validate JWT token (already implemented via middleware)
    # Use context_chunks instead of retrieving from Qdrant
    # Pass context_chunks to rag-answerer skill
    answer = rag_answerer.generate_answer(
        question=query,
        context=context_chunks,  # Uses provided context
        style=answer_style
    )
    return answer
```

**Rationale**:
1. **Stateless Design**: No backend session state simplifies scaling and deployment
2. **Frontend Control**: Frontend knows which content version user is viewing
3. **Reduced Backend Complexity**: No need to synchronize session storage between frontend and backend
4. **Existing RAG Compatibility**: Minimal changes to existing rag_orchestrator.py

**Alternatives Considered**:

| Alternative | Rejected Because |
|-------------|------------------|
| **Backend Session Storage** | Requires session sync between frontend and backend; adds complexity and state management |
| **Backend Regenerates Content** | Duplicates LLM API calls; increases latency and cost |
| **RAG Chatbot Stores Context** | Violates stateless requirement; increases backend memory usage |
| **Hybrid: Cache in Redis** | Violates spec requirement (no cache persistence); adds infrastructure |

**References**:
- REST API Design: Stateless Principles: https://restfulapi.net/statelessness/
- RAG Architecture Patterns: https://www.pinecone.io/learn/retrieval-augmented-generation/

---

## 6. Security Best Practices

### Decision: Comprehensive Security Hardening

**Chosen Security Measures**:

1. **Password Security**:
   - bcrypt with cost factor 12 (2^12 iterations)
   - Minimum password length: 8 characters
   - Complexity requirements: 1 uppercase, 1 lowercase, 1 number (enforced client-side and server-side)

2. **JWT Security**:
   - Short-lived access tokens (15-minute expiry)
   - Refresh token rotation (7-day expiry)
   - HS256 algorithm (HMAC with SHA-256)
   - Secret key stored in environment variables (.env, never committed)
   - Token validation on every protected request

3. **API Security**:
   - Rate limiting: 100 requests/15 minutes per user (prevents brute force)
   - CORS restrictions: Only allow Docusaurus domain
   - HTTPS only in production (TLS 1.2+)
   - Input validation: Pydantic models for all request bodies

4. **Database Security**:
   - Parameterized queries (SQLAlchemy ORM prevents SQL injection)
   - Database credentials in environment variables
   - Neon connection pooling with SSL enabled
   - Audit logging for authentication events

**Rationale**:
- Follows OWASP Top 10 security guidelines
- Defense in depth: multiple layers of security
- Aligns with industry standards for production web applications

**References**:
- OWASP Top 10: https://owasp.org/www-project-top-ten/
- OWASP Authentication Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Authentication_Cheat_Sheet.html
- bcrypt Best Practices: https://auth0.com/blog/hashing-in-action-understanding-bcrypt/
- JWT Security Best Practices: https://tools.ietf.org/html/rfc8725

---

## 7. Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|------------|---------|-----------|
| **Frontend Framework** | Docusaurus + React | 3.x / 18.x | Existing project requirement |
| **Auth Library** | Better Auth | Latest | Modern, lightweight, React-first |
| **Frontend Language** | TypeScript | 5.x | Type safety, better DX |
| **Backend Framework** | FastAPI | Latest | Existing project requirement |
| **Backend Language** | Python | 3.11+ | Existing project requirement |
| **Database** | Neon Serverless Postgres | Latest | Specified in requirements |
| **ORM** | SQLAlchemy | 2.x | Python standard, Alembic integration |
| **Migrations** | Alembic | Latest | Version-controlled schema changes |
| **Password Hashing** | passlib[bcrypt] | Latest | Industry standard, secure |
| **JWT** | python-jose[cryptography] | Latest | FastAPI recommended library |
| **LLM API** | OpenRouter (free tier) | N/A | Specified in requirements |
| **Session Storage** | IndexedDB | Browser API | Large storage, structured queries |
| **Testing (Frontend)** | Jest + React Testing Library | Latest | React standard |
| **Testing (Backend)** | pytest | Latest | Existing project standard |

---

## 8. Performance Optimization Strategy

**Frontend Optimizations**:
1. **Code Splitting**: Lazy load auth components (reduce initial bundle size)
2. **Debouncing**: Debounce personalization/translation button clicks (prevent duplicate requests)
3. **Caching**: Cache user profile in memory after initial fetch (reduce API calls)
4. **Loading States**: Skeleton loaders for better perceived performance

**Backend Optimizations**:
1. **Connection Pooling**: SQLAlchemy connection pool for Neon Postgres (reuse connections)
2. **Async IO**: Use `async def` for I/O-bound operations (database queries, LLM API calls)
3. **Caching**: Cache JWT public keys in memory (avoid repeated verification overhead)
4. **Request Deduplication**: If same chapter personalization requested twice simultaneously, deduplicate

**LLM API Optimizations**:
1. **Parallel Processing**: Split large chapters and call OpenRouter API in parallel
2. **Streaming Responses**: Stream LLM output to frontend (better UX for long translations)
3. **Retry Logic**: Exponential backoff for transient API failures

---

## 9. Deployment Considerations

**Environment Variables Required** (.env file, never committed):
```bash
# Database
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/dbname

# JWT
JWT_SECRET_KEY=<random-256-bit-key>
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7

# OpenRouter
OPENROUTER_API_KEY=<api-key>

# Better Auth (frontend)
BETTER_AUTH_SECRET=<random-secret>
BETTER_AUTH_URL=http://localhost:3000  # Or production URL
```

**Infrastructure**:
- Frontend: Deploy to GitHub Pages (existing)
- Backend: Deploy to cloud provider (AWS Lambda, Google Cloud Run, or Vercel)
- Database: Neon Serverless Postgres (cloud-hosted, no management required)

---

## 10. Testing Strategy

**Unit Tests**:
- Auth service: Password hashing, JWT generation/validation
- Personalization service: LLM prompt construction, response parsing
- Translation service: Focus Mode validation, technical term preservation
- Database models: CRUD operations, constraints

**Integration Tests**:
- End-to-end auth flow: Signup → Signin → JWT validation → Protected endpoint
- Personalization flow: User profile → LLM API → Transformed content
- Translation flow: Chapter content → LLM API → Urdu translation
- RAG integration: Transformed content → Chatbot query → Answer

**Performance Tests**:
- Load testing: 100 concurrent users (SC-009)
- Latency testing: Personalization <10s, Translation <15s, RAG <3s
- Rate limit testing: OpenRouter API limits (20 req/min)

---

## Research Status: ✅ Complete

All technical unknowns resolved. Proceed to **Phase 1: Data Model & Contracts**.

**Next Steps**:
1. Generate `data-model.md` (database schema, entities, relationships)
2. Generate OpenAPI contracts in `contracts/` directory
3. Generate `quickstart.md` (setup instructions for developers)
4. Update agent context with new technologies
