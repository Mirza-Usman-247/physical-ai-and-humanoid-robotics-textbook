# ADR-007: Stateless RAG Chatbot Integration with Transformed Content

> **Scope**: Document decision cluster for integrating personalized/translated content with the existing RAG chatbot including context delivery mechanism and stateless architecture.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** auth-personalization-translation
- **Context:** Need to integrate personalized and translated chapter content with the existing RAG chatbot so that chatbot responses reflect the user's customized content view. Requirements include: (1) RAG chatbot must answer questions based on personalized or translated content (not original), (2) integration must work with session-stored transformations (IndexedDB), (3) maintain existing RAG chatbot performance (<3s response time), (4) avoid backend session synchronization complexity, (5) stateless chatbot architecture preferred for scalability. Existing RAG implementation uses rag-orchestrator service (backend/src/services/rag_orchestrator.py) calling Claude Code RAG skills (rag-retriever, rag-answerer).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects RAG chatbot scalability, state management patterns, frontend-backend coupling
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Frontend-sent context vs backend session vs backend regeneration vs dual RAG indexes
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts frontend state, backend API design, RAG pipeline architecture, chatbot UX
-->

## Decision

**Adopt stateless RAG chatbot with frontend-sent context chunks:**

- **Context Delivery**: Frontend sends transformed content chunks with each chatbot query in API request body
- **RAG Chatbot Modification**: Modify `rag_orchestrator.py` `query()` method to accept optional `context_chunks` parameter
- **Context Source Priority**: If `context_chunks` provided → use them; otherwise → fall back to Qdrant vector search (original content)
- **Chunking Strategy**: Frontend intelligently selects relevant sections from personalized/translated content based on question keywords (smart chunking to fit within OpenRouter 128K token context window)
- **Stateless Pattern**: Backend does not store or track which content version user is viewing; frontend is source of truth
- **Integration**: Minimal changes to existing RAG pipeline; `rag-answerer` skill uses provided context instead of retrieved chunks

**Architecture Pattern**: User asks question → Frontend retrieves personalized/translated content from IndexedDB → Frontend extracts relevant sections → Frontend sends question + context chunks to backend → Backend passes chunks to rag-answerer → Backend returns answer

## Consequences

### Positive

- **Stateless Backend**: No backend session state required; RAG chatbot scales horizontally without session synchronization
- **Frontend Control**: Frontend knows exactly which content version user is viewing (session storage); no ambiguity or race conditions
- **Minimal Backend Changes**: Only modify `rag_orchestrator.py` to accept optional `context_chunks` parameter; no database schema changes, no session management
- **Backward Compatible**: Falls back to Qdrant vector search if no `context_chunks` provided; existing chatbot functionality preserved
- **Simplified Debugging**: Content chunks visible in API request body; easy to inspect what context chatbot received
- **No Sync Issues**: No risk of frontend and backend having different content versions (frontend is single source of truth)

### Negative

- **Request Size**: Sending content chunks in request body increases payload size (~2-5KB per request vs ~500 bytes for question-only)
- **Network Bandwidth**: Repeated context chunks sent with each question (no backend caching); acceptable trade-off for stateless architecture
- **Frontend Complexity**: Frontend must implement smart chunking logic (extract relevant sections based on question keywords)
- **Token Limit Risk**: Large chapters + long conversations may exceed OpenRouter 128K context window; mitigated by intelligent section extraction
- **No Content Reuse**: Backend cannot optimize by caching frequently asked questions on same chapter (each request is independent)

## Alternatives Considered

### Alternative 1: Backend Session Storage with Context Synchronization

**Approach**:
- When user personalizes/translates content, frontend sends result to backend
- Backend stores in Redis session (user_id:chapter_id:transformation_type)
- RAG chatbot retrieves user's current content version from Redis before answering
- Frontend and backend stay synchronized via session updates

**Tradeoffs**:
- ✅ Smaller API request payloads (no content chunks in body)
- ✅ Backend can cache frequently asked questions per chapter
- ✅ Potential for backend analytics (which chapters users personalize)
- ❌ Requires Redis infrastructure ($10-20/month for managed instance)
- ❌ Session synchronization complexity (frontend must notify backend of content changes)
- ❌ Race conditions possible (frontend updates content while backend processes query)
- ❌ Violates spec requirement (FR-015: session-only storage, no backend cache)
- ❌ Increased backend memory usage (storing content for all active users)

**Rejection Rationale**: Adds infrastructure (Redis) and state management complexity. Violates session-only storage requirement. Stateless approach (frontend-sent context) is architecturally simpler and aligns with microservices best practices (stateless services scale better).

### Alternative 2: Backend Regenerates Personalized Content on Each Query

**Approach**:
- Frontend sends user profile + chapter ID + question to backend
- Backend regenerates personalized content on-the-fly using OpenRouter LLM
- Backend passes regenerated content to rag-answerer
- No frontend context chunks, no backend storage

**Tradeoffs**:
- ✅ No frontend chunking logic (backend handles everything)
- ✅ Guaranteed consistency (always regenerates fresh content)
- ✅ No request payload size concerns (only profile + question)
- ❌ Doubles LLM API calls (personalization + RAG answer = 2 OpenRouter requests per question)
- ❌ Increases latency (10s personalization + 3s RAG = 13s total vs 3s with frontend context)
- ❌ Violates performance constraint (SC-006: RAG <3s response time)
- ❌ Doubles costs (2× OpenRouter API calls)
- ❌ Wastes computation (regenerating same content for every question on same chapter)

**Rejection Rationale**: Unacceptable latency (13s vs 3s target). Doubles LLM costs. Frontend already has personalized content in IndexedDB (session storage); no need to regenerate. Chosen approach (frontend-sent context) reuses existing transformation.

### Alternative 3: Dual RAG Indexes (Personalized + Original)

**Approach**:
- Maintain two Qdrant collections: "original-content" and "personalized-content-{user_id}"
- When user personalizes chapter, index personalized version in user-specific Qdrant collection
- RAG chatbot queries user-specific collection if exists; otherwise falls back to original
- Backend manages which collection to query based on user authentication

**Tradeoffs**:
- ✅ No frontend context chunks (backend queries appropriate index)
- ✅ Faster retrieval (Qdrant vector search optimized for speed)
- ✅ Supports semantic search on personalized content (potentially better retrieval quality)
- ❌ Qdrant storage explosion: 100 users × 5 personalized chapters × 500KB = 250MB (exceeds Qdrant Free Tier 1GB with original content)
- ❌ Index maintenance complexity (invalidate user-specific indexes when book updates)
- ❌ Collection proliferation (100 users = 100 Qdrant collections)
- ❌ Violates spec requirement (FR-015: no backend persistence for transformations)
- ❌ Increased Qdrant costs (must upgrade to paid tier for storage)

**Rejection Rationale**: Qdrant Free Tier 1GB insufficient for user-specific collections. Violates session-only storage requirement (personalized content must be ephemeral). Chosen approach (frontend-sent context) keeps Qdrant usage lean (only original content indexed).

### Alternative 4: Hybrid: Backend Fetches from Frontend Session

**Approach**:
- Frontend stores personalized content in IndexedDB (as chosen)
- Backend API endpoint: `GET /api/content/{user_id}/{chapter_id}`
- RAG chatbot calls this endpoint to fetch user's current content version
- Backend proxies frontend session storage via API

**Tradeoffs**:
- ✅ Separates concerns (RAG chatbot doesn't need context in request)
- ✅ Cleaner API design (RAG query endpoint only has question parameter)
- ❌ Impossible: Backend cannot access browser IndexedDB (security sandbox)
- ❌ Would require frontend to upload session content to backend (defeats purpose of session-only storage)
- ❌ Extra API round-trip (backend fetches content, then calls RAG) → increased latency

**Rejection Rationale**: Technically infeasible (backend cannot access browser storage). Would require uploading session content to backend, which violates session-only requirement. Chosen approach (frontend-sent context) is direct and pragmatic.

## References

- Feature Spec: [specs/003-auth-personalization-translation/spec.md](../../specs/003-auth-personalization-translation/spec.md) (FR-016, User Story 4)
- Implementation Plan: [specs/003-auth-personalization-translation/plan.md](../../specs/003-auth-personalization-translation/plan.md) (Phase 4: RAG Integration)
- Research: [specs/003-auth-personalization-translation/research.md](../../specs/003-auth-personalization-translation/research.md) (Section 5: RAG Chatbot Integration Pattern)
- Data Model: [specs/003-auth-personalization-translation/data-model.md](../../specs/003-auth-personalization-translation/data-model.md) (Section 4.3: RAG Chatbot with Transformed Content)
- Existing RAG Orchestrator: [backend/src/services/rag_orchestrator.py](../../backend/src/services/rag_orchestrator.py) (lines 153-220: query method)
- Related ADRs: ADR-001 (RAG Modular Skills Architecture), ADR-006 (Content Transformation Strategy)
- Success Criteria: SC-006 (RAG chatbot <3s response time with personalized/translated context)
- Clarifications: Session 2025-12-17 (Q2: Frontend sends transformed content chunks with each chatbot query via API request body)
