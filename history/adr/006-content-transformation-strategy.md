# ADR-006: Content Transformation Strategy (Personalization & Translation)

> **Scope**: Document decision cluster for on-demand LLM-based content transformation including generation strategy, storage approach, and cost control.

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** auth-personalization-translation
- **Context:** Need to implement personalized chapter content (adapted to user skill level and hardware access) and Urdu translation for the textbook platform. Requirements include: (1) personalization based on 5 skill levels (AI, ML, ROS, Python, Linux) and hardware flags (GPU, Jetson, Robot, Cloud-only), (2) Urdu translation with Focus Mode (technical faithfulness), (3) fast generation (<10s personalization, <15s translation), (4) cost-efficient (<$0.01 per transformation), (5) static Docusaurus site compatibility (GitHub Pages deployment). Constraints: OpenRouter free tier rate limits (20 req/min, 200 req/day), no pre-generated content storage explosion, session-based user experience acceptable.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects content scalability, storage costs, LLM vendor dependencies, user experience patterns
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - On-demand generation vs pre-generated variations vs hybrid caching vs local LLM
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts frontend state management, backend API design, database schema, deployment strategy
-->

## Decision

**Adopt on-demand LLM generation with session-only storage:**

- **Generation Trigger**: User-initiated (click "Personalize Chapter" or "Translate to Urdu" button)
- **LLM Provider**: OpenRouter free tier models
  - Personalization: `meta-llama/llama-3.3-70b-instruct` (advanced reasoning)
  - Translation: `google/gemini-2.0-flash-exp:free` (multilingual capabilities)
- **Storage Strategy**: Session-only in browser IndexedDB (50MB+ capacity)
  - Key: `userId:chapterId:transformationType` (personalization | translation)
  - Lifecycle: Created on-demand, expires on browser close or logout
  - No backend database persistence (reduces costs and complexity)
- **Regeneration Policy**: Regenerate on new session (ensures fresh content if book updates)
- **Cost Control**: Free tier rate limiting (20 req/min, 200 req/day), graceful degradation with user-friendly messages, request queuing with exponential backoff
- **Static Site Compatibility**: Transformations happen at runtime via React components; original Docusaurus markdown remains static

**Architecture Pattern**: User clicks button → Frontend calls backend API → Backend calls OpenRouter LLM → Frontend stores result in IndexedDB → Display transformed content (session-scoped)

## Consequences

### Positive

- **Zero Storage Costs**: No database storage for transformed content; IndexedDB is browser-local (free)
- **Content Freshness**: Regeneration on new session ensures users get latest book updates reflected in transformations
- **Scalability**: On-demand generation scales horizontally with users (no exponential content explosion)
- **Static Site Maintained**: Docusaurus site remains fully static; can deploy to GitHub Pages without serverless functions
- **User Control**: Users explicitly trigger transformations (clear intent, predictable behavior)
- **LLM Flexibility**: Can switch OpenRouter models without data migration (no pre-generated content locked to old models)
- **Development Velocity**: No pre-generation pipeline to build/maintain; faster iteration on transformation prompts

### Negative

- **Latency on First Load**: 10-15 second wait for transformation (SC-002, SC-003); mitigated by loading indicators and caching in session
- **Rate Limit Risk**: 20 req/min limit means 20 concurrent users can saturate free tier; mitigated by request queuing and user feedback
- **Session Loss Frustration**: Users lose personalized content on browser close/logout; must regenerate next session (trade-off for zero storage costs)
- **No Cross-Device Sync**: Personalized content doesn't sync across devices (browser-local only)
- **OpenRouter Dependency**: Single point of failure; if OpenRouter API is down, transformations unavailable (mitigated by disabling buttons with message)
- **Network Required**: Cannot personalize/translate offline (inherent to LLM API approach)

## Alternatives Considered

### Alternative 1: Pre-Generate All Content Variations

**Approach**:
- Pre-generate 3 skill level variations (beginner, intermediate, advanced) × 3 hardware variations (cloud, GPU, robot) = 9 versions per chapter
- Pre-generate Urdu translation for all chapters
- Store all variations in Git repository or CDN
- Frontend selects appropriate version based on user profile

**Tradeoffs**:
- ✅ Zero latency (instant content display)
- ✅ Works offline (all variations pre-loaded)
- ✅ No LLM API dependency at runtime
- ❌ Content explosion: 21 chapters × 9 personalization variations + 21 Urdu translations = 210+ files
- ❌ Storage costs: ~500KB per variation × 210 = 105MB (exceeds GitHub repo size best practices)
- ❌ Build time: 210 LLM generations × 10-15s = 30-50 minutes per build
- ❌ Stale content: Pre-generated variations become outdated when book updates; must regenerate entire set
- ❌ Coarse granularity: 9 variations don't capture nuance of 5 skill levels × 4 hardware combinations = 20 actual user profiles

**Rejection Rationale**: Content explosion unmanageable for MVP. 105MB of generated content violates GitHub repository best practices. Build time too slow for rapid iteration. On-demand generation provides finer granularity at zero storage cost.

### Alternative 2: Database-Persisted Transformations (User-Specific Cache)

**Approach**:
- Store each user's personalized/translated chapters in Neon Postgres
- Database schema: `transformed_content` table (user_id, chapter_id, transformation_type, content, created_at)
- First request: Generate and store; subsequent requests: retrieve from database

**Tradeoffs**:
- ✅ Persists across sessions and devices
- ✅ Faster on subsequent loads (database retrieval ~100ms vs generation ~10-15s)
- ✅ Can analyze transformation usage patterns (which users personalize, which chapters)
- ❌ Database storage costs: 500KB per transformation × 100 users × 5 chapters personalized = 250MB (Neon free tier 512MB limit)
- ❌ Database migration complexity: must version schema, handle content updates (invalidate stale transformations)
- ❌ Increases backend complexity: CRUD operations, cache invalidation logic
- ❌ Privacy concerns: storing personalized content server-side (vs browser-local session storage)

**Rejection Rationale**: Storage costs scale linearly with users and chapters. Database persistence violates spec requirement (FR-015: "session for duration of browsing session only; content regenerated on new session"). Session storage is sufficient for MVP; can revisit if user feedback demands persistence.

### Alternative 3: Hybrid Caching (Redis + Shared Transformations)

**Approach**:
- Cache transformations in Redis with TTL (24 hours)
- Cache key: `chapterContent:hash:transformationType:profileArchetype` (not user-specific)
- Share transformations across users with similar profiles (e.g., all beginner/cloud-only users get same personalization)
- Reduces LLM API calls by deduplicating similar requests

**Tradeoffs**:
- ✅ Reduces LLM API costs (cache hit rate ~50-70% after initial warm-up)
- ✅ Faster for common profile archetypes (cache retrieval ~10ms vs generation ~10s)
- ✅ TTL ensures content stays relatively fresh (24-hour invalidation)
- ❌ Adds Redis infrastructure (deployment complexity, cost ~$10/month for managed Redis)
- ❌ Cache invalidation on book updates requires manual intervention or versioning
- ❌ Less personalized (archetypes are coarse approximations of 5-skill × 4-hardware = 20 profiles)
- ❌ Violates spec requirement (FR-015: no cache persistence)

**Rejection Rationale**: Adds infrastructure complexity (Redis) without significant UX improvement over session storage. Shared archetypes sacrifice personalization granularity. OpenRouter free tier (200 req/day) is sufficient for MVP with session storage. Can revisit if free tier becomes bottleneck.

### Alternative 4: Local LLM (Self-Hosted Llama 3)

**Approach**:
- Deploy Llama 3 70B on GPU server (AWS p3.2xlarge or similar)
- Backend calls local LLM API instead of OpenRouter
- Unlimited generations (no rate limits or token costs)

**Tradeoffs**:
- ✅ No rate limits (can handle unlimited concurrent users)
- ✅ No external API dependency (fully self-contained)
- ✅ Better latency (no network round-trip to OpenRouter)
- ✅ Data privacy (all content stays on our infrastructure)
- ❌ GPU infrastructure costs: $1,000-3,000/month for p3.2xlarge (24/7 uptime)
- ❌ DevOps burden: model deployment, scaling, monitoring, GPU driver management
- ❌ Inferior quality: Local Llama 3 embeddings/generation worse than OpenRouter's fine-tuned models for technical content
- ❌ Not cost-effective for MVP: $1,000/month vs $0/month (OpenRouter free tier)

**Rejection Rationale**: Cost-prohibitive for MVP ($1,000-3,000/month GPU costs). OpenRouter free tier meets success criteria (SC-002, SC-003, SC-012). Local LLM more suitable for production scale after validating product-market fit. Can migrate to self-hosted if OpenRouter costs become unsustainable (>$100/month).

## References

- Feature Spec: [specs/003-auth-personalization-translation/spec.md](../../specs/003-auth-personalization-translation/spec.md) (FR-008, FR-009, FR-010, FR-011, FR-013, FR-014, FR-015)
- Implementation Plan: [specs/003-auth-personalization-translation/plan.md](../../specs/003-auth-personalization-translation/plan.md) (Constraints, Performance Goals)
- Research: [specs/003-auth-personalization-translation/research.md](../../specs/003-auth-personalization-translation/research.md) (Section 3: OpenRouter API Integration, Section 4: Session Storage Strategy)
- Data Model: [specs/003-auth-personalization-translation/data-model.md](../../specs/003-auth-personalization-translation/data-model.md) (Section 2: Session Storage Schema)
- Success Criteria: SC-002 (personalization <10s), SC-003 (translation <15s), SC-004 (95% success rate), SC-012 (cost <$0.01/request)
- OpenRouter Docs: https://openrouter.ai/docs
- Clarifications: Session 2025-12-17 (Q2: On-demand generation, Q3: Session storage only)
