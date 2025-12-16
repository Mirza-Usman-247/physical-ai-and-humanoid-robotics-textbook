---
id: 004
title: ADR Creation Complete
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: 003-auth-personalization-translation
branch: 003-auth-personalization-translation
user: Mirza-Usman-247
command: /sp.adr
labels: ["adr", "architecture-decision", "authentication", "content-transformation", "rag-integration"]
links:
  spec: specs/003-auth-personalization-translation/spec.md
  ticket: null
  adr: history/adr/
  pr: null
files:
  - history/adr/005-authentication-architecture.md
  - history/adr/006-content-transformation-strategy.md
  - history/adr/007-stateless-rag-chatbot-integration.md
tests:
  - Significance test passed for all 3 ADRs (impact, alternatives, scope)
  - All ADRs include explicit alternatives with tradeoffs
  - All ADRs include consequences (positive and negative)
---

## Prompt

Architecture Decision Record: Personalization & Translation

Decision 1:
Use Better Auth for authentication instead of custom auth.
Reason:
Secure, production-ready, minimal maintenance.

Decision 2:
Generate personalized and translated content on-demand using LLMs.
Reason:
Avoid content explosion and keep static site lightweight.

Decision 3:
Use OpenRouter free models for personalization and translation.
Reason:
Cost control and flexibility across providers.

Decision 4:
Keep Docusaurus content static; inject dynamic content at runtime.
Reason:
GitHub Pages compatibility.

Consequences:
- Requires FastAPI backend
- Requires caching strategy
- Enables future multilingual expansion

## Response snapshot

Created 3 Architecture Decision Records documenting architecturally significant decisions from planning phase:

**ADR-005: Authentication Architecture**
- Decision cluster: Better Auth (frontend) + JWT tokens (15-min access, 7-day refresh) + FastAPI validation (python-jose) + bcrypt password hashing (cost factor 12) + Neon Postgres (users, user_profiles tables) + stateless authentication pattern
- Alternatives: Auth0 (SaaS), Custom auth implementation, Firebase Authentication, Passport.js (Node.js middleware)
- Rejection rationale: Auth0 = vendor lock-in + overkill for email/password; Custom = security risk + reinventing wheel; Firebase = Firestore mismatch with Neon Postgres requirement; Passport.js = Node.js-centric not React SPA-focused
- Consequences (positive): Modern React-first library, stateless JWT scales horizontally, no vendor lock-in, GitHub Pages compatible, FastAPI native OAuth2/JWT support
- Consequences (negative): Better Auth learning curve, token management complexity, no built-in 2FA, JWT size overhead, token revocation challenge (short expiry mitigates)

**ADR-006: Content Transformation Strategy (Personalization & Translation)**
- Decision cluster: On-demand LLM generation (user-triggered) + OpenRouter free tier (Llama 3.3 70B personalization, Gemini 2.0 Flash translation) + session-only IndexedDB storage (userId:chapterId:transformationType key) + regenerate on new session + cost control (rate limiting, graceful degradation)
- Alternatives: Pre-generate all variations (9 personalization + Urdu = 210 files), Database-persisted transformations (Neon Postgres cache), Hybrid Redis caching (24-hour TTL shared archetypes), Local LLM (self-hosted Llama 3 on GPU)
- Rejection rationale: Pre-generate = 105MB content explosion + 30-50min builds; Database = storage costs scale linearly + violates session-only requirement; Redis = infrastructure complexity + violates no-cache requirement; Local LLM = $1,000-3,000/month GPU costs
- Consequences (positive): Zero storage costs, content freshness on regeneration, horizontal scalability, static site maintained, user control, LLM flexibility, fast development iteration
- Consequences (negative): 10-15s latency on first load, rate limit risk (20 req/min), session loss frustration, no cross-device sync, OpenRouter dependency, network required

**ADR-007: Stateless RAG Chatbot Integration with Transformed Content**
- Decision cluster: Frontend sends transformed content chunks with each chatbot query + stateless backend (no session state) + optional context_chunks parameter in rag_orchestrator.py + smart chunking (frontend extracts relevant sections) + fallback to Qdrant vector search if no chunks provided
- Alternatives: Backend session storage (Redis), Backend regenerates personalized content on each query, Dual RAG indexes (personalized + original Qdrant collections), Hybrid backend fetches from frontend session
- Rejection rationale: Backend session = Redis infrastructure + sync complexity + violates session-only; Regeneration = doubles LLM calls + 13s latency; Dual indexes = Qdrant storage explosion + violates no-persistence; Hybrid fetch = technically infeasible (backend can't access browser IndexedDB)
- Consequences (positive): Stateless backend scales horizontally, frontend control (source of truth), minimal backend changes, backward compatible, simplified debugging, no sync issues
- Consequences (negative): Larger request size (2-5KB), network bandwidth (repeated chunks), frontend chunking complexity, token limit risk (128K context), no backend caching

All 3 ADRs passed significance checklist:
- ✅ Impact: Long-term architectural consequences (security, costs, scalability, vendor dependencies, state management)
- ✅ Alternatives: Each ADR lists 4 alternatives with explicit tradeoffs
- ✅ Scope: Cross-cutting concerns (frontend, backend, database, deployment, all API endpoints)

## Outcome

- ✅ Impact: Complete architectural documentation for auth, personalization/translation, and RAG integration decisions; clear rationale for technology choices; explicit tradeoffs captured for future reference
- 🧪 Tests: All ADRs passed significance test (impact + alternatives + scope); all include consequences (positive and negative); all reference planning artifacts
- 📁 Files: Created 3 ADRs (ADR-005, ADR-006, ADR-007) totaling ~450 lines of architecture documentation
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown; ADRs provide architectural foundation for task planning
- 🧠 Reflection: Decision clustering worked well (authentication architecture as integrated solution, content transformation strategy holistically, RAG integration pattern); alternatives analysis revealed cost/complexity tradeoffs (pre-generation too expensive, backend session too complex, local LLM too costly); stateless patterns emerged as consistent theme (JWT stateless auth, session-only storage, stateless RAG chatbot)

## Evaluation notes (flywheel)

- Failure modes observed: None; all 3 ADRs created successfully with comprehensive alternatives analysis
- Graders run and results (PASS/FAIL): Significance checklist - PASS (all 3 ADRs have impact, alternatives, scope); Completeness grader - PASS (all ADRs have context, decision, consequences, alternatives, references)
- Prompt variant (if applicable): Standard /sp.adr workflow with decision clustering (grouped related technologies) instead of atomic ADRs
- Next experiment (smallest change to try): For future ADR creation, consider adding "Decision Timeline" section showing when decision can be revisited/re-evaluated (e.g., "Revisit OpenRouter decision if costs exceed $100/month or free tier rate limits become bottleneck")
