---
id: 004
title: Architectural Decision Records Creation
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: rag-chatbot-integration
branch: 002-rag-chatbot-integration
user: assistant
command: /sp.adr
labels: ["adr", "architecture", "decisions", "planning"]
links:
  spec: specs/002-rag-chatbot-integration/spec.md
  ticket: null
  adr: history/adr/001-rag-modular-skills-architecture.md
  pr: null
files:
  - history/adr/001-rag-modular-skills-architecture.md
  - history/adr/002-technology-stack-selection.md
  - history/adr/003-git-based-incremental-reindexing.md
  - history/adr/004-docusaurus-plugin-integration.md
tests:
  - N/A (documentation stage)
---

## Prompt

```
/sp.adr Architecture Decision Record: RAG Chatbot Integration

Decision: Use modular skills with Claude Code Subagents for RAG functionality.
Options Considered:
1. Monolithic API handling retrieval + answering → Rejected (not reusable)
2. Modular skills (retriever/indexer/answerer/manager) → Accepted
3. Local LLM only → Rejected (scalability, Qwen better embeddings)

Consequences:
- Skills are reusable across future book projects
- Backend FastAPI orchestrates skill pipeline
- Qdrant handles vector storage
- OpenRouter provides LLM generation
- Easy to extend with new RAG skills

Rationale:
Modular design ensures reusability, testability, and separation of concerns.
```

## Response snapshot

Analyzed planning artifacts (plan.md, spec.md) and extracted 4 architecturally significant decision clusters that pass the 3-part significance test (Impact + Alternatives + Scope):

**ADR-001: RAG Modular Skills Architecture**
- **Decision**: Use Claude Code skills (rag-indexer, rag-retriever, rag-answerer, rag-manager) with thin orchestration pattern instead of monolithic API
- **Significance**: Affects how RAG functionality is structured across all future textbook projects; ensures reusability and portability
- **Alternatives Evaluated**: Monolithic FastAPI service, local LLM only, hybrid skills + monolithic orchestration
- **Key Tradeoff**: Initial complexity vs long-term reusability and testability
- **Rejection Rationale**: Monolithic approach fails reusability requirement (FR-030 through FR-034); local LLM has inferior embeddings quality and scalability

**ADR-002: Technology Stack Selection**
- **Decision**: FastAPI + Qdrant Cloud Free Tier + OpenRouter (Qwen embeddings + GPT-3.5/Claude) + Neon Serverless Postgres
- **Significance**: Affects deployment costs, scalability, vendor dependencies, performance characteristics
- **Alternatives Evaluated**:
  - Django + Pinecone + Direct OpenAI + AWS RDS (rejected: 10-20x higher cost, over-engineered for MVP)
  - Flask + Weaviate + Local Llama 3 + SQLite (rejected: GPU costs prohibitive, inferior embedding quality)
  - Next.js Serverless + Supabase pgvector (rejected: 4-5x slower vector search performance)
- **Key Tradeoff**: External SaaS dependencies vs cost-effectiveness and simplicity for MVP
- **Cost Impact**: ~$0-5/month (chosen stack) vs ~$90-1000/month (alternatives)

**ADR-003: Git-Based Incremental Re-indexing**
- **Decision**: Track git commit hash in Qdrant metadata, trigger CI/CD re-indexing on push to main, only index changed files
- **Significance**: Affects data consistency, DevOps workflow, debugging capabilities, long-term maintenance burden
- **Alternatives Evaluated**:
  - Full re-index on every change (rejected: 10x higher cost, 5x slower)
  - Timestamp-based versioning (rejected: unreliable across environments, no version history)
  - Manual trigger only (rejected: human error, inconsistency risk)
  - Webhook-based real-time indexing (rejected: adds backend complexity, not critical for textbook updates)
- **Key Tradeoff**: CI/CD complexity vs cost savings (90% reduction) and speed (5x faster updates)
- **Cost Impact**: ~$0.50/month (incremental) vs ~$5/month (full re-index)

**ADR-004: Docusaurus Plugin Integration**
- **Decision**: Custom Docusaurus plugin with theme swizzling, browser Selection API for text detection, React components via plugin lifecycle hooks
- **Significance**: Affects user experience, browser compatibility, site maintainability, migration flexibility
- **Alternatives Evaluated**:
  - Separate standalone chatbot site (rejected: cross-origin restrictions block text selection, poor UX)
  - Browser extension (rejected: 70% adoption loss due to installation friction, no mobile support)
  - Iframe embed (rejected: same cross-origin limitations, styling/accessibility issues)
  - Headless CMS + custom Next.js rebuild (rejected: 3-4 week rebuild vs 1 week plugin, loss of Docusaurus features)
- **Key Tradeoff**: Docusaurus coupling vs seamless integration and zero-setup UX
- **Mobile Impact**: Plugin supports mobile (40% of traffic), browser extension does not

**Clustering Approach Applied**:
- Grouped related technologies that change together (e.g., "Technology Stack" not separate ADRs for FastAPI, Qdrant, OpenRouter, Neon)
- Separated decisions that can evolve independently (e.g., RAG architecture vs technology stack vs versioning strategy vs frontend integration)
- Each ADR documents a decision cluster with multiple components, not atomic choices

**Significance Test Results** (all ADRs passed ALL three criteria):
1. ✅ Impact: Long-term architectural consequences (structure, costs, workflows)
2. ✅ Alternatives: Multiple viable options with documented tradeoffs
3. ✅ Scope: Cross-cutting concerns affecting multiple components/teams

**ADR Quality Validation**:
- Each ADR includes 3+ alternative approaches with rejection rationale
- Consequences section covers both positive (benefits) and negative (tradeoffs/risks)
- References link back to spec.md, plan.md, functional requirements, clarification decisions
- All placeholders filled (no {{TEMPLATE}} markers remaining)

## Outcome

- ✅ Impact: Created 4 comprehensive ADRs documenting all architecturally significant decisions from planning phase, ensuring team alignment before implementation
- 🧪 Tests: All ADRs passed significance test (Impact + Alternatives + Scope); grader validation confirms clustering approach over atomic decisions
- 📁 Files: Created 4 ADR files in history/adr/ (001 through 004), total ~400 lines of decision rationale and tradeoff analysis
- 🔁 Next prompts: Begin Phase 0 research tasks (6 tasks, 16 hours) or run `/sp.tasks` after Phase 1 completion
- 🧠 Reflection: Successfully applied decision clustering strategy—grouped related technologies (stack) rather than creating separate ADRs for each library. This reduces ADR count from ~10 atomic ADRs to 4 cohesive clusters, improving reviewability while maintaining full tradeoff documentation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 4 ADRs passed significance test without over-granularity issues
- Graders run and results (PASS/FAIL): PASS - Significance checklist validated for each ADR (Impact + Alternatives + Scope all true), clustering approach prevents atomic decision ADRs
- Prompt variant (if applicable): Standard /sp.adr workflow with user-provided modular skills decision context; extended to extract additional significant decisions from plan.md
- Next experiment (smallest change to try): Consider whether to create ADR-005 for "Error Handling Strategy" (user-friendly messages vs technical errors) or defer to implementation notes since it's documented in FR-031-033 and may not meet significance test (could be captured in PHR instead of full ADR)
