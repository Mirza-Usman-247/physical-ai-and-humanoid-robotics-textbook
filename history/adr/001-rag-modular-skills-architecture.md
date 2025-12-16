# ADR-001: RAG Modular Skills Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** rag-chatbot-integration
- **Context:** Building a Retrieval-Augmented Generation (RAG) chatbot for the Physical AI & Humanoid Robotics textbook. Need to decide whether to implement RAG functionality as a monolithic API service or as modular, reusable Claude Code skills. The system must support basic Q&A, context-aware queries with user-selected text, multi-turn conversations, and feedback collection. A key requirement is reusability across future textbook projects.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Defines how RAG functionality is structured across projects
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Monolithic API, modular skills, local LLM, hybrid approaches
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Affects all RAG-related development, testing, and deployment
-->

## Decision

**Adopt a modular RAG architecture using Claude Code skills with thin orchestration:**

- **rag-indexer**: Chunks markdown content, generates Qwen embeddings, uploads to Qdrant with git commit hash metadata
- **rag-retriever**: Performs semantic search over Qdrant, supports hybrid search (vector + keyword), returns top-k chunks with scores
- **rag-answerer**: Generates cited answers from retrieved context via OpenRouter LLM, enforces "never fabricate" policy with mandatory inline citations
- **rag-manager**: Thin orchestration layer that sequences retrieve → answer → validate pipeline, delegates all domain logic to sub-skills

**Orchestration Pattern**: Backend FastAPI service calls rag-manager skill, which delegates to specialized sub-skills. No domain logic duplication—rag-manager only handles sequencing, validation, and error aggregation.

**Reusability Guarantee**: Each skill is self-contained with comprehensive SKILL.md documentation, independent tests, and no hardcoded textbook-specific logic.

## Consequences

### Positive

- **Reusability**: Skills can be used independently or composed for future textbook projects without modification
- **Testability**: Each skill has isolated unit tests with mock dependencies (Qdrant, OpenRouter)
- **Separation of Concerns**: Clear boundaries—indexer handles ingestion, retriever handles search, answerer handles generation
- **Maintainability**: Changes to indexing logic don't affect answering logic; skills evolve independently
- **Portability**: Skills are cross-platform Python modules that work in local development, CI/CD, and production
- **Modularity**: Can swap retrieval strategies (semantic → hybrid → keyword) or LLM providers without touching other skills
- **Team Velocity**: Engineers can work on different skills in parallel without conflicts

### Negative

- **Initial Complexity**: Requires defining clear skill interfaces, SKILL.md documentation, and orchestration contracts upfront
- **Orchestration Overhead**: rag-manager adds an extra layer; must avoid becoming a "god object" that duplicates logic
- **Testing Burden**: Need to test both individual skills (unit) and integrated pipeline (integration)
- **Skill Boundaries**: Ambiguity about which logic belongs in retriever vs answerer (e.g., re-ranking, filtering)
- **Dependency Management**: Skills must manage Qdrant/OpenRouter SDKs independently, potential version conflicts
- **Learning Curve**: New developers must understand skill interaction patterns and orchestration flow

## Alternatives Considered

### Alternative 1: Monolithic FastAPI Service

**Description**: Single FastAPI application with routes handling retrieval + answering logic directly (no separate skills).

**Tradeoffs**:
- ✅ Simpler initial implementation, fewer moving parts
- ✅ No orchestration complexity, single codebase
- ❌ Not reusable across projects—would need to copy-paste code
- ❌ Tight coupling between retrieval and generation logic
- ❌ Testing requires spinning up full backend service

**Rejection Rationale**: Fails the reusability requirement (FR-030 through FR-034). Future textbook projects would have no easy path to adopt RAG functionality.

### Alternative 2: Local LLM Only (No External APIs)

**Description**: Deploy a local LLM (e.g., Llama 3, Mistral) for embeddings and generation instead of using OpenRouter API.

**Tradeoffs**:
- ✅ No external API dependencies, lower long-term costs
- ✅ Full control over model performance and privacy
- ❌ Requires GPU infrastructure for inference (higher DevOps burden)
- ❌ Qwen embeddings (1536-dim) outperform local models for semantic search
- ❌ Scalability concerns—local LLM can't handle 100 concurrent users without significant hardware

**Rejection Rationale**: OpenRouter + Qwen provides better embeddings quality and scalability with minimal setup. Cost-effective for MVP (estimated 1,000 queries/day = ~$10/month). Can revisit local LLM for future optimization.

### Alternative 3: Hybrid Skills + Monolithic Orchestration

**Description**: Create retriever and answerer skills, but keep orchestration logic inside FastAPI backend (not as a separate rag-manager skill).

**Tradeoffs**:
- ✅ Slightly simpler—one less skill to maintain
- ✅ Backend has direct control over pipeline sequencing
- ❌ Orchestration logic is not reusable (locked in backend code)
- ❌ Testing pipeline requires backend setup, harder to isolate

**Rejection Rationale**: Violates modularity principle—future projects would need to reimplement orchestration. rag-manager as a skill ensures pipeline logic is portable.

## References

- Feature Spec: [specs/002-rag-chatbot-integration/spec.md](../../specs/002-rag-chatbot-integration/spec.md)
- Implementation Plan: [specs/002-rag-chatbot-integration/plan.md](../../specs/002-rag-chatbot-integration/plan.md)
- Related ADRs: ADR-002 (Technology Stack), ADR-003 (Content Versioning Strategy)
- Functional Requirements: FR-030 (Portable skills), FR-034 (Orchestration layer), FR-037 (Delegation pattern)
- User Input: `/sp.adr` command with explicit modular skills decision context
