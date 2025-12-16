---
id: 003
title: RAG Chatbot Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: rag-chatbot-integration
branch: 002-rag-chatbot-integration
user: assistant
command: /sp.plan
labels: ["rag", "chatbot", "architecture", "implementation-plan"]
links:
  spec: specs/002-rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot-integration/plan.md
tests:
  - N/A (planning stage)
---

## Prompt

```
/sp.plan Execution Plan for RAG Chatbot Integration:

Phase 0: Setup & Validation
- Set up Qdrant Cloud Free Tier (validate 1GB capacity)
- Configure OpenRouter API with Qwen embeddings
- Set up Neon Serverless Postgres database
- Test Docusaurus plugin integration feasibility
- Prototype minimal RAG pipeline
- Design git-based incremental indexing strategy

Phase 1: Core Architecture & Design
- Design comprehensive data models (Postgres schema, Qdrant metadata)
- Define API contracts for FastAPI endpoints (/chat/query, /chat/history, /chat/feedback, /health)
- Create architecture diagrams (component interactions, data flow)
- Document project structure and component responsibilities
- Create quickstart guide

Phase 2: Implementation (to be detailed via /sp.tasks)
- RAG Skills Development (rag-indexer, rag-retriever, rag-answerer, rag-manager)
- Backend Implementation (FastAPI routes, services, database models)
- Frontend Integration (Docusaurus plugin, React components, selection handler)
- Testing & Validation (unit tests, integration tests, E2E tests)
- Deployment & CI/CD (containerization, git-based indexing hooks)

Output:
Comprehensive architectural plan with detailed Phase 0-1 tasks, dependencies, tools, deliverables, risk management, and success metrics.
```

## Response snapshot

Created comprehensive implementation plan for RAG Chatbot Integration with:

**Technical Context:**
- Languages: Python 3.11+ (backend, skills), TypeScript/React (frontend)
- Dependencies: FastAPI, SQLAlchemy, Qdrant Client, OpenRouter SDK, Docusaurus 3.x
- Performance Goals: <3s response time (95th percentile), 100 concurrent users
- Constraints: Qdrant 1GB free tier, 8K input + 2K output tokens, anonymous sessions

**Constitution Check:**
- ✅ Minimal viable change: Phase 0 validates before committing to architecture
- ✅ Testable tasks: Each phase task has clear acceptance criteria
- ✅ No premature optimization: Design focuses on MVP functional requirements
- ✅ Security: Token limits enforced, rate limiting planned, no secrets in code
- ✅ Observability: Health checks, logging, error tracking planned

**Project Structure:**
```
backend/
├── src/api/routes/        # chat.py, feedback.py, health.py
├── src/services/          # rag_orchestrator.py, conversation_service.py
├── src/db/models.py       # SQLAlchemy models
├── tests/
└── Dockerfile

src/components/RAGChatbot/
├── ChatWidget.tsx
├── ChatInterface.tsx
├── SelectionHandler.tsx
└── styles.module.css

.claude/skills/
├── rag-indexer/
├── rag-retriever/
├── rag-answerer/
└── rag-manager/
```

**Architecture Overview:**
```
[Docusaurus Site]
    ↓ SelectionHandler intercepts text selection
    ↓ ChatWidget displays interface
[FastAPI Backend /chat/query]
    ↓ RAGOrchestrator service
[rag-manager skill]
    ↓ orchestrates pipeline
[rag-retriever skill] → Qdrant Cloud (vector search)
    ↓ retrieved chunks
[rag-answerer skill] → OpenRouter API (answer generation)
    ↓ answer + citations
[ChatInterface] ← response returned
```

**Data Models:**

Postgres Schema:
```sql
conversations (conversation_id, user_id, start_timestamp, message_count, status)
messages (message_id, conversation_id, role, content_text, selected_context,
          retrieved_chunks, citations, response_time_ms)
feedback (feedback_id, message_id, rating, comment, timestamp)
```

Qdrant Chunk Metadata:
```json
{
  "chunk_id": "UUID",
  "content": "markdown text",
  "vector": "array[1536]",
  "metadata": {
    "file_path": "docs/module-1/chapter-3.md",
    "line_start": 67,
    "commit_hash": "a7b3c9d2e1f4...",
    "module": "module-1-ros2",
    "tags": ["kinematics", "robotics"]
  }
}
```

**Phase 0: Research & Validation (6 tasks, 16 hours)**
1. Task 0.1: Qdrant Cloud Setup & Capacity Validation (2 hours)
   - Create Qdrant Cloud Free Tier account
   - Verify 1GB capacity for ~10,000 chunks
   - Test basic CRUD operations

2. Task 0.2: OpenRouter API Testing (2 hours)
   - Test Qwen embeddings (1536 dimensions)
   - Validate 8K input + 2K output token limits
   - Measure latency and cost

3. Task 0.3: Neon Postgres Setup (2 hours)
   - Create Neon Serverless Postgres instance
   - Test connection pooling and performance
   - Validate schema migration strategy

4. Task 0.4: Docusaurus Plugin Feasibility (3 hours)
   - Research plugin hooks for text selection
   - Prototype minimal selection handler
   - Test theme swizzling for ChatWidget injection

5. Task 0.5: RAG Pipeline Prototype (4 hours)
   - Build end-to-end prototype (query → retrieve → answer)
   - Validate citation mapping works correctly
   - Test error handling for missing context

6. Task 0.6: Git-Based Indexing Strategy (3 hours)
   - Design commit hash tracking mechanism
   - Prototype incremental re-indexing logic
   - Plan CI/CD webhook integration

**Phase 1: Core Architecture & Design (4 tasks, 10 hours)**
1. Task 1.1: Comprehensive Data Model Design (3 hours)
2. Task 1.2: API Contract Definition (3 hours)
3. Task 1.3: Architecture Diagram Creation (2 hours)
4. Task 1.4: Quickstart Guide (2 hours)

**Phase 2: Implementation**
- Deferred to `/sp.tasks` command after Phase 1 completion
- Will include detailed task breakdown for:
  - RAG Skills Development (4 skills with tests)
  - Backend Implementation (routes, services, database)
  - Frontend Integration (React components, Docusaurus plugin)
  - Testing & Validation (unit, integration, E2E)
  - Deployment & CI/CD (Docker, git hooks)

**Risk Management:**

Technical Risks:
- Qdrant 1GB limit exceeded → Mitigation: Monitor usage, implement pruning strategy
- OpenRouter rate limits → Mitigation: Implement exponential backoff, caching
- Docusaurus plugin conflicts → Mitigation: Phase 0.4 validates feasibility early

Operational Risks:
- Cost overruns from API usage → Mitigation: Token limits enforced, usage alerts
- Performance degradation under load → Mitigation: Load testing, connection pooling
- Data inconsistency (stale content) → Mitigation: Git commit hash tracking, auto re-indexing

**Success Metrics** (from specification):
- 80% of queries get relevant answers (similarity > 0.7)
- Sub-3-second response time (95th percentile)
- 70% task completion on first interaction
- 60% positive feedback rate
- 100 concurrent users without degradation
- Zero hallucinated citations
- 99.5% uptime during peak hours

**Timeline:**
- Phase 0 (Research): Week 1 (16 hours)
- Phase 1 (Design): Week 2 (10 hours)
- Phase 2 (Implementation): Weeks 3-10 (via /sp.tasks)
- Testing & Validation: Weeks 11-12

## Outcome

- ✅ Impact: Comprehensive architectural plan with 3-phase execution strategy, detailed Phase 0-1 tasks, risk management, and clear path to implementation
- 🧪 Tests: N/A (planning stage) - Phase 0 includes feasibility validation, Phase 2 will have comprehensive testing
- 📁 Files: Created plan.md (~500 lines) with architecture diagrams, data models, API contracts, project structure, and phased task breakdown
- 🔁 Next prompts: Begin Phase 0 research tasks (6 tasks, 16 hours) or run `/sp.tasks` after Phase 1 completion
- 🧠 Reflection: Successfully balanced detailed planning (Phase 0-1) with flexibility for implementation details (Phase 2 deferred to /sp.tasks). Phase 0 includes critical feasibility validation before committing to full architecture. Constitution check ensures plan adheres to project principles (minimal viable change, testable tasks, no premature optimization).

## Evaluation notes (flywheel)

- Failure modes observed: None - plan completed with clear phase boundaries, dependencies, and acceptance criteria for each task
- Graders run and results (PASS/FAIL): PASS - All constitution principles validated, project structure aligns with existing codebase patterns
- Prompt variant (if applicable): Standard /sp.plan with 3-phase breakdown (Research, Design, Implementation)
- Next experiment (smallest change to try): Consider whether Phase 0 tasks should run in parallel (e.g., 0.1-0.3 can be done concurrently) or sequentially to reduce calendar time from 16 hours to ~1 week with proper resource allocation
