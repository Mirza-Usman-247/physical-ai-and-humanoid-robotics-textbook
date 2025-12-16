---
description: "Atomic implementation tasks for RAG Chatbot Integration"
---

# Tasks: RAG Chatbot Integration for Physical AI Textbook

**Input**: Design documents from `/specs/002-rag-chatbot-integration/`
**Prerequisites**: plan.md, spec.md (user stories), ADRs (001-004)

**Tests**: Tests are NOT explicitly requested in the specification. Tasks focus on implementation with validation checkpoints per user story.

**Organization**: Tasks are grouped by user story (P1-P4) to enable independent implementation and testing of each story. Each phase represents a complete, deployable increment.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a web application with:
- **Backend**: `backend/src/` (FastAPI Python)
- **Frontend**: `src/` (Docusaurus React components)
- **Skills**: `.claude/skills/` (Reusable RAG skills)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, external services setup, and basic structure

- [x] T001 Create backend directory structure: backend/src/{api,services,db,config.py,utils}
- [x] T002 Create frontend directory structure: src/{components/RAGChatbot,plugins/chatbot-plugin,api,hooks}
- [x] T003 [P] Create backend/requirements.txt with FastAPI 0.109+, qdrant-client 1.7+, openai SDK, psycopg2, tiktoken, pyyaml
- [x] T004 [P] Create backend/Dockerfile for Python 3.11+ container deployment
- [x] T005 [P] Create backend/.env.example with environment variable templates (QDRANT_URL, OPENROUTER_API_KEY, NEON_DATABASE_URL)
- [x] T006 Set up Qdrant Cloud Free Tier account and create collection with vector_size=1536, distance=cosine
- [x] T007 [P] Set up OpenRouter API account and obtain API key for Qwen embeddings + LLM access
- [x] T008 [P] Set up Neon Serverless Postgres database instance and obtain connection string
- [x] T009 Configure backend/src/config.py to load environment variables with validation

**Checkpoint**: Infrastructure ready - external services provisioned, project structure in place

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T010 Create Postgres schema: tables for users, conversations, messages, feedback in backend/src/db/models.py (SQLAlchemy)
- [x] T011 Implement database connection pool in backend/src/db/connection.py with Neon Postgres
- [x] T012 [P] Create Alembic migration framework in backend/src/db/migrations/ for schema versioning
- [x] T013 [P] Implement token counter utility in backend/src/utils/token_counter.py using tiktoken for 8K input / 2K output limits
- [x] T014 [P] Implement input validation utility in backend/src/utils/validators.py for query sanitization (prevent prompt injection)
- [x] T015 [P] Create Pydantic request models in backend/src/api/models/request.py (ChatQueryRequest, FeedbackRequest)
- [x] T016 [P] Create Pydantic response models in backend/src/api/models/response.py (ChatQueryResponse, ErrorResponse)
- [x] T017 Implement rate limiting middleware in backend/src/api/middleware/rate_limit.py (20 requests/min per session)
- [x] T018 [P] Implement global error handler in backend/src/api/middleware/error_handler.py with user-friendly messages
- [x] T019 Initialize FastAPI app in backend/src/api/main.py with middleware, CORS, and health endpoint routing
- [x] T020 Implement /health endpoint in backend/src/api/routes/health.py with Qdrant and Postgres connectivity checks

**Checkpoint**: Foundation ready - all RAG skills and user stories can now proceed in parallel

---

## Phase 3: User Story 1 - Basic Q&A Without Selection (Priority: P1) 🎯 MVP

**Goal**: Students can ask questions about textbook content and receive cited answers grounded in indexed chapters. This is the core MVP - delivers immediate value independently.

**Independent Test**: Ask "What is forward kinematics?" and verify answer includes inline citations linking to specific textbook sections, with no hallucinated content.

### RAG Skills Implementation for User Story 1

- [x] T021 [P] [US1] Implement rag-indexer skill: Create .claude/skills/rag-indexer/indexer.py with markdown chunking (512 tokens, 128 overlap)
- [x] T022 [P] [US1] Implement rag-indexer skill: Create .claude/skills/rag-indexer/chunker.py for semantic splitting by H2/H3 headings
- [x] T023 [P] [US1] Implement rag-indexer skill: Create .claude/skills/rag-indexer/embeddings.py for Qwen embedding generation via OpenRouter
- [x] T024 [P] [US1] Implement rag-indexer skill: Create .claude/skills/rag-indexer/SKILL.md with comprehensive documentation (inputs, outputs, examples)
- [x] T025 [P] [US1] Implement rag-retriever skill: Create .claude/skills/rag-retriever/retriever.py for semantic search over Qdrant (top-5, score >0.7)
- [x] T026 [P] [US1] Implement rag-retriever skill: Create .claude/skills/rag-retriever/SKILL.md with comprehensive documentation
- [x] T027 [P] [US1] Implement rag-answerer skill: Create .claude/skills/rag-answerer/answerer.py for LLM answer generation via OpenRouter with citation mapping
- [x] T028 [P] [US1] Implement rag-answerer skill: Create .claude/skills/rag-answerer/citation_mapper.py to map [N] markers to retrieved chunks
- [x] T029 [P] [US1] Implement rag-answerer skill: Create .claude/skills/rag-answerer/SKILL.md with comprehensive documentation
- [x] T030 [US1] Implement rag-manager skill: Create .claude/skills/rag-manager/manager.py to orchestrate pipeline (query → retrieve → answer → validate)
- [x] T031 [US1] Implement rag-manager skill: Create .claude/skills/rag-manager/validator.py to ensure all [N] citations map to actual chunks
- [x] T032 [US1] Implement rag-manager skill: Create .claude/skills/rag-manager/SKILL.md with comprehensive documentation

### Backend Services for User Story 1

- [x] T033 [US1] Implement RAG orchestrator in backend/src/services/rag_orchestrator.py to call rag-manager skill with error handling
- [x] T034 [US1] Implement conversation service in backend/src/services/conversation_service.py for Postgres CRUD (create conversation, add message, retrieve history)
- [x] T035 [US1] Implement POST /chat/query endpoint in backend/src/api/routes/chat.py (accept query, call orchestrator, return answer with citations)
- [x] T036 [US1] Implement GET /chat/history endpoint in backend/src/api/routes/chat.py (retrieve conversation by conversation_id)
- [x] T037 [US1] Add token limit enforcement in backend/src/services/rag_orchestrator.py to truncate context if >8K input tokens
- [x] T038 [US1] Add retry logic with exponential backoff in backend/src/services/rag_orchestrator.py for OpenRouter transient failures (max 3 retries)

### Frontend Components for User Story 1

- [x] T039 [P] [US1] Create ChatWidget component in src/components/RAGChatbot/ChatWidget.tsx (persistent floating button)
- [x] T040 [P] [US1] Create ChatInterface component in src/components/RAGChatbot/ChatInterface.tsx (modal UI with MessageList, InputBox)
- [x] T041 [P] [US1] Create MessageList component in src/components/RAGChatbot/MessageList.tsx (display conversation history with citations)
- [x] T042 [P] [US1] Create InputBox component in src/components/RAGChatbot/InputBox.tsx (user input field with submit handler)
- [x] T043 [P] [US1] Create CitationLink component in src/components/RAGChatbot/CitationLink.tsx (clickable [N] markers that scroll to source)
- [x] T044 [P] [US1] Create styles in src/components/RAGChatbot/styles.module.css (Docusaurus theme-consistent styling with dark mode support)
- [x] T045 [US1] Create Axios API client in src/api/chatbotClient.ts (wrapper for /chat/query, /chat/history endpoints with error handling)
- [x] T046 [US1] Create useChatHistory hook in src/hooks/useChatHistory.ts (manage conversation state, message list, loading states)
- [x] T047 [US1] Create Docusaurus plugin in src/plugins/chatbot-plugin/index.ts (register plugin with Docusaurus build)
- [x] T048 [US1] Implement theme swizzle in src/plugins/chatbot-plugin/theme/ChatbotWrapper/index.tsx (mount ChatWidget globally)

### Indexing for User Story 1

- [x] T049 [US1] Run rag-indexer skill to index all 21 textbook chapters (34 files total) into Qdrant with git commit hash metadata
- [x] T050 [US1] Verify Qdrant collection contains ~700-900 chunks with correct metadata (file_path, line_start, commit_hash, module, chapter)

**Checkpoint**: MVP complete! User Story 1 delivers standalone value - students can ask questions and get cited answers. Test independently before proceeding to US2.

**Validation Scenarios**:
1. Ask "What is forward kinematics?" → Verify cited answer with [1], [2] markers linking to textbook sections
2. Ask "What is quantum computing?" → Verify "I don't have sufficient information" response (out of scope)
3. Ask follow-up "Can you explain that more?" → Verify context retention from previous exchange
4. Ask 5 questions rapidly → Verify rate limiting triggers at 20 requests/min

---

## Phase 4: User Story 2 - Context-Aware Q&A with Selected Text (Priority: P2)

**Goal**: Students can highlight text and ask clarifying questions about that specific selection. Builds on P1 by adding precise context targeting.

**Independent Test**: Select paragraph about "ROS 2 topics", click "Ask about this", ask "Can you give me a practical example?" and verify answer incorporates selected text as primary context.

### Text Selection Implementation for User Story 2

- [x] T051 [P] [US2] Create SelectionHandler component in src/components/RAGChatbot/SelectionHandler.tsx using browser Selection API (detect mouseup events)
- [x] T052 [P] [US2] Create useTextSelection hook in src/hooks/useTextSelection.ts (manage selected text state, source location extraction)
- [x] T053 [US2] Implement button overlay injection in src/components/RAGChatbot/SelectionHandler.tsx ("Ask about this" button at cursor position)
- [x] T054 [US2] Update ChatInterface component in src/components/RAGChatbot/ChatInterface.tsx to display selected text snippet at conversation top
- [x] T055 [US2] Update POST /chat/query request model in backend/src/api/models/request.py to accept optional selected_text and source_location fields
- [x] T056 [US2] Update rag-retriever skill in .claude/skills/rag-retriever/retriever.py to prioritize chunks from same chapter as selected text source
- [x] T057 [US2] Update RAG orchestrator in backend/src/services/rag_orchestrator.py to combine query + selected_text for embedding generation
- [x] T058 [US2] Update token counter in backend/src/utils/token_counter.py to account for selected_text in 8K input limit calculation
- [x] T059 [US2] Integrate SelectionHandler with ChatWidget in src/plugins/chatbot-plugin/theme/ChatbotWrapper/index.tsx (detect selection → open chatbot)

**Checkpoint**: User Story 2 complete - text selection enables targeted clarifications. Test independently alongside US1.

**Validation Scenarios**:
1. Select code snippet → Ask "What does this line do?" → Verify answer references specific line from selection
2. Select mathematical formula → Ask "Walk through this step-by-step" → Verify breakdown maintains formula notation
3. No text selected → Open chatbot → Verify falls back to standard Q&A (US1 functionality)
4. Select 5000 characters → Verify truncation to 2000 chars with user notification

---

## Phase 5: User Story 3 - Multi-Turn Conversational Learning (Priority: P3)

**Goal**: Students engage in extended learning sessions with context retention, adaptive answers, and follow-up suggestions. Enhances engagement beyond basic Q&A.

**Independent Test**: Conduct multi-turn conversation ("What is forward kinematics?" → "How differs from inverse?" → "Show DH convention") and verify context retention across all turns.

### Conversation Context Implementation for User Story 3

- [x] T060 [US3] Update conversation service in backend/src/services/conversation_service.py to retrieve previous 5 Q&A exchanges for context window
- [x] T061 [US3] Update RAG orchestrator in backend/src/services/rag_orchestrator.py to inject conversation history into LLM prompt for follow-up resolution
- [x] T062 [US3] Implement follow-up suggestion generator in backend/src/services/rag_orchestrator.py using curriculum graph (2-3 recommendations per answer)
- [x] T063 [US3] Update ChatQueryResponse model in backend/src/api/models/response.py to include follow_up_suggestions array
- [x] T064 [US3] Update MessageList component in src/components/RAGChatbot/MessageList.tsx to display follow-up question chips below answers
- [x] T065 [US3] Implement "Start New Conversation" button in src/components/RAGChatbot/ChatInterface.tsx to clear context and create new conversation
- [x] T066 [US3] Add conversation turn counter in backend/src/services/conversation_service.py to track when context window approaches 10 turns
- [x] T067 [US3] Implement context summarization in backend/src/services/rag_orchestrator.py when conversation exceeds 10 turns (summarize key points)
- [x] T068 [US3] Update token counter in backend/src/utils/token_counter.py to account for conversation history in 8K input limit

**Checkpoint**: User Story 3 complete - multi-turn conversations enable natural dialogue and adaptive tutoring. Test independently with US1+US2.

**Validation Scenarios**:
1. Ask 3 related questions → Verify 4th question "Compare these approaches" correctly references prior exchanges
2. Progress from basic to advanced questions → Verify answer complexity adapts (beginner → intermediate → advanced style)
3. Complete understanding a concept → Verify 2-3 relevant next topics suggested based on curriculum flow
4. Exceed 10 turns → Verify summarization message appears suggesting new focused conversation

---

## Phase 6: User Story 4 - Feedback and Iterative Improvement (Priority: P4)

**Goal**: Students rate answers and provide feedback. Instructors review analytics to identify content gaps. Enables data-driven quality improvements.

**Independent Test**: Submit thumbs up/down on multiple answers, access admin dashboard, verify interaction analytics display correctly (questions by topic, satisfaction ratings).

### Feedback System Implementation for User Story 4

- [x] T069 [P] [US4] Create FeedbackButtons component in src/components/RAGChatbot/FeedbackButtons.tsx (thumbs up/down with optional comment field)
- [x] T070 [P] [US4] Implement feedback service in backend/src/services/feedback_service.py for Postgres CRUD (create feedback, query by filters)
- [x] T071 [US4] Implement POST /chat/feedback endpoint in backend/src/api/routes/feedback.py (accept message_id, rating, optional comment)
- [x] T072 [US4] Update MessageList component in src/components/RAGChatbot/MessageList.tsx to embed FeedbackButtons below each assistant message
- [x] T073 [US4] Update API client in src/api/chatbotClient.ts to add submitFeedback method with error handling
- [x] T074 [US4] Create admin dashboard skeleton in src/pages/admin/analytics.tsx (protected route, basic layout)
- [x] T075 [US4] Implement GET /admin/analytics endpoint in backend/src/api/routes/admin.py (aggregate queries: total, by topic, satisfaction rate, common unanswered)
- [x] T076 [US4] Implement interaction logger in backend/src/services/conversation_service.py to store: query, selected_context, retrieved_chunks, citations, response_time
- [x] T077 [US4] Create analytics dashboard visualizations in src/pages/admin/analytics.tsx (charts for question frequency, low satisfaction topics, content gaps)
- [x] T078 [US4] Implement content gap detector in backend/src/services/feedback_service.py (detect recurring "I don't know" patterns, generate report)

**Checkpoint**: User Story 4 complete - feedback loop enables continuous quality improvement. All P1-P4 features now functional.

**Validation Scenarios**:
1. Click thumbs up → Verify feedback logged with conversation_id, message_id, timestamp
2. Click thumbs down → Provide text feedback → Verify interaction flagged for instructor review
3. Access admin dashboard → Verify metrics: total queries, topics with low satisfaction, unanswered questions
4. Multiple students ask similar unanswered questions → Verify content gap report highlights missing coverage

---

## Phase 7: Git-Based Incremental Re-indexing & CI/CD

**Purpose**: Automate content synchronization between textbook updates and vector database

- [x] T079 [P] Update rag-indexer skill in .claude/skills/rag-indexer/indexer.py to support incremental mode (compare commit hashes, re-index only changed files)
- [x] T080 [P] Create GitHub Actions workflow in .github/workflows/reindex-on-commit.yml (trigger on push to main)
- [x] T081 Implement git diff detection in .github/workflows/reindex-on-commit.yml (detect changed .md files using git diff HEAD~1 HEAD)
- [x] T082 Configure workflow to invoke rag-indexer skill for changed files only in .github/workflows/reindex-on-commit.yml
- [x] T083 Implement chunk deletion logic in .claude/skills/rag-indexer/indexer.py (remove chunks from deleted files using file_path filter)
- [x] T084 Add commit hash update in .claude/skills/rag-indexer/embeddings.py to store current commit hash in Qdrant metadata
- [x] T085 Create manual re-index endpoint in backend/src/api/routes/admin.py (POST /admin/reindex?full=true with auth token requirement)
- [x] T086 Test incremental re-indexing: modify 1 chapter → commit → verify only that chapter re-indexed in <2 minutes

**Checkpoint**: CI/CD automation complete - content updates automatically propagate to vector DB without manual intervention.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and deployment readiness

- [x] T087 [P] Create backend deployment documentation in backend/README.md (Docker build, environment variables, health checks)
- [x] T088 [P] Create frontend integration documentation in src/plugins/chatbot-plugin/README.md (Docusaurus setup, plugin configuration)
- [x] T089 [P] Add API rate limiting dashboard in backend/src/api/routes/admin.py (monitor requests per user, identify abuse patterns)
- [x] T090 [P] Implement caching strategy in backend/src/services/rag_orchestrator.py for repeated queries (Redis optional, in-memory for MVP)
- [x] T091 [P] Add observability: structured logging in backend/src/api/main.py (request IDs, response times, error tracking)
- [x] T092 [P] Add monitoring alerts in backend/src/api/routes/health.py (Qdrant connectivity failures, Postgres connection pool exhaustion)
- [x] T093 [P] Security hardening: validate all user inputs in backend/src/utils/validators.py (SQL injection prevention, XSS prevention)
- [x] T094 [P] Performance optimization: add connection pooling tuning in backend/src/db/connection.py (10 concurrent connections minimum)
- [x] T095 [P] Create quickstart validation script to test end-to-end flow: index sample chapters → query → verify cited answer
- [x] T096 Update project README.md with architecture diagram, setup instructions, and usage examples
- [x] T097 Code cleanup: ensure all functions have docstrings with type hints, remove debug print statements
- [x] T098 Run final validation: test all 4 user stories independently + together, verify success criteria SC-001 through SC-010

**Checkpoint**: Production ready - all user stories implemented, tested, documented, and ready for deployment.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories ✅ MVP
  - User Story 2 (P2): Can start after Foundational - Extends US1 but independently testable
  - User Story 3 (P3): Can start after Foundational - Benefits from US1+US2 but independently testable
  - User Story 4 (P4): Can start after Foundational - Enhances US1-US3 but independently testable
- **Re-indexing CI/CD (Phase 7)**: Depends on US1 rag-indexer skill completion
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)** 🎯: Can start after Foundational (Phase 2) - **No dependencies on other stories** - This is the MVP
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Extends US1 with text selection but can be tested independently
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances with multi-turn context but can be tested independently
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Adds feedback layer but can be tested independently

### Within Each User Story

- RAG Skills (T021-T032 for US1) can run in parallel across skills (indexer, retriever, answerer, manager)
- Backend services depend on skills completion
- Frontend components marked [P] can run in parallel
- Integration tasks (API client, hooks) require both backend and frontend components

### Parallel Opportunities

- **Phase 1 Setup**: T003, T004, T005, T007, T008 (external services + config files) can all run in parallel
- **Phase 2 Foundational**: T012, T013, T014, T015, T016, T018 (different files) can run in parallel
- **US1 RAG Skills**: T021-T024 (indexer), T025-T026 (retriever), T027-T029 (answerer) can run in parallel per skill
- **US1 Frontend**: T039-T044 (all React components) can run in parallel
- **US2 Selection**: T051, T052 (hook and handler) can run in parallel
- **US4 Feedback**: T069, T070, T071 (component, service, endpoint) can run in parallel

---

## Parallel Example: User Story 1 (MVP)

```bash
# After Foundational phase completes, launch all RAG skills in parallel:
Task T021-T024: "Implement rag-indexer skill (indexer.py, chunker.py, embeddings.py, SKILL.md)"
Task T025-T026: "Implement rag-retriever skill (retriever.py, SKILL.md)"
Task T027-T029: "Implement rag-answerer skill (answerer.py, citation_mapper.py, SKILL.md)"

# Once skills done, launch backend services:
Task T033: "RAG orchestrator in backend/src/services/rag_orchestrator.py"
Task T034: "Conversation service in backend/src/services/conversation_service.py"

# Launch all frontend components in parallel:
Task T039: "ChatWidget in src/components/RAGChatbot/ChatWidget.tsx"
Task T040: "ChatInterface in src/components/RAGChatbot/ChatInterface.tsx"
Task T041: "MessageList in src/components/RAGChatbot/MessageList.tsx"
Task T042: "InputBox in src/components/RAGChatbot/InputBox.tsx"
Task T043: "CitationLink in src/components/RAGChatbot/CitationLink.tsx"
Task T044: "Styles in src/components/RAGChatbot/styles.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only) - Recommended for Initial Launch

1. Complete **Phase 1: Setup** (T001-T009) - 6 hours estimated
2. Complete **Phase 2: Foundational** (T010-T020) - 10 hours estimated
3. Complete **Phase 3: User Story 1** (T021-T050) - 40 hours estimated
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Verify: Ask questions → Get cited answers → No hallucinations → <3s response time
5. Deploy MVP and collect user feedback before proceeding to US2

**MVP Deployment Value**: Students can immediately ask questions and get high-quality cited answers. This delivers 80% of the value with 40% of the complexity.

### Incremental Delivery (After MVP Validation)

1. **Foundation + US1 (MVP)** → Test independently → **Deploy Week 6** ✅
2. Add **US2 (Text Selection)** → Test independently → **Deploy Week 8**
3. Add **US3 (Multi-Turn)** → Test independently → **Deploy Week 10**
4. Add **US4 (Feedback)** → Test independently → **Deploy Week 11**
5. Add **Phase 7 (CI/CD Re-indexing)** → **Deploy Week 12**
6. Polish (Phase 8) → **Final Release Week 13**

Each phase adds incremental value without breaking previous functionality.

### Parallel Team Strategy (If Multiple Developers Available)

With 3 developers after Foundational phase completes:

1. **Team completes Setup + Foundational together** (1-2 weeks)
2. Once Foundational is done, split work:
   - **Developer A**: User Story 1 (RAG skills + backend) - 3 weeks
   - **Developer B**: User Story 1 (Frontend components + integration) - 3 weeks
   - **Developer C**: User Story 2 (Text selection) + User Story 3 (Multi-turn) - 3 weeks
3. Week 4-5: Integration testing, US1 MVP deployment
4. Week 6-8: US2, US3, US4 parallel development
5. Week 9-10: CI/CD automation, polish, final release

---

## Task Summary

- **Total Tasks**: 98 tasks (T001-T098)
- **Phase 1 Setup**: 9 tasks (T001-T009)
- **Phase 2 Foundational**: 11 tasks (T010-T020) ⚠️ BLOCKS all user stories
- **Phase 3 User Story 1 (P1) MVP**: 30 tasks (T021-T050) 🎯
- **Phase 4 User Story 2 (P2)**: 9 tasks (T051-T059)
- **Phase 5 User Story 3 (P3)**: 9 tasks (T060-T068)
- **Phase 6 User Story 4 (P4)**: 10 tasks (T069-T078)
- **Phase 7 Re-indexing CI/CD**: 8 tasks (T079-T086)
- **Phase 8 Polish**: 12 tasks (T087-T098)

**Parallel Task Count**: 41 tasks marked [P] can run in parallel with others in same phase

**MVP Scope** (Recommended First Deployment):
- Phase 1 Setup (9 tasks)
- Phase 2 Foundational (11 tasks)
- Phase 3 User Story 1 (30 tasks)
- **Total MVP**: 50 tasks, estimated 56 hours (6-7 weeks with 1 developer)

**Success Criteria Mapping**:
- SC-001 (80% queries >0.7 similarity): Validated in T050 (indexing verification)
- SC-002 (<3s response time): Validated in T098 (final validation)
- SC-003 (70% task completion): Measured post-deployment via US4 analytics
- SC-004 (60% positive feedback): Measured via US4 feedback system (T069-T078)
- SC-005 (100 concurrent users): Load testing in T094 (performance optimization)
- SC-006 (Zero hallucinations): Validated by T031 (citation validator) + T098
- SC-007 (5+ content gaps identified): Measured via US4 analytics (T078)
- SC-008 (Chatbot loads <2s): Frontend performance testing in T044 + T098
- SC-009 (40% multi-turn engagement): Measured via US3 conversation tracking (T066)
- SC-010 (99.5% uptime): Monitored via T092 (health check alerts)

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Tests are not explicitly requested, so tasks focus on implementation with validation checkpoints
