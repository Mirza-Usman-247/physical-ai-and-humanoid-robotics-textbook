# Tasks: Authentication, Personalization, and Translation Integration

**Input**: Design documents from `/specs/003-auth-personalization-translation/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Tests are included for all user stories per TDD workflow (Red → Green → Refactor)

**Organization**: Tasks are grouped by user story (US1-US4) from spec.md to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story this task belongs to (US1, US2, US3, US4, or Foundational)
- File paths follow plan.md structure (frontend: `src/`, backend: `backend/src/`)

---

## Phase 0: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [X] **T001** [P] Install backend dependencies: `pip install passlib[bcrypt] python-jose[cryptography] python-multipart` (backend/requirements.txt) - Already installed
- [X] **T002** [P] Install frontend dependencies: `npm install better-auth zod idb` (package.json) - Completed
- [X] **T003** [P] Configure environment variables for backend (.env: JWT_SECRET_KEY, OPENROUTER_API_KEY, NEON_DATABASE_URL) - Added to .env.example
- [X] **T004** [P] Configure environment variables for frontend (.env.local: BETTER_AUTH_URL, BETTER_AUTH_SECRET) - Created .env.local.example
- [X] **T005** Create Alembic migration script for users and user_profiles tables (backend/src/db/migrations/versions/YYYYMMDD_add_user_auth_tables.py) - Created

**Checkpoint**: Dependencies installed, environment configured, ready for foundational infrastructure

---

## Phase 1: Foundational (Blocking Prerequisites)

**Purpose**: Core authentication infrastructure that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [X] **T006** Create User model with SQLAlchemy ORM (backend/src/db/models.py: User class with id, email, password_hash, is_active, created_at, updated_at) - Created as AuthUser
- [X] **T007** Create UserProfile model with SQLAlchemy ORM (backend/src/db/models.py: UserProfile class with user_id FK, skill levels, hardware flags) - Completed
- [X] **T008** Run Alembic migration to create users and user_profiles tables (`alembic upgrade head`) - Completed, tables created successfully
- [X] **T009** [P] Create JWT utilities module (backend/src/utils/jwt.py: create_access_token, create_refresh_token, verify_token functions) - Completed
- [X] **T010** [P] Create password hashing utilities (backend/src/utils/password.py: hash_password, verify_password with bcrypt cost factor 12) - Completed
- [X] **T011** Create JWT validation middleware (backend/src/api/middleware/auth.py: get_current_user dependency for FastAPI) - Completed
- [X] **T012** [P] Create Pydantic request/response models for auth (backend/src/api/models/request.py, response.py: SignupRequest, SigninRequest, SignupResponse, SigninResponse, UserProfile) - Completed

### Frontend Foundation

- [X] **T013** [P] Configure Better Auth client (src/lib/auth.ts: createAuth with Neon Postgres config, email/password enabled) - Completed
- [X] **T014** [P] Create auth TypeScript types (src/types/auth.ts: User, UserProfile, AuthTokens, SignupRequest, SigninRequest interfaces) - Completed
- [X] **T015** [P] Create auth API client (src/api/auth.ts: signup, signin, refresh, logout functions using Better Auth) - Completed
- [X] **T016** Create AuthProvider React context (src/components/Auth/AuthProvider.tsx: wraps app with Better Auth context) - Completed
- [X] **T017** Create useAuth custom hook (src/hooks/useAuth.ts: provides user, tokens, isAuthenticated, signin, signup, logout) - Completed
- [X] **T018** Update Docusaurus Root theme component to inject AuthProvider (src/theme/Root.tsx) - Completed

**✅ Checkpoint PASSED**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 2: User Story 1 - User Registration and Profile Setup (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with email/password and complete skill/hardware profile

**Independent Test**: Complete signup flow and verify user + profile stored in Neon Postgres database

### Tests for User Story 1 (Red Phase - Write failing tests first)

- [ ] **T019** [P] [US1] Contract test for POST /api/auth/signup endpoint (backend/tests/api/test_auth.py: test_signup_success, test_signup_duplicate_email, test_signup_invalid_password)
- [ ] **T020** [P] [US1] Integration test for full signup flow (backend/tests/integration/test_auth_flow.py: test_signup_creates_user_and_profile)
- [ ] **T021** [P] [US1] Unit test for password hashing (backend/tests/utils/test_password.py: test_hash_password, test_verify_password)
- [ ] **T022** [P] [US1] Unit test for JWT token generation (backend/tests/utils/test_jwt.py: test_create_access_token, test_create_refresh_token, test_verify_token)

### Implementation for User Story 1 (Green Phase - Make tests pass)

**Backend Implementation**

- [ ] **T023** [US1] Create auth service module (backend/src/services/auth_service.py: signup_user function - hash password, create user, create profile, return tokens)
- [ ] **T024** [US1] Create profile service module (backend/src/services/profile_service.py: get_profile, create_profile functions)
- [ ] **T025** [US1] Implement POST /api/auth/signup endpoint (backend/src/api/routes/auth.py: signup route calls auth_service.signup_user, returns SignupResponse)
- [ ] **T026** [US1] Implement POST /api/auth/signin endpoint (backend/src/api/routes/auth.py: signin route verifies password, returns SigninResponse with tokens)
- [ ] **T027** [US1] Implement POST /api/auth/refresh endpoint (backend/src/api/routes/auth.py: refresh route validates refresh token, returns new access + refresh tokens)
- [ ] **T028** [US1] Implement POST /api/auth/logout endpoint (backend/src/api/routes/auth.py: logout route - client-side token deletion, server returns success)
- [ ] **T029** [US1] Implement GET /api/profile endpoint with JWT middleware (backend/src/api/routes/profile.py: get_profile route requires authentication, returns UserProfile)
- [ ] **T030** [US1] Add auth router to FastAPI app (backend/src/api/main.py: app.include_router(auth.router, prefix="/api/auth"))
- [ ] **T031** [US1] Add profile router to FastAPI app (backend/src/api/main.py: app.include_router(profile.router, prefix="/api"))

**Frontend Implementation**

- [ ] **T032** [P] [US1] Create SignupForm component (src/components/Auth/SignupForm.tsx: email, password, 5 skill level inputs, hardware checkboxes, calls auth.signup)
- [ ] **T033** [P] [US1] Create SigninForm component (src/components/Auth/SigninForm.tsx: email, password inputs, calls auth.signin)
- [ ] **T034** [P] [US1] Create Zod validation schema for signup (src/components/Auth/SignupForm.tsx: signupSchema with email, password complexity, skill levels 1-5)
- [ ] **T035** [US1] Add "Sign Up" and "Sign In" buttons to Docusaurus navbar (src/theme/Navbar/index.tsx or docusaurus.config.js navbar customization)
- [ ] **T036** [US1] Create signup/signin modal or pages (src/pages/signup.tsx, src/pages/signin.tsx OR modal components)
- [ ] **T037** [US1] Implement JWT token storage in memory (src/hooks/useAuth.ts: store tokens in React state, refresh on mount if refresh token exists)
- [ ] **T038** [US1] Add error handling and user feedback for auth failures (src/components/Auth/: display validation errors, network errors, duplicate email errors)

**Checkpoint**: At this point, User Story 1 should be fully functional - users can signup, signin, and have their profile stored in database

---

## Phase 3: User Story 2 - Chapter Content Personalization (Priority: P2)

**Goal**: Enable logged-in users to personalize chapter content based on their skill profile

**Independent Test**: Login, click "Personalize Chapter", verify content adapts to user's skill levels and hardware access

### Tests for User Story 2 (Red Phase)

- [ ] **T039** [P] [US2] Contract test for POST /api/personalize endpoint (backend/tests/api/test_personalization.py: test_personalize_success, test_personalize_unauthorized, test_personalize_rate_limit)
- [ ] **T040** [P] [US2] Integration test for personalization flow (backend/tests/integration/test_personalization_flow.py: test_personalize_adapts_to_beginner_profile, test_personalize_adapts_to_advanced_profile)
- [ ] **T041** [P] [US2] Unit test for personalization service (backend/tests/services/test_personalization_service.py: test_build_personalization_prompt, test_parse_llm_response)

### Implementation for User Story 2 (Green Phase)

**Backend Implementation**

- [ ] **T042** [P] [US2] Create OpenRouter API client (backend/src/utils/openrouter.py: call_llm function with model selection, streaming support)
- [ ] **T043** [US2] Create personalization service (backend/src/services/personalization_service.py: personalize_chapter function - build prompt from profile, call OpenRouter Llama 3.3 70B, parse response)
- [ ] **T044** [US2] Build personalization prompt template (backend/src/services/personalization_service.py: prompt includes user skill levels, hardware access, instruction to adapt explanation depth, code complexity, hardware assumptions)
- [ ] **T045** [US2] Implement POST /api/personalize endpoint with JWT middleware (backend/src/api/routes/personalization.py: personalize route validates token, calls personalization_service, returns PersonalizeResponse)
- [ ] **T046** [US2] Add personalization router to FastAPI app (backend/src/api/main.py: app.include_router(personalization.router, prefix="/api"))
- [ ] **T047** [US2] Add rate limiting for personalization endpoint (backend/src/api/middleware/rate_limit.py: extend to track personalization API calls per user, queue requests if rate limit hit)
- [ ] **T048** [US2] Add error handling for OpenRouter API failures (backend/src/services/personalization_service.py: catch timeout, rate limit, API errors, return friendly messages)

**Frontend Implementation**

- [ ] **T049** [P] [US2] Create personalization API client (src/api/personalization.ts: personalizeChapter function sends chapter content + user profile)
- [ ] **T050** [P] [US2] Create personalization TypeScript types (src/types/personalization.ts: PersonalizeRequest, PersonalizeResponse, TransformationMetadata)
- [ ] **T051** [P] [US2] Create PersonalizeButton component (src/components/ChapterActions/PersonalizeButton.tsx: onClick triggers personalization, shows loading indicator, stores result in IndexedDB)
- [ ] **T052** [P] [US2] Create IndexedDB storage utility (src/plugins/sessionStorage.ts: TransformedContentStore schema, saveTransformedContent, getTransformedContent, clearOnLogout functions)
- [ ] **T053** [US2] Create usePersonalization custom hook (src/hooks/usePersonalization.ts: handles personalization request, loading state, error state, IndexedDB storage)
- [ ] **T054** [US2] Inject PersonalizeButton into chapter pages (modify Docusaurus MDX rendering or create wrapper component that checks auth status, displays button at chapter start)
- [ ] **T055** [US2] Create ContentTransformer component (src/components/ChapterActions/ContentTransformer.tsx: renders personalized content from IndexedDB, replaces original MDX content, shows "View Original" button)
- [ ] **T056** [US2] Add loading indicators for personalization (10-second wait, progress spinner, estimated time display)
- [ ] **T057** [US2] Handle personalization errors gracefully (display error message, option to retry, fallback to original content)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - users can signup, signin, and personalize chapters

---

## Phase 4: User Story 3 - Chapter Translation to Urdu (Priority: P3)

**Goal**: Enable logged-in users to translate entire chapter to Urdu with technical faithfulness

**Independent Test**: Login, click "Translate to Urdu", verify entire chapter translated with technical terms preserved

### Tests for User Story 3 (Red Phase)

- [ ] **T058** [P] [US3] Contract test for POST /api/translate endpoint (backend/tests/api/test_translation.py: test_translate_success, test_translate_unauthorized, test_translate_invalid_language)
- [ ] **T059** [P] [US3] Integration test for translation flow (backend/tests/integration/test_translation_flow.py: test_translate_preserves_code_snippets, test_translate_preserves_technical_terms)
- [ ] **T060** [P] [US3] Unit test for translation service (backend/tests/services/test_translation_service.py: test_build_translation_prompt_focus_mode, test_extract_preserved_terms)

### Implementation for User Story 3 (Green Phase)

**Backend Implementation**

- [ ] **T061** [US3] Create translation service (backend/src/services/translation_service.py: translate_chapter function - build Focus Mode prompt, call OpenRouter Gemini 2.0 Flash, parse Urdu output, extract preserved terms)
- [ ] **T062** [US3] Build translation prompt template with Focus Mode (backend/src/services/translation_service.py: prompt instructs faithful translation, no extra commentary, preserve technical terms in English or transliterate, preserve code blocks)
- [ ] **T063** [US3] Implement POST /api/translate endpoint with JWT middleware (backend/src/api/routes/translation.py: translate route validates token, calls translation_service, returns TranslateResponse)
- [ ] **T064** [US3] Add translation router to FastAPI app (backend/src/api/main.py: app.include_router(translation.router, prefix="/api"))
- [ ] **T065** [US3] Add rate limiting for translation endpoint (backend/src/api/middleware/rate_limit.py: track translation API calls per user, same rate limit as personalization)
- [ ] **T066** [US3] Add error handling for translation API failures (backend/src/services/translation_service.py: handle Urdu generation errors, timeout, malformed output)

**Frontend Implementation**

- [ ] **T067** [P] [US3] Create translation API client (src/api/translation.ts: translateChapter function sends chapter content, targetLanguage="ur", mode="focus")
- [ ] **T068** [P] [US3] Create translation TypeScript types (src/types/translation.ts: TranslateRequest, TranslateResponse, TranslationMetadata)
- [ ] **T069** [P] [US3] Create TranslateButton component (src/components/ChapterActions/TranslateButton.tsx: onClick triggers translation, shows loading indicator 15s, stores result in IndexedDB)
- [ ] **T070** [US3] Create useTranslation custom hook (src/hooks/useTranslation.ts: handles translation request, loading state, error state, IndexedDB storage with key userId:chapterId:translation)
- [ ] **T071** [US3] Inject TranslateButton into chapter pages (same injection point as PersonalizeButton, check auth status, display at chapter start)
- [ ] **T072** [US3] Update ContentTransformer to render Urdu content (src/components/ChapterActions/ContentTransformer.tsx: detect translation type, apply RTL styling for Urdu, preserve English code snippets)
- [ ] **T073** [US3] Add "View Original" button for translated content (toggles between Urdu and English)
- [ ] **T074** [US3] Enforce one-transformation-at-a-time constraint (disable Personalize if Translate active, disable Translate if Personalize active, show message to choose one)

**Checkpoint**: All user stories 1, 2, 3 should now be independently functional

---

## Phase 5: User Story 4 - RAG Chatbot Context Awareness (Priority: P2)

**Goal**: RAG chatbot answers reflect personalized/translated content user is currently viewing

**Independent Test**: Personalize/translate chapter, ask RAG chatbot question, verify answer reflects customized content

### Tests for User Story 4 (Red Phase)

- [ ] **T075** [P] [US4] Integration test for RAG with personalized context (backend/tests/integration/test_rag_with_context.py: test_chatbot_uses_personalized_context, test_chatbot_answers_in_urdu_for_translated_content)
- [ ] **T076** [P] [US4] Unit test for context chunk extraction (frontend tests or backend tests: test_extract_relevant_sections_from_transformed_content)

### Implementation for User Story 4 (Green Phase)

**Backend Implementation**

- [ ] **T077** [US4] Modify RAG orchestrator to accept optional context_chunks parameter (backend/src/services/rag_orchestrator.py: update query method signature, add context_chunks: Optional[List[str]] parameter)
- [ ] **T078** [US4] Update RAG orchestrator logic to use provided context (backend/src/services/rag_orchestrator.py: if context_chunks provided → pass to rag-answerer; else → fallback to Qdrant vector search)
- [ ] **T079** [US4] Modify chat API endpoint to accept context chunks (backend/src/api/routes/chat.py: update ChatQueryRequest model to include optional contextChunks field)
- [ ] **T080** [US4] Pass context chunks from chat endpoint to RAG orchestrator (backend/src/api/routes/chat.py: extract contextChunks from request, pass to rag_orchestrator.query)

**Frontend Implementation**

- [ ] **T081** [P] [US4] Create smart chunking utility (src/utils/chunking.ts: extractRelevantSections function extracts sections from chapter based on question keywords, fits within 128K token limit)
- [ ] **T082** [US4] Modify RAG chatbot API client to send context chunks (src/api/rag.ts: update chatQuery function to include contextChunks parameter from IndexedDB)
- [ ] **T083** [US4] Update RAG chatbot component to retrieve transformed content (src/components/RAGChatbot/ChatInterface.tsx: on question submission, check IndexedDB for personalized/translated content, extract relevant chunks, send with query)
- [ ] **T084** [US4] Handle edge case: context chunks exceed request size limit (src/utils/chunking.ts: intelligently prioritize most relevant sections, truncate if needed, warn user if content too large)
- [ ] **T085** [US4] Add visual indicator in chatbot showing context type (src/components/RAGChatbot/ChatInterface.tsx: display badge "Using personalized content" or "Using Urdu translation" or "Using original content")

**Checkpoint**: All 4 user stories complete and integrated - RAG chatbot now context-aware

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Quality improvements, security hardening, documentation

**Security & Performance**

- [ ] **T086** [P] Add input validation for all API endpoints (backend/src/api/routes/: Pydantic models validate email format, password complexity, skill levels 1-5, hardware flags boolean)
- [ ] **T087** [P] Add HTTPS-only enforcement in production (backend/src/api/main.py: CORS middleware restrict to HTTPS origins)
- [ ] **T088** [P] Implement request logging for auth and transformation endpoints (backend/src/api/middleware/: log user_id, endpoint, timestamp, response status)
- [ ] **T089** [P] Add OpenRouter API key rotation mechanism (backend/src/utils/openrouter.py: support multiple API keys, rotate on rate limit)
- [ ] **T090** [P] Optimize IndexedDB queries (src/plugins/sessionStorage.ts: add indexes for userId and chapterId, batch writes)

**Documentation & Validation**

- [ ] **T091** [P] Update quickstart.md with final setup instructions (specs/003-auth-personalization-translation/quickstart.md: verify all steps accurate, add troubleshooting for common errors)
- [ ] **T092** [P] Create API documentation with Swagger UI examples (backend/src/api/main.py: ensure Swagger UI displays all endpoints with request/response examples)
- [ ] **T093** [P] Add inline code comments for security-critical functions (backend/src/utils/password.py, jwt.py: explain why bcrypt cost factor 12, why 15-min access token expiry)
- [ ] **T094** Run quickstart.md validation end-to-end (follow setup guide from scratch, verify all endpoints work)

**Testing & QA**

- [ ] **T095** [P] End-to-end test: Signup → Personalize → Chatbot query (tests/e2e/test_full_flow.py: verify entire user journey works)
- [ ] **T096** [P] End-to-end test: Signup → Translate → Chatbot query in Urdu (tests/e2e/test_translation_flow.py)
- [ ] **T097** [P] Performance test: 100 concurrent users signup (tests/performance/test_auth_load.py: verify SC-009 - 100 concurrent users without degradation)
- [ ] **T098** [P] Performance test: Personalization <10s (tests/performance/test_personalization_latency.py: verify SC-002)
- [ ] **T099** [P] Performance test: Translation <15s (tests/performance/test_translation_latency.py: verify SC-003)
- [ ] **T100** [P] Performance test: RAG chatbot <3s with transformed context (tests/performance/test_rag_latency.py: verify SC-006)
- [ ] **T101** Generate QA report (document test results, performance metrics, success criteria validation, any failures/issues)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 0)**: No dependencies - can start immediately
- **Foundational (Phase 1)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 2-5)**: All depend on Foundational phase completion
  - US1 (P1) → US2 (P2): US2 depends on US1 (needs authentication to personalize)
  - US1 (P1) → US3 (P3): US3 depends on US1 (needs authentication to translate)
  - US1 (P1) + US2/US3 → US4 (P2): US4 depends on US1 + (US2 OR US3) for transformed content
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 1) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after US1 complete (needs authentication) - Independently testable
- **User Story 3 (P3)**: Can start after US1 complete (needs authentication) - Independently testable
- **User Story 4 (P2)**: Can start after US1 + (US2 OR US3) complete (needs transformed content to test integration)

### Parallel Opportunities

- **Phase 0 (Setup)**: All tasks T001-T005 can run in parallel [P]
- **Phase 1 (Foundational)**: T006-T007 (models) parallel, T009-T010 (utils) parallel, T012 (Pydantic models) parallel, T013-T015 (frontend) parallel
- **Phase 2 (US1 Tests)**: T019-T022 all tests can run in parallel [P]
- **Phase 2 (US1 Frontend)**: T032-T034 (components) can run in parallel [P]
- **Phase 3 (US2 Tests)**: T039-T041 all tests can run in parallel [P]
- **Phase 3 (US2 Backend)**: T042-T043 can run in parallel [P]
- **Phase 3 (US2 Frontend)**: T049-T052 can run in parallel [P]
- **Phase 4 (US3 Tests)**: T058-T060 all tests can run in parallel [P]
- **Phase 4 (US3 Frontend)**: T067-T069 can run in parallel [P]
- **Phase 5 (US4 Tests)**: T075-T076 can run in parallel [P]
- **Phase 5 (US4 Frontend)**: T081-T082 can run in parallel [P]
- **Phase 6 (Polish)**: T086-T093, T095-T100 can all run in parallel [P]

---

## Implementation Strategy

### MVP First (P1 Only)

1. Complete Phase 0: Setup (T001-T005)
2. Complete Phase 1: Foundational (T006-T018) - CRITICAL, blocks all stories
3. Complete Phase 2: User Story 1 - Authentication (T019-T038)
4. **STOP and VALIDATE**: Test User Story 1 independently (users can signup, signin, profile stored)
5. Deploy/demo if ready → **MVP: User authentication functional**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Auth) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Personalization) → Test independently → Deploy/Demo
4. Add User Story 3 (Translation) → Test independently → Deploy/Demo
5. Add User Story 4 (RAG Integration) → Test independently → Deploy/Demo
6. Complete Phase 6 (Polish) → Final QA → Production deployment

### TDD Workflow (Red → Green → Refactor)

For each user story:
1. **Red Phase**: Write failing tests first (T019-T022 for US1, T039-T041 for US2, etc.)
2. **Green Phase**: Implement minimum code to make tests pass (T023-T038 for US1, T042-T057 for US2, etc.)
3. **Refactor Phase**: Clean up code, optimize, add comments (happens during Phase 6 or inline)

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [US1], [US2], [US3], [US4] = User Story labels for traceability to spec.md
- Each user story independently completable and testable (validates spec requirement)
- Verify tests fail (Red) before implementing (Green) - TDD workflow
- Commit after each task or logical group (T001-T005 setup, T019-T022 US1 tests, etc.)
- Stop at any checkpoint to validate story independently before proceeding
- Rate limiting enforced: 20 req/min OpenRouter (may need queuing if >20 concurrent users)
- Session storage: Clear IndexedDB on logout (T037 auth implementation handles this)

**Total Tasks**: 101 tasks across 6 phases
**Estimated Effort** (rough):
- Phase 0 (Setup): 2 hours
- Phase 1 (Foundational): 8 hours
- Phase 2 (US1 Auth): 12 hours
- Phase 3 (US2 Personalization): 10 hours
- Phase 4 (US3 Translation): 8 hours
- Phase 5 (US4 RAG Integration): 6 hours
- Phase 6 (Polish): 6 hours
- **Total: ~52 hours** (MVP: Phase 0-2 = ~22 hours)
