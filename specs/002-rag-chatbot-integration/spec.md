# Feature Specification: RAG Chatbot Integration for Physical AI Textbook

**Feature Branch**: `002-rag-chatbot-integration`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "Project Specification: RAG Chatbot Integration in Physical AI & Humanoid Robotics Book - Integrate a Retrieval-Augmented Generation (RAG) chatbot inside the published textbook site. The chatbot answers user queries based on the book content, including user-selected text. Use OpenRouter API and Qwen embeddings for vectorization and retrieval."

## Clarifications

### Session 2025-12-16

- Q: How should user-selected text be captured and passed to the chatbot from the Docusaurus frontend? → A: Use Docusaurus plugin hooks to intercept text selection and inject chatbot trigger button
- Q: What is the maximum token limit per request to the OpenRouter API (input + output combined)? → A: 8,000 input tokens + 2,000 output tokens per request
- Q: What should the chatbot return when retrieval fails or embeddings are unavailable? → A: Return friendly message explaining the issue + suggest alternative actions (e.g., "Unable to search textbook right now. Try rephrasing your question or check back in a moment.")
- Q: What are the role boundaries between rag-manager and the sub-skills (retriever/indexer/answerer)? → A: rag-manager orchestrates the pipeline (query → retrieve → answer) but delegates all domain logic to sub-skills; it only handles sequencing, validation, and error aggregation
- Q: What versioning strategy should be used for book content updates and keeping the vector database synchronized? → A: Git-based versioning: Track git commit hash in vector DB metadata; re-index automatically on new commits to main branch using CI/CD hooks

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Basic Q&A Without Selection (Priority: P1)

A student reads the textbook online and has a question about a concept they just read. They open the chatbot interface, type their question (e.g., "What is forward kinematics?"), and receive an answer grounded in the textbook content with citations showing which chapters/sections the answer came from.

**Why this priority**: This is the core MVP functionality - basic question answering from indexed content. Without this, the chatbot has no value. It delivers immediate utility and can be independently deployed.

**Independent Test**: Can be fully tested by asking questions about any indexed textbook chapter and verifying answers are accurate, cited, and grounded in book content. Delivers standalone value for students seeking quick clarifications.

**Acceptance Scenarios**:

1. **Given** a student is reading the textbook online and has a question, **When** they type "What is forward kinematics?" into the chatbot, **Then** the chatbot retrieves relevant content from the vector database and generates an answer with inline citations linking to specific textbook sections
2. **Given** the chatbot receives a question outside the textbook scope (e.g., "What is quantum computing?"), **When** the retrieval system finds no relevant content above the similarity threshold, **Then** the chatbot responds with "I don't have sufficient information in the textbook to answer this question" and suggests related topics that are covered
3. **Given** multiple chapters discuss the same concept, **When** a student asks about that concept, **Then** the chatbot synthesizes information from multiple sources and cites all relevant sections
4. **Given** the student asks a follow-up question, **When** the chatbot context window retains previous conversation history, **Then** answers maintain coherence and reference earlier exchanges

---

### User Story 2 - Context-Aware Q&A with Selected Text (Priority: P2)

A student highlights a specific paragraph or code snippet in the textbook, then clicks a "Ask about this" button. The chatbot opens with the selected text pre-loaded as context, allowing the student to ask clarifying questions specifically about that selection (e.g., "Can you explain this formula in simpler terms?" or "Why is this approach used here?").

**Why this priority**: Builds on P1 by adding precise context targeting. Students often need clarification on specific passages, diagrams, or code. This dramatically improves answer relevance by grounding both the question and retrieval in user-selected context.

**Independent Test**: Can be tested by selecting various types of textbook content (text, code, formulas, tables) and verifying the chatbot correctly incorporates the selection into its context retrieval and answer generation. Delivers value independently for targeted clarifications.

**Acceptance Scenarios**:

1. **Given** a student selects a paragraph about ROS 2 topics, **When** they click "Ask about this" and ask "Can you give me a practical example?", **Then** the chatbot uses the selected text as primary context and retrieves additional examples from the indexed content
2. **Given** a student selects a code snippet, **When** they ask "What does this line do?", **Then** the chatbot references the specific code line from the selection and explains it with context from surrounding documentation
3. **Given** a student selects a mathematical formula, **When** they ask "Can you walk through this step-by-step?", **Then** the chatbot provides a detailed breakdown while maintaining the formula notation from the selection
4. **Given** no text is selected, **When** the student opens the chatbot, **Then** it operates in standard Q&A mode without selection context (falls back to P1 functionality)

---

### User Story 3 - Multi-Turn Conversational Learning (Priority: P3)

A student engages in an extended learning session with the chatbot, asking a series of related questions that build on each other (e.g., starting with "What is forward kinematics?" → "How does it differ from inverse kinematics?" → "Can you show me the Denavit-Hartenberg convention?"). The chatbot maintains conversation context, progressively adapts answer complexity based on student understanding, and suggests relevant follow-up topics.

**Why this priority**: Enhances the learning experience by enabling natural dialogue and adaptive tutoring. While valuable, it requires P1 (basic Q&A) and benefits from P2 (context awareness) to be truly effective. This is a nice-to-have that improves engagement but isn't essential for MVP.

**Independent Test**: Can be tested by conducting multi-turn conversations on various topics and verifying context retention, answer coherence across turns, and appropriate follow-up suggestions. Delivers standalone value as an intelligent tutoring interface.

**Acceptance Scenarios**:

1. **Given** a student has asked 3 questions about kinematics in the current session, **When** they ask "Can you compare these two approaches?", **Then** the chatbot references previous exchanges and synthesizes a comparison
2. **Given** a student's questions progress from basic to advanced on a topic, **When** the chatbot analyzes question complexity patterns, **Then** it adapts answer detail level (beginner → intermediate → advanced style)
3. **Given** a student completes understanding a concept, **When** the chatbot generates follow-up suggestions, **Then** it recommends 2-3 relevant next topics based on curriculum flow and prerequisite relationships
4. **Given** a conversation exceeds 10 turns, **When** the context window approaches limits, **Then** the chatbot summarizes key points and suggests starting a new focused conversation

---

### User Story 4 - Feedback and Iterative Improvement (Priority: P4)

Students can rate chatbot answers (thumbs up/down) and optionally provide feedback on why an answer was helpful or unhelpful. Instructors can review flagged interactions, see common misconceptions, and identify gaps in textbook coverage. The system logs all interactions for quality monitoring and continuous improvement.

**Why this priority**: Essential for long-term quality but not required for initial launch. Enables data-driven improvements, quality assurance, and identification of content gaps. Should be added after P1-P3 are stable and user adoption begins.

**Independent Test**: Can be tested by submitting feedback on various answers, reviewing admin dashboards showing interaction patterns, and verifying data persistence and retrieval. Delivers standalone value for quality monitoring and content improvement workflows.

**Acceptance Scenarios**:

1. **Given** a student receives a chatbot answer, **When** they click thumbs up/down, **Then** the feedback is logged with conversation ID, question, answer, and timestamp
2. **Given** a student marks an answer as unhelpful, **When** they optionally provide text feedback, **Then** the system stores the feedback and flags the interaction for instructor review
3. **Given** an instructor accesses the admin dashboard, **When** they view interaction analytics, **Then** they see metrics including questions asked frequency, topics with lowest satisfaction ratings, and common unanswered questions
4. **Given** multiple students ask similar unanswered questions, **When** the system detects recurring "I don't know" patterns, **Then** it generates a content gap report highlighting missing textbook coverage

---

### Edge Cases

- **What happens when a student's question is ambiguous or contains typos?** The chatbot should attempt to interpret the intent, possibly suggest corrections ("Did you mean 'forward kinematics'?"), and if still unclear, ask the student to rephrase.

- **How does the system handle extremely long selected text (multiple pages)?** The system should truncate or chunk the selection intelligently, prioritizing the most relevant portions based on semantic density, and inform the user that only a portion was used as context (maximum 2000 characters processed).

- **What happens when the vector database is unavailable or returns errors?** The chatbot should display a user-friendly error message explaining the issue and suggesting alternative actions (e.g., "Unable to search textbook right now. Try rephrasing your question or check back in a moment."), then log the failure for monitoring and alerting.

- **What happens when multiple students query the same question simultaneously?** The system should handle concurrent requests efficiently through proper database connection pooling and caching strategies, with no degradation in response times for individual users.

- **How does the system prevent prompt injection or malicious queries?** Input sanitization and content filtering should be applied before processing queries, and suspicious patterns (e.g., attempts to manipulate system prompts) should be logged and blocked.

- **What happens when the indexed textbook content becomes stale after updates?** The system automatically detects content changes via git commit hash comparison; CI/CD hooks trigger incremental re-indexing on new commits to main branch, updating only changed files and removing chunks from deleted files without full downtime.

## Requirements *(mandatory)*

### Functional Requirements

#### Core Chat Interface (P1)

- **FR-001**: The chatbot interface MUST be embedded in the Docusaurus textbook site and accessible from every page via a persistent button or widget
- **FR-002**: Users MUST be able to type natural language questions up to 500 words in length
- **FR-003**: The chatbot MUST display responses with inline citations formatted as clickable links that scroll to the relevant textbook section
- **FR-004**: The system MUST show typing indicators while processing queries and generating responses
- **FR-005**: Users MUST be able to view conversation history for the current session (minimum 20 turns)
- **FR-006**: The chatbot MUST respond within 5 seconds for 95% of queries (measured from user submission to first token displayed)

#### Retrieval-Augmented Generation (P1)

- **FR-007**: The system MUST retrieve top 5 most relevant text chunks from Qdrant based on semantic similarity to the user query using Qwen embeddings
- **FR-008**: The system MUST apply a similarity threshold of 0.7 (adjustable) and return "I don't know" responses when no chunks exceed this threshold
- **FR-009**: Retrieved chunks MUST include metadata: file path, section titles (H1-H3), line numbers, and last modified timestamp
- **FR-010**: The answer generation MUST use OpenRouter API with prompt injection of retrieved context chunks (verbatim, numbered for citation mapping), respecting a maximum of 8,000 input tokens and 2,000 output tokens per request
- **FR-011**: Generated answers MUST include inline citation markers [1], [2], etc. that map to specific retrieved chunks
- **FR-012**: The system MUST validate that all citation markers in answers correspond to actual retrieved chunks (no hallucinated citations)
- **FR-013**: The system MUST truncate or reduce retrieved context chunks if the total input (query + selected text + context + prompt) exceeds 8,000 tokens, prioritizing the most relevant chunks

#### Context-Aware Selection (P2)

- **FR-014**: Users MUST be able to highlight text on any textbook page and trigger the chatbot via a button overlay injected using Docusaurus plugin hooks that intercept text selection events
- **FR-015**: Selected text (up to 2000 characters) MUST be passed to the chatbot as primary context along with its source location
- **FR-016**: The retrieval system MUST prioritize chunks from the same chapter/section as the selected text when user queries reference "this" or "here"
- **FR-017**: The chatbot interface MUST display the selected text snippet at the top of the conversation for user reference

#### Conversation Context (P3)

- **FR-018**: The system MUST retain the previous 5 Q&A exchanges in the conversation context window
- **FR-019**: Follow-up questions (e.g., "Can you explain that more?") MUST resolve references using conversation history
- **FR-020**: The system MUST generate 2-3 relevant follow-up question suggestions after each answer based on curriculum progression
- **FR-021**: Users MUST be able to clear conversation history manually via a "Start New Conversation" button

#### Feedback and Monitoring (P4)

- **FR-022**: Users MUST be able to rate each chatbot answer with thumbs up/down
- **FR-023**: Users MUST have the option to provide free-text feedback (up to 500 characters) explaining their rating
- **FR-024**: The system MUST log all interactions including: user query, selected context (if any), retrieved chunks, generated answer, citations, response time, and feedback
- **FR-025**: Instructors MUST be able to access an admin dashboard showing interaction analytics: total queries, questions by topic, average satisfaction, and common unanswered queries

#### Backend and Data Management (P1)

- **FR-026**: The FastAPI backend MUST expose RESTful endpoints for: `/chat/query` (POST), `/chat/history` (GET), `/chat/feedback` (POST), `/health` (GET)
- **FR-027**: All chat conversations MUST be stored in Neon Serverless Postgres with tables for: users, conversations, messages, feedback
- **FR-028**: Vector embeddings MUST be stored in Qdrant Cloud Free Tier with collections organized by textbook version (e.g., `textbook-v1-0`)
- **FR-029**: The system MUST support git-based incremental re-indexing: track git commit hash in vector DB chunk metadata, automatically re-index changed files on new commits to main branch via CI/CD hooks, and remove chunks from deleted files
- **FR-030**: All API endpoints MUST implement rate limiting (20 requests per minute per user) to prevent abuse

#### Error Handling and Reliability (P1)

- **FR-031**: When retrieval fails or embeddings are unavailable, the system MUST return a user-friendly error message that explains the issue and suggests alternative actions (e.g., "Unable to search textbook right now. Try rephrasing your question or check back in a moment.")
- **FR-032**: All system errors (database failures, API timeouts, embedding generation errors) MUST be logged with context (user query, timestamp, error type, stack trace) for monitoring and debugging
- **FR-033**: The system MUST implement retry logic with exponential backoff for transient failures (max 3 retries) before returning an error to the user

#### Reusable RAG Skills (All Priorities)

- **FR-034**: The `rag-retriever` skill MUST accept query text and return ranked chunks with scores and metadata, operating independently without dependencies on other skills
- **FR-035**: The `rag-indexer` skill MUST chunk markdown content semantically (by H2/H3 headings, 512 tokens with 128 overlap) and generate Qwen embeddings, operating independently during indexing workflows
- **FR-036**: The `rag-answerer` skill MUST generate answers with citations given a query and retrieved context chunks, operating independently with only the chunks as input
- **FR-037**: The `rag-manager` skill MUST orchestrate the pipeline (query → retrieve → answer → validate citations) by delegating all domain logic to sub-skills; it MUST only handle sequencing, validation, and error aggregation without duplicating retrieval or generation logic
- **FR-038**: All skills MUST be packaged as Claude Code skills with SKILL.md documentation and independent test suites for portability to future projects

### Key Entities

- **User**: Represents a student or reader interacting with the chatbot. Attributes include: user ID (session-based anonymous), session ID, interaction timestamp, conversation history.

- **Conversation**: Represents a multi-turn dialogue session. Attributes include: conversation ID, user session ID, start timestamp, end timestamp, message count, status (active/archived).

- **Message**: Represents a single Q&A exchange within a conversation. Attributes include: message ID, conversation ID, role (user/assistant), content text, selected context (if any), retrieved chunks (with metadata), citations, timestamp, response time.

- **Feedback**: Represents user evaluation of a chatbot answer. Attributes include: feedback ID, message ID, rating (positive/negative), free-text comment, timestamp.

- **TextbookChunk**: Represents an indexed segment of textbook content in the vector database. Attributes include: chunk ID, content text, embedding vector (Qwen-generated), file path, section titles (H1-H3), line range, last indexed timestamp, git commit hash (for version tracking and incremental updates), version tag.

- **RetrievalResult**: Represents a chunk returned by semantic search. Attributes include: chunk reference, similarity score, rank position, metadata (from TextbookChunk).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 80% of student queries receive answers with similarity scores above 0.7 (indicating relevant textbook content was found)
- **SC-002**: Average response time from query submission to answer display is under 3 seconds for 95th percentile
- **SC-003**: Students successfully complete their learning task (finding information, clarifying concept) on first chatbot interaction 70% of the time
- **SC-004**: 60% of chatbot answers receive positive feedback (thumbs up) from students within the first month
- **SC-005**: The chatbot handles 100 concurrent users without response time degradation beyond 10%
- **SC-006**: Zero hallucinated citations - 100% of citation markers in answers map to actual retrieved chunks
- **SC-007**: Instructors identify 5+ content gaps or improvement areas from chatbot interaction analytics within first 3 months
- **SC-008**: The chatbot interface loads within 2 seconds on the textbook site for 95% of page loads
- **SC-009**: Students ask at least 2 follow-up questions per conversation 40% of the time (indicating engagement with multi-turn dialogue)
- **SC-010**: The system maintains 99.5% uptime during peak usage hours (academic semester weeks)

### Quality Indicators

- **Grounding**: All answers are verifiable against indexed textbook content (no fabricated information)
- **Relevance**: Retrieved chunks directly address the user query (measured by manual review of 100 sample interactions)
- **Clarity**: Answers are understandable by the target audience (beginner to intermediate students) without requiring additional clarification
- **Completeness**: Answers address all parts of multi-part questions and synthesize information from multiple sources when needed

## Assumptions

1. **Target Audience**: Primary users are university-level students studying physical AI and robotics (undergraduate and graduate), with secondary users being instructors and researchers.

2. **Textbook Format**: The textbook is published as a Docusaurus site with markdown/MDX content organized in hierarchical folders (modules > chapters > sections), with consistent frontmatter metadata.

3. **Embedding Model**: Qwen embeddings are suitable for technical robotics content and provide sufficient semantic understanding for retrieval tasks. Dimension size and API access are available through OpenRouter.

4. **Authentication**: Initial version uses anonymous sessions (no login required) with session-based tracking. User authentication for persistent history is a future enhancement.

5. **Language**: Primary support is for English queries and content. Multilingual support can be added later based on demand.

6. **Deployment Environment**: The FastAPI backend will be deployed as a containerized service on cloud infrastructure with managed services (Neon Postgres, Qdrant Cloud).

7. **Content Update Frequency**: Textbook content updates occur weekly or less frequently, allowing scheduled re-indexing rather than real-time incremental updates.

8. **Cost Constraints**: The solution operates within the free tiers of Qdrant Cloud (1GB limit) and Neon Postgres (reasonable query volumes for student usage).

9. **Data Retention**: Conversation logs are retained for 90 days for analytics and debugging, then archived or anonymized per data privacy practices.

10. **Mobile Support**: The chatbot interface must be mobile-responsive but does not require a native mobile app—web interface accessed via mobile browsers is sufficient.

11. **LLM Model Selection**: The system will use a cost-effective yet high-quality model (defaulting to GPT-3.5 Turbo or Claude 3.5 Sonnet) via OpenRouter API, with the ability to upgrade to GPT-4 for complex queries if needed.

## Dependencies

- **External**: Existing RAG skills suite (rag-indexer, rag-retriever, rag-answerer, rag-manager) must be functional and tested
- **External**: Docusaurus site must have a plugin/component integration mechanism for embedding React components
- **External**: OpenRouter API access with support for Qwen embeddings and LLM inference
- **External**: Qdrant Cloud free tier account with sufficient capacity for textbook vectorization
- **External**: Neon Serverless Postgres account with connection pooling and sufficient throughput
- **Internal**: Decision on migrating existing OpenAI-based embeddings to Qwen (affects re-indexing requirements)

## Out of Scope

- **Authentication System**: User login, accounts, and persistent cross-device conversation history are deferred to future phases
- **Real-Time Collaboration**: Multiple students co-editing or sharing chat sessions
- **Voice/Audio Interface**: Speech-to-text queries and text-to-speech answers
- **Diagram/Image Understanding**: Questions about diagrams, images, or videos require manual extraction
- **Code Execution**: The chatbot explains code but does not execute, test, or debug code snippets interactively
- **Instructor Content Management**: Tools for instructors to manually curate, edit, or override chatbot responses
- **Advanced Analytics Dashboard**: Detailed instructor analytics (heatmaps, engagement funnels, A/B testing) beyond basic metrics
- **Multi-Textbook Support**: Extending to multiple books in a shared deployment
- **Offline Mode**: Requires internet connectivity
