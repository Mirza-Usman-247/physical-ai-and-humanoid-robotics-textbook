---
id: 002
title: RAG Chatbot Clarifications
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: rag-chatbot-integration
branch: 002-rag-chatbot-integration
user: assistant
command: /sp.clarify
labels: ["rag", "chatbot", "clarifications", "architecture"]
links:
  spec: specs/002-rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-chatbot-integration/spec.md
tests:
  - N/A (clarification stage)
---

## Prompt

```
/sp.clarify Clarifications Required for RAG Chatbot Integration:

1. How to handle user-selected text dynamically in Docusaurus frontend for RAG queries.
2. Maximum token limits per request to OpenRouter API.
3. Frequency and batch size for Qwen embeddings indexing.
4. Role boundaries between rag-manager and sub-skills (retriever/indexer/answerer).
5. Error handling: what to return if retrieval fails or embeddings are unavailable.
6. Integration constraints between Claude Code Subagents and FastAPI endpoints.
7. Versioning strategy for book content updates and vector DB sync.

Output:
Clear assumptions, constraints, and decisions that remove ambiguity for authors and engineers.
```

## Response snapshot

Conducted interactive clarification session addressing 5 critical ambiguities in the RAG Chatbot specification:

**Question 1: Text Selection Mechanism**
- Decision: Use Docusaurus plugin hooks to intercept text selection and inject chatbot trigger button
- Impact: Integrated tightly with Docusaurus ecosystem; updated FR-014 to specify implementation approach

**Question 2: OpenRouter API Token Limits**
- Decision: 8,000 input tokens + 2,000 output tokens per request
- Impact: Added FR-013 for token limit enforcement and context truncation strategy; ensures cost control and prevents API failures

**Question 3: Error Handling Strategy**
- Decision: Return friendly message explaining issue + suggest alternative actions
- Impact: Added new "Error Handling and Reliability" section with FR-031 through FR-033 covering user-friendly errors, logging, and retry logic

**Question 4: RAG Skill Role Boundaries**
- Decision: rag-manager orchestrates pipeline but delegates all domain logic to sub-skills; only handles sequencing, validation, error aggregation
- Impact: Updated FR-034 through FR-037 to clarify architectural separation; ensures modularity and reusability

**Question 5: Content Versioning Strategy**
- Decision: Git-based versioning with commit hash tracking; automatic re-indexing via CI/CD hooks on new commits
- Impact: Updated FR-029, TextbookChunk entity (added git_commit_hash attribute), and edge case handling for stale content

**Updated Sections:**
- Clarifications (new section with 5 Q&A pairs)
- Functional Requirements: Added 3 new FRs (FR-013, FR-031-033), updated 6 existing FRs (FR-014-017 renumbered, FR-029 enhanced, FR-034-037 clarified)
- Key Entities: Enhanced TextbookChunk with git commit hash
- Edge Cases: Updated 2 edge cases (error handling, stale content)

**Deferred Topics** (lower priority for planning phase):
- Frequency and batch size for Qwen embeddings indexing (performance tuning detail)
- Integration constraints between Claude Code Subagents and FastAPI endpoints (implementation detail)

## Outcome

- ✅ Impact: Resolved 5 critical architectural and operational ambiguities; specification now ready for planning with clear technical constraints and design decisions
- 🧪 Tests: All clarifications integrated incrementally with validation after each update
- 📁 Files: Updated spec.md with 5 clarifications, 8 requirement changes, 1 entity enhancement, 2 edge case updates
- 🔁 Next prompts: `/sp.plan` to create detailed architectural design and implementation tasks
- 🧠 Reflection: Interactive clarification process successfully balanced user control (5 questions with recommended options) with efficiency (avoided over-clarification on low-impact details). Incremental spec updates after each answer prevented context loss and ensured consistency.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 5 questions answered clearly, no disambiguation needed
- Graders run and results (PASS/FAIL): PASS - All clarifications properly integrated with no contradictions or numbering conflicts after renumbering
- Prompt variant (if applicable): Standard /sp.clarify with user-provided clarification list; prioritized by architectural impact
- Next experiment (smallest change to try): Consider whether to ask about batch sizes and indexing frequency during planning phase when performance requirements become more concrete, or defer to implementation tuning
