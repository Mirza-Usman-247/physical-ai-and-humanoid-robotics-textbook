---
id: 002
title: Clarifications Resolved - Auth Translation
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4.5
feature: 003-auth-personalization-translation
branch: 003-auth-personalization-translation
user: developer
command: /sp.clarify
labels: ["clarifications", "questionnaire", "caching", "translation", "permissions", "architecture"]
links:
  spec: specs/003-auth-personalization-translation/spec.md
  clarifications: specs/003-auth-personalization-translation/clarifications.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-auth-personalization-translation/clarifications.md
  - specs/003-auth-personalization-translation/spec.md (updated)
  - specs/003-auth-personalization-translation/checklists/requirements.md (updated)
tests:
  - All 7 clarification questions answered
  - Functional requirements updated with explicit decisions
  - Assumptions expanded to cover edge cases
---

## Prompt

Clarifications Required:

1. Exact questions asked during signup for software and hardware background.
2. How personalization differs from RAG answers (static chapter rewrite vs dynamic overlay).
3. Whether personalized content is cached or regenerated every request.
4. Urdu translation quality expectations (technical terms: translated or preserved).
5. User control over reverting to original English content.
6. Permission boundaries: personalization and translation only for logged-in users.
7. How skills interact with RAG without duplicating logic.

Output:
Explicit assumptions and decisions to remove ambiguity.

## Response snapshot

Created comprehensive clarifications document addressing all 7 questions:

**1. Signup Questionnaire Design:**
- Software: Multi-select (Python, ML, ROS, Linux, C++, JS, None/Beginner, Other)
- Hardware: Single-select radio (GPU, Jetson, Robot, CloudOnly)
- Optional: Learning goal (free text)
- Storage: JSONB for skills, ENUM for hardware

**2. Personalization vs RAG Architecture:**
- Personalization = Static chapter rewrite (one-time transformation)
- RAG = Dynamic Q&A (real-time responses)
- No shared logic between systems
- Different triggers, scopes, outputs

**3. Caching Strategy:**
- Redis cache with 30-minute TTL
- Key format: `personalized:{userId}:{chapterId}:{profileHash}`
- Cache invalidation: Profile update, TTL expiration
- First request: ~10s, cached: <100ms

**4. Urdu Translation Rules:**
- 95% English technical term preservation target
- 100% code snippet preservation
- Hybrid format: "Forward kinematics (آگے کی حرکیات)..."
- Measurable quality criteria defined

**5. User Toggle Controls:**
- "Show Original Content" button for personalization
- "Show Original English" button for translation
- Session storage for preferences
- Translation never persists (refresh → English)

**6. Permission Model:**
- Buttons completely hidden for anonymous users (not disabled)
- Strict API authentication (401 errors)
- No confusing disabled UI elements

**7. Skills Separation:**
- Distinct AI skills: personalization-skill, translation-skill separate from RAG
- Shared LLM infrastructure only (OpenRouter/DeepSeek)
- No logic duplication, different prompts and contexts

Updated specification with 5 new functional requirements (FR-044 to FR-048) and 4 new assumptions (A-009 to A-012).

## Outcome

- ✅ Impact: All ambiguities removed, spec ready for implementation planning
- 🧪 Tests: 48 functional requirements now explicitly testable
- 📁 Files: clarifications.md (16 sections, 7 decisions), spec.md (updated requirements), checklist (updated)
- 🔁 Next prompts: /sp.plan (proceed to architecture design)
- 🧠 Reflection: Clarifications transformed vague requirements into explicit, measurable decisions. Caching strategy, translation rules, and permission boundaries now unambiguous.

## Evaluation notes (flywheel)

- Failure modes observed: None - all clarifications successfully integrated
- Graders run and results (PASS/FAIL):
  - Signup Questions: PASS (explicit UI and storage format)
  - Architecture: PASS (clear separation of personalization vs RAG)
  - Caching: PASS (Redis strategy with TTL and invalidation rules)
  - Translation: PASS (95% term preservation, hybrid format)
  - Permissions: PASS (hidden buttons, API enforcement)
- Prompt variant (if applicable): Standard /sp.clarify workflow
- Next experiment (smallest change to try): /sp.plan to design database schema and API contracts
