---
id: 004
title: Propagate spec corrections
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: continuation
labels: ["spec-update", "consistency", "clarifications", "21-chapters", "12-sections"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/spec.md
tests:
  - N/A (specification consistency update)
---

## Prompt

Continue updating `specs/001-physical-ai-textbook/spec.md` to propagate all corrections from the Major Clarification session throughout the document.

(This prompt represents the continuation of work after conversation summarization. The user requested that I continue the last task without asking further questions.)

## Response snapshot

Completed comprehensive spec.md update to propagate all 9 critical corrections from "Session 2025-12-06 (Major Correction)" throughout the entire specification document.

**Sections Updated** (8 edits across 7 major sections):

1. **FR-001 (Content Structure)**: Updated from "4-5 chapters (total 20-25)" to **exact 21-chapter structure** with precise breakdown: Module 0 (3 chapters, Weeks 1-2), Module 1 (5 chapters, Weeks 3-5), Module 2 (4 chapters, Weeks 6-7), Module 3 (5 chapters, Weeks 8-10), Module 4 (3 chapters, Weeks 11-12), Capstone (1 chapter, Week 13).

2. **FR-002 (Chapter Template)**: Expanded from "11-section template" to **12-section template** with full enumeration: (1) Frontmatter, (2) Learning Objectives, (3) Prerequisites, (4) **Weekly Mapping** [NEW], (5) Motivating Scenario, (6) Core Theory & Mathematics, (7) Worked Example, (8) Hands-On Code, (9) Application to Humanoid Robots, (10) **Common Pitfalls & Debugging** [NEW], (11) Exercises, (12) Further Reading & References.

3. **FR-002a (NEW)**: Added mandatory "Weekly Breakdown" table requirement for every module overview page.

4. **FR-002b (NEW)**: Added mandatory "Building Your Physical AI Lab" top-level section with hardware tables (~$700 Jetson kit, RTX workstation, robot options), cloud costs, Economy Jetson Student Kit table, and latency trap warning.

5. **FR-026 (Documentation Management)**: Completely rewrote to reflect corrected MCP Context-7 workflow: "At the start of EVERY chapter generation, Claude Code MUST explicitly call MCP Context-7 tools to retrieve exact documentation... **Pre-fetching once at project start is INVALID**".

6. **Key Entities - Module**: Updated from "5 total, 4-5 chapters each" to "6 total including capstone" with exact chapter counts per module. Added "Weekly Breakdown table" requirement.

7. **Key Entities - Chapter**: Updated from 11 sections to **12 sections** with full reference to FR-002.

8. **Success Criteria SC-004**: Updated from "11-section template compliance" to "12-section template compliance".

9. **Success Criteria SC-011 (NEW)**: Added "Weekly Breakdown" table presence and accuracy validation.

10. **Success Criteria SC-012 (NEW)**: Added "Building Your Physical AI Lab" section existence with all required tables.

11. **Internal Dependencies - MCP Context-7**: Updated to "available to retrieve versioned documentation at the start of EVERY chapter generation... Context-7 is invoked per-chapter, not pre-fetched once at project start".

12. **Scope - In Scope**: Updated from "5 modules" to "**exactly 21 chapters** across 5 modules + capstone", added "Building Your Physical AI Lab" section, added "Weekly Breakdown" tables.

**Consistency Achieved**:
- Zero conflicts between Clarifications section and Functional Requirements
- All 9 major corrections from clarification session now reflected in requirements, entities, success criteria, dependencies, and scope
- Official book title reflected in header and content
- Exact chapter count (21) consistent across all sections
- 12-section template enforced in FR-002, Key Entities, and SC-004
- MCP Context-7 per-chapter invocation workflow documented in FR-026 and Internal Dependencies

**Impact**: The spec.md file is now internally consistent and fully aligned with the "Session 2025-12-06 (Major Correction)" which serves as the single source of truth. All downstream artifacts (plan.md, tasks.md) can now be generated or updated based on this corrected specification without encountering conflicts or ambiguities.

## Outcome

- ✅ Impact: Specification fully consistent with Major Correction clarifications. Zero conflicts between sections. Ready for `/sp.plan` update and `/sp.tasks` generation.
- 🧪 Tests: N/A (specification consistency update, but all success criteria SC-001 through SC-012 now properly defined)
- 📁 Files: 1 file modified (spec.md with 12 atomic edits propagating corrections)
- 🔁 Next prompts: Update `plan.md` to reflect 21 chapters (not 20-25), correct MCP Context-7 workflow, update data model for 12-section template; then run `/sp.tasks` to generate granular task breakdown
- 🧠 Reflection: Systematic propagation ensured no orphaned references to old values (20-25 chapters, 11 sections, pre-fetch workflow). Adding FR-002a and FR-002b as separate requirements improves traceability. SC-011 and SC-012 provide measurable validation criteria for new mandatory sections.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 9 major corrections successfully propagated to all relevant sections
- Graders run and results (PASS/FAIL): PASS (manual validation: zero conflicts between Clarifications and Requirements/Entities/Success Criteria/Dependencies/Scope sections)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
