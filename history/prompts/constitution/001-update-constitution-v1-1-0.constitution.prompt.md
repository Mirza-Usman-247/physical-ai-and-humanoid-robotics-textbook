---
id: 001
title: Update constitution v1.1.0
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "governance", "textbook-project"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

Project Constitution for "Humanoid Robotics & Physical AI"
===========================================================

I. PURPOSE
----------
This constitution defines the strict rules, standards, workflows, and responsibilities
for all agents, tools, and humans involved in producing the "Humanoid Robotics & Physical AI"
textbook and Docusaurus site.

The project stack:

- Spec-Kit Plus (spec-driven authoring)
- Claude Code (primary content generator)
- Docusaurus (frontend + publishing)
- GitHub Pages (deployment)
- Version-Pinned Code Examples (Python, ROS 2)
- Required Glossary + Notation system
- ADRs for major decisions

No part of the project may deviate from this constitution.

II. AUTHORITATIVE PRINCIPLES
----------------------------

1. **100% Spec-Driven Development**
   - All content MUST follow `/sp.specify`, `/sp.plan`, and `/sp.tasks`.
   - Claude Code outputs MUST match the approved spec exactly.

2. **Technical Accuracy Only**
   - All equations, control models, robotics math, and code MUST be correct, verifiable,
     and sourced.
   - No speculation or unstated assumptions.

3. **Deterministic, Reproducible Output**
   - Agents MUST generate consistent results following the spec and constitution.

4. **Single Source of Truth (SSOT)**
   - Specs → `specs/[chapter]/`
   - Docusaurus docs → `docs/[module]/`
   - Code examples → `/examples/[chapter]/`
   - Glossary + notation → global references

5. **Runnable, Version-Pinned Code Only**
   - Code MUST be executable, safe, and validated.
   - Required safety notes:
     *"Run in simulation first"*, *"E-Stop accessible"*, *"Torque limits enforced"*.

6. **Zero Hallucinations**
   - Agents MUST use citations, derivations, or authoritative references.
   - All claims MUST be provably true.

III. BOOK STRUCTURE
-------------------

Mandatory modules:

- Module 0: Introduction
- Module 1: Foundations
- Module 2: Control & Sensorimotor Systems
- Module 3: Physical Intelligence & RL
- Module 4: Applications & Future

All chapters MUST follow the strict template:
- Learning objectives
- Prerequisites
- Motivation
- Core concepts
- Worked example
- Humanoid applications
- Runnable code
- Summary
- Exercises
- References

IV. DOCUMENTATION & SITE RULES
-------------------------------

1. Docusaurus pages MUST include:
   - title
   - description
   - keywords
   - sidebar_position

2. No broken links, no missing images, no warnings.

3. Pages MUST render under 3 seconds on mobile.

4. SEO required:
   - OpenGraph
   - sitemap
   - robots.txt

V. WORKFLOW & ARTIFACTS
------------------------

1. `/sp.specify` → define chapter scope + objectives
2. `/sp.plan` → list diagrams, equations, examples, references
3. `/sp.tasks` → break into Claude Code tasks
4. Claude Code generates all content
5. Review: technical, clarity, constitution compliance
6. Docusaurus build → GitHub Pages deploy

Required artifacts per chapter:
- `spec.md`
- `plan.md`
- `tasks.md`
- `history/prompts/` logs
- ADRs for major decisions

VI. ADR POLICY
--------------

Create an ADR when a decision:
- Impacts multiple modules
- Has alternatives
- Has long-term architectural consequences

Examples:
- Simulation engine choice (Mujoco vs Isaac vs Gazebo)
- ROS 2 vs custom middleware
- Notation system
- Module ordering

VII. QUALITY GATES (MUST PASS)
------------------------------

- Docusaurus build = PASS
- Broken links = PASS
- Spell check = PASS
- Accessibility (WCAG AA) = PASS
- Technical accuracy = PASS
- Code tests = PASS
- Lighthouse ≥ 90
- No TODO / TBD

VIII. GOVERNANCE
----------------

- This constitution overrides all other documents.
- Changes require rationale, discussion, migration steps, and updated templates.
- Quarterly audits required.

## Response snapshot

Constitution updated successfully to version 1.1.0 (MINOR version bump).

**Changes made**:
- Enhanced structure and clarity across all sections
- Formalized six core authoritative principles with explicit requirements and rationales
- Defined mandatory 5-module book structure (Module 0 + Modules 1-4)
- Expanded chapter template with 11 required sections
- Added comprehensive Docusaurus page and SEO requirements
- Formalized 7-step mandatory workflow
- Enhanced ADR policy with three-part trigger test and concrete examples
- Established 8 quality gates that must pass before deployment
- Structured governance with amendment procedure, versioning policy, and compliance audits

**Templates validated**:
- `.specify/templates/spec-template.md`: ✅ Aligned
- `.specify/templates/plan-template.md`: ✅ Aligned (Constitution Check section matches)
- `.specify/templates/tasks-template.md`: ✅ Aligned (phase structure supports book workflow)

**Files modified**:
- `.specify/memory/constitution.md`: Completely restructured with all placeholders filled

**Version change**: 1.0.0 → 1.1.0
- Rationale: MINOR bump (new principles and sections added but backward compatible)

## Outcome

- ✅ Impact: Constitution formalized with comprehensive governance for "Humanoid Robotics & Physical AI" textbook project
- 🧪 Tests: N/A (constitution document)
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Ready to proceed with `/sp.specify` for first chapter/module
- 🧠 Reflection: Constitution now provides clear, enforceable standards for all agents and humans. All templates are aligned and ready for use.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS (all placeholders filled, structure complete, templates validated)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
