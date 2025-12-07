---
id: 001
title: Create textbook spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: /sp.specify
labels: ["spec", "textbook", "physical-ai", "robotics", "educational-content"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/spec.md
  - specs/001-physical-ai-textbook/checklists/requirements.md
tests:
  - N/A (specification document)
---

## Prompt

Project: "Physical AI & Humanoid Robotics"
Goal: Produce a spec-driven textbook + Docusaurus site using Spec-Kit Plus → Claude Code → GitHub Pages. All tools MUST use latest documentation via MCP context-7. All output MUST follow the Project Constitution.

I. SCOPE
--------
Create a rigorous, accurate book on Physical AI & Humanoid Robotics covering:
- Embodied intelligence, sensors, physics-aware AI
- ROS 2 (control middleware)
- Gazebo & Unity (Digital Twin simulation)
- NVIDIA Isaac (perception, VSLAM, RL, sim-to-real)
- Vision-Language-Action (VLA) + Whisper speech → ROS 2 actions
- Humanoid kinematics, locomotion, manipulation
- Capstone: Autonomous Humanoid (voice → plan → navigation → perception → manipulation)

II. REQUIRED MODULES
--------------------
0. Intro to Physical AI
1. ROS 2: The Robotic Nervous System
2. Digital Twin: Gazebo + Unity
3. NVIDIA Isaac: The AI-Robot Brain
4. Vision-Language-Action Robotics
Capstone: Autonomous Humanoid Project

III. CHAPTER REQUIREMENTS
-------------------------
All chapters MUST include:
- Learning objectives, prerequisites
- Core concepts (definitions + equations)
- Worked example
- Humanoid applications
- Runnable, version-pinned code (Python/ROS 2)
- Summary, exercises, references

IV. CODE & SITE RULES
---------------------
- Runnable, validated, safe (sim-first, E-Stop, torque limits)
- Stored in `/examples/[chapter]/`
- Docusaurus pages MUST have metadata + clean build
- No broken links; SEO enabled; deploy to GitHub Pages

V. HARDWARE & SIMULATION
------------------------
Book includes guidance for:
- RTX workstation (Isaac/Gazebo)
- Jetson Orin Nano/NX (edge brain)
- RealSense D435i
- Robots: Unitree Go2 (proxy) or G1/OP3 (humanoid)
Cloud option: AWS g5/g6 instances for Isaac Sim.

VI. WORKFLOW
------------
- `/sp.specify` → `/sp.plan` → `/sp.tasks` → Claude Code
- Deterministic, spec-compliant generation
- ADRs for major decisions (simulation engine, ROS version, notation)

VII. DELIVERABLES
-----------------
- `spec.md`, `plan.md`, `tasks.md` per chapter
- `/docs/[module]/` pages
- `/examples/[chapter]/` runnable code
- ADR log
- Production-ready Docusaurus site

End of /sp.specs

## Response snapshot

Created comprehensive feature specification for "Physical AI & Humanoid Robotics Textbook" with following structure:

**Branch**: 001-physical-ai-textbook (newly created)

**User Stories** (4 prioritized scenarios):
1. Self-Paced Learning Journey (P1) - Core educational value
2. Instructor Course Material (P2) - Classroom adoption
3. Practitioner Reference Guide (P3) - Professional reference
4. Capstone Project Implementation (P2) - End-to-end integration

**Functional Requirements** (24 requirements across 5 categories):
- Content Structure (FR-001 to FR-004): 5 modules, 11-section chapter template, equation derivations, technical accuracy
- Code Examples (FR-005 to FR-009): Runnable code, version-pinned dependencies, safety warnings
- Docusaurus Site (FR-010 to FR-014): Metadata, build success, performance, SEO, link integrity
- Educational Quality (FR-015 to FR-018): Learning objectives, prerequisites, exercise types, accessibility
- Workflow Compliance (FR-022 to FR-024): Spec-driven workflow, ADRs, required artifacts

**Success Criteria** (10 measurable outcomes):
- SC-001: Module completion time (10 hours for Modules 0-1)
- SC-002: Code success rate (90% execute on first attempt)
- SC-003: Site performance (Lighthouse ≥90, <2min build)
- SC-004: Template compliance (100% of chapters)
- SC-005: Link integrity (zero broken links)
- SC-006: Technical accuracy (100% backed by citations)
- SC-007: Capstone completion (20-40 hours)
- SC-008: Expert validation (3+ reviewers)
- SC-009: Exercise quality (clear criteria, aligned objectives)
- SC-010: Accessibility (WCAG AA compliant)

**Key Entities**: Module, Chapter, Code Example, Exercise, Reference, Glossary Entry, Notation, ADR, Hardware Specification

**Dependencies**: ROS 2 Humble, NVIDIA Isaac Sim 4.x, Gazebo, Unity 2022.x LTS, Whisper, Docusaurus 3.x, Python 3.10+, Constitution v1.1.0, Spec-Kit Plus tools

**Scope**: Clearly defined In Scope (textbook content, code, site, capstone, hardware recommendations) vs Out of Scope (hardware provision, video content, multi-language, certifications)

**Risks**: External tool version changes, hardware unavailability, content volume underestimation, technical accuracy challenges (all with mitigations)

**Validation**: Spec quality checklist created and validated - all 12 items passed, zero [NEEDS CLARIFICATION] markers, ready for `/sp.plan`

## Outcome

- ✅ Impact: Complete feature specification for ambitious Physical AI & Humanoid Robotics textbook project, following Project Constitution v1.1.0 and Spec-Kit Plus workflow
- 🧪 Tests: N/A (specification document, but validation checklist confirms spec quality)
- 📁 Files: 2 files created (spec.md, checklists/requirements.md)
- 🔁 Next prompts: `/sp.plan` to create implementation plan, identify ADRs for simulation engine, ROS version, notation system, module ordering
- 🧠 Reflection: Specification is comprehensive, testable, and constitution-compliant. All requirements are measurable and unambiguous. Ready for planning phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - spec completed without clarification needs due to detailed user input
- Graders run and results (PASS/FAIL): PASS (spec quality checklist: 12/12 items passed)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
