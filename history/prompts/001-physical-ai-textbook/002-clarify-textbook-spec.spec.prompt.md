---
id: 002
title: Clarify textbook spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: /sp.clarify
labels: ["clarification", "spec-refinement", "textbook", "workflow", "deployment"]
links:
  spec: specs/001-physical-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-textbook/spec.md
tests:
  - N/A (specification refinement)
---

## Prompt

Please analyze my project spec and ask all missing questions needed to fully clarify it.

My project:
"AI/Spec-Driven Book Creation using Docusaurus (latest version) with Context 7 MCP for documentation retrieval. The book will be generated, structured, and maintained using Spec-Kit Plus and Gemini CLI, and deployed to GitHub Pages."

Clarify:
1. Docusaurus requirements (version, theme, plugins, structure, deployment).
2. MCP Context-7 usage (how documentation is fetched, integrated, and referenced).
3. Book content workflow (chapters, formats, automation process).
4. Spec-Kit Plus configuration needs.
5. Gemini CLI usage (models, commands, file generation).
6. GitHub Pages deployment details.
7. Any ambiguous, conflicting, or incomplete sections.

Ask every question needed to ensure the spec becomes 100% clear and complete before writing ADRs or starting work.

## Response snapshot

Conducted systematic clarification session with 5 targeted questions addressing critical ambiguities in the Physical AI & Humanoid Robotics Textbook spec.

**Questions Asked & Answered** (5 of 5):

1. **Content Generation Workflow**: Who/what generates actual chapter content?
   - **Answer**: Claude Code generates content via Spec-Kit Plus workflow
   - **Integration**: Updated FR-022 to explicitly state Claude Code generates all chapter content (markdown, equations, code examples, exercises)

2. **MCP Context-7 Documentation Strategy**: How does MCP retrieve and integrate external documentation?
   - **Answer**: Pre-fetch all docs once at project start
   - **Integration**: Added FR-026 (Documentation Management) requiring MCP Context-7 to pre-fetch versioned documentation snapshots at project initialization; updated Internal Dependencies section

3. **Docusaurus Configuration**: Which theme and preset for the textbook site?
   - **Answer**: Classic theme with docs preset
   - **Integration**: Updated FR-010 to specify Docusaurus 3.x with classic theme and docs preset, configured for technical documentation with sidebar navigation and search

4. **GitHub Pages Deployment**: How should the site be deployed?
   - **Answer**: GitHub Actions CI/CD on push
   - **Integration**: Added FR-015a (Deployment and CI/CD) requiring automated GitHub Actions pipeline triggered on every push to main branch, running all quality gates before deployment

5. **Chapter Granularity**: How many chapters per module?
   - **Answer**: 4-5 chapters per module
   - **Integration**: Updated FR-001 to specify 4-5 chapters per module (20-25 total chapters); updated Module entity definition

**Sections Modified** (8 sections):
- New Clarifications section with Session 2025-12-06 (5 Q&A entries)
- FR-001: Content Structure (chapter count added)
- FR-010: Docusaurus Site (theme/preset specified)
- FR-015a: Deployment and CI/CD (new requirement)
- FR-022: Workflow Compliance (content generation clarified)
- FR-026: Documentation Management (new requirement)
- Key Entities - Module (chapter count updated)
- Internal Dependencies (MCP Context-7 pre-fetching added)

**Coverage Assessment**: All 10 taxonomy categories now marked as Resolved or Clear. No outstanding critical ambiguities. Spec ready for planning phase.

**Impact**: Eliminated all major decision points that would have caused rework during planning or implementation. Clarifications ensure:
- Clear content generation pipeline (Claude Code via Spec-Kit Plus)
- Stable documentation references (pre-fetched, versioned)
- Standard, proven technology choices (Docusaurus classic theme, GitHub Actions)
- Defined content volume (20-25 chapters total)

## Outcome

- ✅ Impact: Specification fully clarified with 5 critical decisions captured and integrated. All ambiguities resolved, ready for `/sp.plan`
- 🧪 Tests: N/A (specification refinement, but coverage assessment confirms 10/10 categories resolved)
- 📁 Files: 1 file modified (spec.md with 8 sections updated/added)
- 🔁 Next prompts: `/sp.plan` to create implementation plan with ADRs for ROS version, Isaac Sim choice, notation system, module ordering
- 🧠 Reflection: Systematic questioning approach successfully identified and resolved all user-requested clarifications plus discovered chapter granularity gap. Spec now provides unambiguous foundation for planning.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 5 questions answered successfully, integrated atomically
- Graders run and results (PASS/FAIL): PASS (coverage assessment: 10/10 categories resolved, zero outstanding ambiguities)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
