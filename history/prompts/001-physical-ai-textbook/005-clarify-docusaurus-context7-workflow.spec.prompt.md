---
id: 005
title: Clarify Docusaurus Context-7 workflow
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: 001-physical-ai-textbook
branch: 001-physical-ai-textbook
user: user
command: /sp.clarify
labels: ["clarification", "context-7", "docusaurus", "automation", "quality-gates", "code-validation"]
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

Please makesure we are using context7 mcp for latest documentation for docasaurus

## Response snapshot

Conducted focused clarification session addressing MCP Context-7 integration for Docusaurus documentation plus 4 related automation and validation concerns.

**Questions Asked & Answered** (5 of 5):

1. **Docusaurus Documentation via Context-7**: Should Claude Code retrieve Docusaurus 3.x docs via MCP Context-7?
   - **Answer**: Retrieve Docusaurus docs for initial setup + whenever modifying site code (components, config, plugins)
   - **Integration**: Created FR-026a requiring Context-7 retrieval during site setup and component modifications; updated Internal Dependencies section

2. **Docusaurus Version Pinning**: Should Docusaurus be version-pinned like other dependencies?
   - **Answer**: Pin to major version (Docusaurus 3.x latest stable) - allows minor/patch updates within v3
   - **Integration**: Updated FR-026a with version pinning rationale; updated External Dependencies to clarify "latest stable within v3" strategy

3. **Code Example Validation**: How should code examples be validated for runnability?
   - **Answer**: Automated CI testing - validation scripts execute examples in Docker containers with pinned dependencies
   - **Integration**: Created FR-009a requiring Docker-based CI validation; updated FR-015a to include code validation as quality gate

4. **Code Storage Strategy**: Should code be in `/examples/` directory or embedded in MDX?
   - **Answer**: Hybrid approach - simple inline examples embedded in MDX, complex multi-file examples in `/examples/[chapter]/`
   - **Integration**: Updated FR-009 to specify hybrid approach (≤20 lines embedded, complex in /examples/); updated FR-009a to clarify validation applies to /examples/ code

5. **Image Optimization**: Should image optimization be automated or manual?
   - **Answer**: Automated optimization in CI - GitHub Actions optimizes images, fails if >500KB after compression
   - **Integration**: Updated FR-019 to require automated GitHub Actions workflow for image compression; updated FR-015a to include image optimization as quality gate

**Sections Modified** (8 sections):
- New Clarifications subsection: "Session 2025-12-06 (Docusaurus Documentation via MCP Context-7)" (5 Q&A entries)
- FR-026a (NEW): Docusaurus documentation retrieval via Context-7
- FR-009: Hybrid code storage approach (embedded + /examples/)
- FR-009a (NEW): Automated code validation in Docker
- FR-019: Automated image optimization workflow
- FR-015a: Updated CI/CD quality gates (code validation, image optimization)
- External Dependencies: Docusaurus 3.x version pinning clarified
- Internal Dependencies: Docusaurus Context-7 requirement added

**Coverage Assessment**: All 10 taxonomy categories marked as Resolved or Clear. Zero outstanding critical ambiguities. Spec ready for planning phase.

**Impact**: User's primary concern (Context-7 for Docusaurus) comprehensively addressed with 5 complementary clarifications ensuring:
- Latest Docusaurus 3.x documentation used for all site code
- Version stability through major version pinning (3.x)
- Automated quality gates for code validation and image optimization
- Clear hybrid code storage strategy resolving earlier /examples/ contradiction
- Fully automated CI/CD pipeline with Docker-based validation

## Outcome

- ✅ Impact: Specification fully clarified with 5 critical automation and validation decisions captured and integrated. All ambiguities resolved, ready for `/sp.plan`
- 🧪 Tests: N/A (specification refinement, but automated testing strategies now fully defined in FR-009a, FR-019, FR-015a)
- 📁 Files: 1 file modified (spec.md with 8 sections updated/added)
- 🔁 Next prompts: `/sp.plan` to create implementation plan incorporating Docusaurus Context-7 workflow, Docker-based code validation, and automated image optimization
- 🧠 Reflection: User's focused request about Context-7 for Docusaurus revealed broader automation strategy gaps. Sequential questioning uncovered 4 related concerns (version pinning, code validation, storage strategy, image optimization) that form coherent quality assurance framework. Hybrid code approach (Q4) elegantly resolved earlier constitution deviation about /examples/ directory.

## Evaluation notes (flywheel)

- Failure modes observed: None - all 5 questions answered successfully, integrated atomically with zero conflicts
- Graders run and results (PASS/FAIL): PASS (coverage assessment: 10/10 categories resolved, zero outstanding ambiguities, all new requirements cross-referenced in CI/CD quality gates)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
