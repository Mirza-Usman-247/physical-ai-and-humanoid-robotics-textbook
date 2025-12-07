<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0 (MINOR: Constitution refinement with enhanced structure and clarity)
Modified principles:
- Purpose clarified with explicit project stack requirements
- Book Structure Requirements refined with explicit module definitions
- Workflow & Artifacts section expanded with clearer execution flow
- Quality Gates formalized with comprehensive pass/fail criteria
- ADR Policy enhanced with concrete examples
Added sections:
- Authoritative Principles (six core principles with detailed requirements)
- Documentation & Site Rules (Docusaurus-specific requirements)
- Governance section restructured with clearer amendment procedures
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ aligned (Constitution Check references)
- .specify/templates/spec-template.md: ✅ aligned (requirement structure matches)
- .specify/templates/tasks-template.md: ✅ aligned (phase-based structure supports book workflow)
- No command files found - N/A
Follow-up TODOs: None (all placeholders filled)
-->

# Project Constitution for "Humanoid Robotics & Physical AI"

**Project Name**: Humanoid Robotics & Physical AI (Textbook & Docusaurus Site)
**Version**: 1.1.0
**Ratified**: 2025-12-06
**Last Amended**: 2025-12-06

---

## I. PURPOSE

This constitution defines the strict rules, standards, workflows, and responsibilities for all agents, tools, and humans involved in producing the "Humanoid Robotics & Physical AI" textbook and Docusaurus site.

### Project Stack

The following technologies and tools are MANDATORY and define the foundation of this project:

- **Spec-Kit Plus**: Spec-driven authoring and development workflow
- **Claude Code**: Primary content generator for all textbook content
- **Docusaurus**: Frontend framework and publishing platform
- **GitHub Pages**: Deployment and hosting platform
- **Version-Pinned Code Examples**: Python and ROS 2 examples with explicit version constraints
- **Required Glossary + Notation System**: Standardized terminology and mathematical notation
- **ADRs**: Architecture Decision Records for major decisions

**Constitutional Override**: No part of the project may deviate from this constitution without formal amendment.

---

## II. AUTHORITATIVE PRINCIPLES

These principles are non-negotiable and MUST be enforced in all project activities.

### 1. 100% Spec-Driven Development

All content MUST follow the prescribed workflow: `/sp.specify` → `/sp.plan` → `/sp.tasks`.

**Requirements**:
- Claude Code outputs MUST match the approved specification exactly
- No content may be generated without a corresponding specification
- All deviations require explicit documentation and justification
- Specifications are the single source of truth for requirements

**Rationale**: Ensures consistency, traceability, and reproducibility across all content generation.

### 2. Technical Accuracy Only

All equations, control models, robotics mathematics, and code MUST be correct, verifiable, and sourced.

**Requirements**:
- Every technical claim MUST be supported by: (a) citation, (b) derivation/proof, or (c) experimental/empirical evidence
- No speculation or unstated assumptions permitted
- All algorithms and formulas must be mathematically sound and tested
- Code examples must be executable and validated

**Rationale**: Academic and professional credibility depends on absolute technical accuracy. Readers trust this textbook for correct information.

### 3. Deterministic, Reproducible Output

Agents MUST generate consistent results following the specification and constitution.

**Requirements**:
- Same specification input MUST produce equivalent output across executions
- Randomness or non-deterministic behavior must be explicitly documented and justified
- All tools and dependencies must be version-pinned
- Build processes must be repeatable and documented

**Rationale**: Enables collaboration, debugging, and maintenance. Users must be able to reproduce examples and results.

### 4. Single Source of Truth (SSOT)

All project artifacts have a designated canonical location.

**Required Structure**:
- Specifications: `specs/[chapter]/`
- Docusaurus documentation: `docs/[module]/`
- Code examples: `examples/[chapter]/`
- Glossary and notation: Global reference files (linked from all chapters)
- Architecture decisions: `history/adr/`
- Prompt history: `history/prompts/`

**Rationale**: Prevents duplication, inconsistency, and conflicting information. Makes maintenance scalable.

### 5. Runnable, Version-Pinned Code Only

Code MUST be executable, safe, and validated.

**Requirements**:
- All code examples must run successfully in the specified environment
- Dependencies must be version-pinned (e.g., `numpy==1.24.3`, `ros2==humble`)
- Safety notes REQUIRED for hardware-related code:
  - "Run in simulation first"
  - "E-Stop accessible"
  - "Torque limits enforced"
- All code must include validation tests or scripts
- Pseudocode must be explicitly marked as non-executable

**Rationale**: Readers must be able to run examples successfully. Safety is paramount in robotics.

### 6. Zero Hallucinations

Agents MUST use citations, derivations, or authoritative references for all claims.

**Requirements**:
- All factual claims must be verifiable
- Citations must be provided for external information
- Mathematical derivations must be shown step-by-step
- Assumptions must be explicitly stated
- Uncertain information must be clearly marked with "NEEDS VERIFICATION" or similar

**Rationale**: Academic integrity and reader trust depend on factual accuracy and intellectual honesty.

---

## III. BOOK STRUCTURE

### Mandatory Modules

The textbook MUST include the following **6 modules** with **exactly 21 chapters** in order:

- **Module 0: Physical AI Foundations** (3 chapters, Weeks 1-2)
- **Module 1: ROS 2 for Physical AI** (5 chapters, Weeks 3-5)
- **Module 2: Digital Twin** (4 chapters, Weeks 6-7)
- **Module 3: NVIDIA Isaac** (5 chapters, Weeks 8-10)
- **Module 4: VLA Humanoid Robotics** (3 chapters, Weeks 11-12)
- **Capstone: Autonomous Humanoid Project** (1 chapter, Week 13)

**Total**: 6 modules, 21 chapters, 13-week course structure

### Strict Chapter Template

Every chapter MUST include the following sections in order:

1. **Frontmatter** (Docusaurus metadata: title, description, keywords, sidebar_position)
2. **Learning Objectives** (3-7 measurable objectives)
3. **Prerequisites**
   - Internal chapters (links to previous chapters)
   - External topics if needed (e.g., linear algebra, Lagrangian mechanics, ROS 2 basics)
4. **Motivating Context**
   - Why this concept matters for humanoid robotics
   - Real-world problem or application
5. **Core Concepts**
   - Formal definitions
   - Equations with derivations
   - Diagrams (clearly labeled and referenced)
   - Algorithmic flow (pseudocode or flowcharts)
   - Assumptions explicitly stated
6. **Worked Example**
   - Complete solution with code, mathematics, and explanation
   - Step-by-step walkthrough
7. **Applications to Humanoid Robotics**
   - Real-world system examples (e.g., Atlas, Digit, HRP-5P, Optimus)
   - How the concept is used in practice
8. **Code Example**
   - Fully runnable code with version-pinned dependencies
   - Inline comments explaining reasoning ("why"), not syntax ("what")
   - Safety notes if hardware-related
9. **Summary**
   - Key concepts concisely restated (bullet list)
10. **Exercises** (minimum 3)
    - Conceptual questions
    - Computational problems
    - Implementation-focused tasks
11. **References**
    - Academic citation format (APA, IEEE, or similar)
    - Peer-reviewed sources preferred

**Deviation Policy**: Any deviation from this template requires explicit justification and approval.

---

## IV. DOCUMENTATION & SITE RULES

### Docusaurus Page Requirements

All Docusaurus pages MUST include:

1. **Frontmatter metadata**:
   - `title`: Page title (H1 equivalent)
   - `description`: Brief summary for SEO
   - `keywords`: Comma-separated list for search
   - `sidebar_position`: Numeric ordering in navigation

2. **Link integrity**: No broken links (internal or external)

3. **Media optimization**:
   - No missing images
   - Images ≤ 500KB each
   - Images stored in `/static/img/[chapter]/`
   - Meaningful alt-text for all diagrams

4. **Performance**: Pages MUST render under 3 seconds on mobile

5. **Build quality**: No warnings or errors during Docusaurus build

### SEO Requirements

The site MUST include:

- **OpenGraph** metadata for social sharing
- **Sitemap** (`sitemap.xml`) for search engines
- **robots.txt** for crawler directives

---

## V. WORKFLOW & ARTIFACTS

### Mandatory Workflow

All content creation MUST follow this sequence:

1. **`/sp.specify`**: Define chapter scope, learning objectives, and requirements
2. **`/sp.plan`**: List diagrams, equations, code examples, and references needed
3. **`/sp.tasks`**: Break into granular tasks for Claude Code execution
4. **Claude Code Generation**: Generate all content per specification
5. **Review**:
   - Technical accuracy validation
   - Clarity and pedagogical quality review
   - Constitution compliance check
6. **Docusaurus Build**: Verify site builds without errors
7. **GitHub Pages Deploy**: Publish to production

### Required Artifacts per Chapter

Each chapter MUST produce:

- `specs/[chapter]/spec.md`: Feature specification
- `specs/[chapter]/plan.md`: Implementation plan
- `specs/[chapter]/tasks.md`: Task breakdown
- `history/prompts/[chapter]/`: Prompt History Records (PHRs)
- `history/adr/`: ADRs for architecturally significant decisions

### Code Example Requirements

Code examples MUST:

1. Be runnable without modification
2. Include version-pinned dependencies in a requirements file or inline
3. Include comments explaining reasoning ("why"), not syntax ("what")
4. Be placed in `examples/[chapter]/`
5. Include a validation test or script
6. Follow safety requirements:
   - "Ensure E-Stop accessible"
   - "Run in simulation first before hardware"
   - "Limit torques to safe range"

---

## VI. ADR POLICY

### When to Create an ADR

Create an Architecture Decision Record when a decision:

1. **Impacts multiple modules**: Affects more than one module or more than two chapters
2. **Has alternatives**: Multiple viable options were considered
3. **Has long-term architectural consequences**: Will be difficult or costly to change later

### ADR Trigger Test (Three-Part)

All three must be true:

- **Impact**: Does this have long-term structural consequences?
- **Alternatives**: Were multiple viable options considered?
- **Scope**: Is this cross-cutting or does it influence system design?

If **ALL** are true, suggest creating an ADR.

### Examples of ADR-Worthy Decisions

- **Simulation engine choice**: Mujoco vs Isaac Sim vs Gazebo
- **Middleware selection**: ROS 2 vs custom middleware
- **Notation system**: Mathematical symbol conventions
- **Module ordering**: Pedagogical sequencing and dependencies

### ADR Suggestion Format

When an architecturally significant decision is detected, agents MUST suggest:

```
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

**Policy**: Wait for user consent; never auto-create ADRs.

---

## VII. QUALITY GATES (MUST PASS)

All of the following MUST pass before content can be merged or deployed:

1. **Docusaurus Build**: `npm run build` = PASS (no errors)
2. **Broken Links**: Link checker = PASS (no broken links)
3. **Spell Check**: Spell check = PASS (no typos)
4. **Accessibility**: WCAG AA compliance = PASS
5. **Technical Accuracy**: Technical review = PASS (equations, code, facts verified)
6. **Code Tests**: All code examples execute successfully = PASS
7. **Performance**: Lighthouse score ≥ 90
8. **Completeness**: No TODO, TBD, or placeholder content remaining

**Enforcement**: PRs cannot be merged unless all quality gates pass.

---

## VIII. GOVERNANCE

### Amendment Procedure

Changes to this constitution require:

1. **Documented rationale**: Why is the change necessary?
2. **Team discussion**: Stakeholder review and consensus
3. **Migration steps**: How will existing content be updated?
4. **Template updates**: All dependent templates must be synchronized

### Versioning Policy

Constitution versions follow semantic versioning:

- **MAJOR**: Backward-incompatible governance or principle removals/redefinitions
- **MINOR**: New principles/sections added or materially expanded guidance
- **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements

### Compliance and Audits

- **Quarterly audits required** to detect drift from constitutional principles
- **Constitution supersedes all other documents**: In case of conflict, this constitution prevails
- **Checklist for reviewers MUST include constitution compliance**

### Enforcement

This constitution is the highest authority in the project. All team members, agents, and tools MUST comply. Non-compliance must be documented and justified, or the work must be revised to comply.

---

## IX. SUMMARY ENFORCEMENT CHECKLIST

The book MUST be:

1. ✅ Created using Spec-Kit Plus workflow
2. ✅ Authored using Claude Code as primary content generator
3. ✅ Published using Docusaurus
4. ✅ Contain 5 modules (Module 0 + Modules 1-4)
5. ✅ Built and deployed to GitHub Pages
6. ✅ Spec-driven (all content follows `/sp.specify` → `/sp.plan` → `/sp.tasks`)
7. ✅ Technically accurate (all equations, code, and facts verified)
8. ✅ Consistent (terminology, notation, chapter template followed)
9. ✅ Pass all quality gates before deployment

---

**END OF CONSTITUTION**
