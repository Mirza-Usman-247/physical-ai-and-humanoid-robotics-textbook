# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Project: Physical AI & Humanoid Robotics - Goal: Produce a spec-driven textbook + Docusaurus site using Spec-Kit Plus → Claude Code → GitHub Pages. All tools MUST use latest documentation via MCP context-7. All output MUST follow the Project Constitution."

## Clarifications

### Session 2025-12-06 (Initial)

- Q: Who/what generates the actual chapter content (markdown, equations, code examples)? → A: Claude Code generates content via Spec-Kit Plus workflow
- Q: How does MCP Context-7 retrieve and integrate external documentation (ROS 2, Isaac, Unity, etc.)? → A: **SUPERSEDED** - See Session 2025-12-06 (Major Correction) below
- Q: Which Docusaurus theme and preset should be used for the textbook site? → A: Classic theme with docs preset
- Q: How should the Docusaurus site be deployed to GitHub Pages? → A: GitHub Actions CI/CD on push
- Q: How many chapters should each of the 5 modules contain? → A: **SUPERSEDED** - See Session 2025-12-06 (Major Correction) below

### Session 2025-12-06 (Major Correction) - **SINGLE SOURCE OF TRUTH**

**CRITICAL**: This session supersedes any conflicting information from earlier clarifications or spec sections.

1. **Official Book Title**: "Physical AI & Humanoid Robotics – From Simulated Brains to Embodied Intelligence" (full university-level textbook + hands-on lab manual)

2. **Exact Chapter Count**: **21 chapters total** (not 20-25)
   - Module 0: 3 chapters (Weeks 1-2)
   - Module 1 (ROS 2): 5 chapters (Weeks 3-5)
   - Module 2 (Digital Twin): 4 chapters (Weeks 6-7)
   - Module 3 (NVIDIA Isaac): 5 chapters (Weeks 8-10)
   - Module 4 (VLA Robotics): 3 chapters (Weeks 11-12)
   - Capstone: 1 full chapter (Week 13)

3. **Week-by-Week Mapping** (13-week course):
   | Week    | Module & Content |
   |---------|------------------|
   | 1-2     | Module 0 – Introduction to Physical AI (3 chapters) |
   | 3-5     | Module 1 – ROS 2: The Robotic Nervous System (5 chapters) |
   | 6-7     | Module 2 – The Digital Twin: Gazebo & Unity (4 chapters) |
   | 8-10    | Module 3 – NVIDIA Isaac: The AI-Robot Brain (5 chapters) |
   | 11-12   | Module 4 – Vision-Language-Action (VLA) Robotics (3 chapters) |
   | 13      | Capstone – Autonomous Humanoid Project (1 full guide chapter) |

4. **Weekly Breakdown Table Requirement**: Every module overview page MUST contain a visible "Weekly Breakdown" table matching the above structure

5. **Hardware & Lab Section (Mandatory)**: Top-level section titled "Building Your Physical AI Lab – Hardware, Cloud, and Budget Options" MUST include:
   - Full hardware tables (RTX workstation, Jetson kit ~$700, robot options)
   - Cloud alternative cost calculations
   - "Economy Jetson Student Kit" table
   - Warning about latency trap when controlling real robots from cloud

6. **Chapter Template (12 Sections - Corrected)**:
   1. Frontmatter (title, description, sidebar_position, keywords)
   2. Learning Objectives (3-7, measurable, active voice)
   3. Prerequisites (internal + external links)
   4. Weekly Mapping (e.g., "Week 4 – Tuesday & Thursday")
   5. Motivating Scenario (real-world humanoid task)
   6. Core Theory & Mathematics (with full derivations)
   7. Worked Example (step-by-step)
   8. Hands-On Code (runnable, version-pinned, safety warnings)
   9. Application to Humanoid Robots
   10. Common Pitfalls & Debugging
   11. Exercises (1× Conceptual, 1× Computational, 2-3× Implementation)
   12. Further Reading & References

7. **Capstone Status**: Full standalone chapter (not appendix) with weekly schedule, architecture diagram, integration steps, demo video placeholders

8. **MCP Context-7 Workflow (Corrected)**: At the start of EVERY chapter generation, Claude Code MUST explicitly call Context-7 tools for exact versions (ROS 2 Humble, Isaac Sim 2024.x, etc.) to guarantee up-to-date syntax and APIs. **Pre-fetching once at project start is INVALID**.

9. **New Success Criteria**:
   - SC-011: The "Weekly Breakdown" table is present and 100% accurate on every module intro page
   - SC-012: The hardware/lab chapter exists and contains all budget tables and cloud cost calculations

### Session 2025-12-06 (Docusaurus Documentation via MCP Context-7)

- Q: Should Claude Code also retrieve latest Docusaurus 3.x documentation via MCP Context-7 during site setup and component development? → A: Retrieve Docusaurus docs for initial setup + whenever modifying site code (components, config, plugins)
- Q: Should Docusaurus be version-pinned to a specific minor version like other dependencies? → A: Pin to major version (Docusaurus 3.x latest stable) - allows minor/patch updates within v3
- Q: How should code examples be validated to ensure they are runnable? → A: Automated CI testing - validation scripts execute examples in Docker containers with pinned dependencies
- Q: Should code be stored in `/examples/` directory or embedded in chapter MDX files? → A: Hybrid approach - simple inline examples embedded in MDX chapters, complex multi-file examples in `/examples/[chapter]/` for CI validation
- Q: Should image optimization be automated or manual, and what happens if images exceed 500KB? → A: Automated optimization in CI - GitHub Actions optimizes images, fails if >500KB after compression

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Self-Paced Learning Journey (Priority: P1)

A robotics engineer, graduate student, or advanced undergraduate wants to learn Physical AI and humanoid robotics through structured, progressive content with hands-on code examples.

**Why this priority**: This is the core value proposition of the textbook - enabling individual learners to build knowledge from foundations to advanced applications.

**Independent Test**: Reader can start at Module 0 (Introduction), progress through each module sequentially, run all code examples successfully in their local/cloud environment, and complete exercises to validate understanding. Delivers immediate educational value.

**Acceptance Scenarios**:

1. **Given** a reader with basic programming knowledge and no prior robotics experience, **When** they read Module 0 and Module 1, **Then** they understand fundamental concepts of Physical AI, embodied intelligence, and can explain why robotics requires physics-aware computation.

2. **Given** a reader has completed Module 1, **When** they work through Module 2 (ROS 2), **Then** they can write, compile, and run ROS 2 nodes for basic robot control, understand the pub/sub architecture, and create a simple teleoperation system.

3. **Given** a reader is working through any chapter, **When** they encounter a code example, **Then** the code runs successfully in the specified environment (with version-pinned dependencies), produces expected output, and includes clear safety warnings for hardware operations.

4. **Given** a reader completes a chapter, **When** they attempt the exercises, **Then** exercises test conceptual understanding, computational skills, and implementation ability at appropriate difficulty levels.

---

### User Story 2 - Instructor Course Material (Priority: P2)

A university professor or industry trainer wants to use the textbook as the foundation for a Physical AI / Humanoid Robotics course.

**Why this priority**: Extends the reach and impact of the textbook by enabling classroom adoption. Educational institutions are key stakeholders for technical textbooks.

**Independent Test**: Instructor can select modules/chapters aligned with course objectives, assign reading and exercises to students, demonstrate code examples in lectures, and assess student learning using provided exercises and capstone project. Can be validated by piloting in a single course.

**Acceptance Scenarios**:

1. **Given** an instructor planning a semester-long course, **When** they review the textbook structure, **Then** they can map modules to weekly topics, identify prerequisite knowledge for each chapter, and plan lab sessions using provided code examples.

2. **Given** an instructor is preparing lecture materials, **When** they access chapter content, **Then** each chapter includes clear learning objectives, worked examples suitable for live demonstration, and references to authoritative sources for deeper exploration.

3. **Given** students are assigned exercises from a chapter, **When** the instructor reviews exercise solutions, **Then** exercises have clear assessment criteria, range from basic to challenging, and align with stated learning objectives.

---

### User Story 3 - Practitioner Reference Guide (Priority: P3)

A professional robotics engineer working on a humanoid robotics project needs quick reference material for specific topics (e.g., inverse kinematics, VSLAM implementation, sim-to-real transfer).

**Why this priority**: Adds long-term value beyond initial learning. Professionals need reliable, accurate reference material when solving real-world problems.

**Independent Test**: Practitioner can navigate directly to relevant chapter via table of contents or search, quickly locate needed information (equations, algorithms, code patterns), and apply it to their specific problem. Can be tested by having practitioners solve realistic scenarios using only the textbook.

**Acceptance Scenarios**:

1. **Given** a practitioner needs to implement inverse kinematics for a 7-DOF humanoid arm, **When** they navigate to the kinematics chapter, **Then** they find mathematical derivations, algorithmic pseudocode, and production-quality Python/ROS 2 implementation they can adapt.

2. **Given** a practitioner is debugging a VSLAM pipeline, **When** they consult the NVIDIA Isaac module, **Then** they find technical details on sensor fusion, coordinate frame transformations, and troubleshooting guidance for common failure modes.

3. **Given** a practitioner wants to cite the textbook in technical documentation or papers, **When** they check the references section, **Then** all claims are backed by peer-reviewed sources, equations are properly derived, and content meets academic citation standards.

---

### User Story 4 - Capstone Project Implementation (Priority: P2)

A learner or team wants to build a complete autonomous humanoid system by following the capstone project guide, integrating voice commands, navigation, perception, and manipulation.

**Why this priority**: Demonstrates end-to-end application of all concepts. Capstone projects are critical for cementing learning and showcasing skills.

**Independent Test**: Team follows capstone guide, integrates all required components (speech recognition via Whisper, VLA for action planning, Isaac for perception, ROS 2 for control), and demonstrates a working autonomous humanoid performing a realistic task (e.g., "fetch me a bottle from the shelf"). Success validated by functional demonstration.

**Acceptance Scenarios**:

1. **Given** a learner has completed Modules 0-4, **When** they start the capstone project, **Then** the guide provides clear architecture diagrams, component integration steps, and debugging strategies for common integration issues.

2. **Given** a team is implementing the voice-to-action pipeline, **When** they integrate Whisper speech recognition with ROS 2 actions, **Then** provided code examples demonstrate the interface pattern, error handling, and latency optimization.

3. **Given** a team completes the capstone, **When** they test the system end-to-end, **Then** the humanoid successfully interprets voice commands, plans collision-free paths, localizes itself and objects in the environment, and executes manipulation tasks safely.

---

### Edge Cases

- **What happens when a reader's hardware environment differs from specified requirements?** The textbook provides cloud alternatives (AWS g5/g6 instances) and discusses minimum vs. recommended specifications. Code examples include environment detection and graceful degradation.

- **How does the system handle readers skipping prerequisites?** Each chapter explicitly lists prerequisites with links to relevant prior chapters and external resources. Exercises in early sections test foundational knowledge needed for later chapters.

- **What if code examples fail due to dependency version conflicts?** All code uses strict version pinning (e.g., `numpy==1.24.3`, `ros2==humble`). Installation guides include containerized environments (Docker) to ensure reproducibility.

- **How are safety-critical operations handled for readers with physical robots?** All hardware-related code includes prominent safety warnings ("Run in simulation first", "E-Stop accessible", "Torque limits enforced"). Simulation-first workflow is mandatory before hardware deployment.

- **What if external tools/APIs change after publication?** The textbook documents specific versions (ROS 2 Humble, Isaac Sim 4.x, Unity 2022.x) and provides migration guides for version updates in supplementary materials.

---

## Requirements *(mandatory)*

### Functional Requirements

#### Content Structure

- **FR-001**: The textbook MUST contain **exactly 21 chapters** organized into 6 modules: Module 0 (Physical AI Foundations, **3 chapters**, Weeks 1-2), Module 1 (ROS 2: The Robotic Nervous System, **5 chapters**, Weeks 3-5), Module 2 (Digital Twin: Gazebo + Unity, **4 chapters**, Weeks 6-7), Module 3 (NVIDIA Isaac: The AI-Robot Brain, **5 chapters**, Weeks 8-10), Module 4 (Vision-Language-Action Robotics, **3 chapters**, Weeks 11-12), and Capstone (Integrated Physical AI System, **1 chapter**, Week 13).

- **FR-002**: Every chapter MUST include the following **12 sections** in order: (1) Frontmatter (title, description, sidebar_position, keywords), (2) Learning Objectives (3-7 measurable, active voice), (3) Prerequisites (internal + external links), (4) Weekly Mapping (e.g., "Week 4 – Tuesday & Thursday"), (5) Motivating Scenario (real-world humanoid task), (6) Core Theory & Mathematics (with full derivations), (7) Worked Example (step-by-step), (8) Hands-On Code (runnable, version-pinned, safety warnings), (9) Application to Humanoid Robots, (10) Common Pitfalls & Debugging, (11) Exercises (1× Conceptual, 1× Computational, 2-3× Implementation), (12) Further Reading & References.

- **FR-003**: All mathematical equations MUST include derivations or citations to authoritative sources. No equation may be stated without proof or reference.

- **FR-004**: All technical claims (hardware capabilities, algorithm performance, safety requirements) MUST be verifiable through citations, experimental data, or logical derivation.

- **FR-002a**: Every module overview page MUST include a "Weekly Breakdown" table mapping chapters to specific weeks in the 13-week course structure (as defined in the Major Correction clarification session).

- **FR-002b**: The site MUST include a top-level section titled "Building Your Physical AI Lab – Hardware, Cloud, and Budget Options" containing: full hardware tables (RTX workstation, Jetson kit ~$700, robot options), cloud alternative cost calculations, "Economy Jetson Student Kit" table, and warning about latency trap when controlling real robots from cloud.

#### Code Examples

- **FR-005**: Every code example MUST be runnable without modification in the specified environment (Python 3.10+, ROS 2 Humble, NVIDIA Isaac Sim 4.x, etc.).

- **FR-006**: All code dependencies MUST be version-pinned in requirements files (e.g., `requirements.txt` or `package.xml`).

- **FR-007**: Code examples involving physical robots MUST include safety warnings: "Run in simulation first", "E-Stop accessible", "Torque limits enforced".

- **FR-008**: Every code example MUST include inline comments explaining reasoning ("why") rather than syntax ("what").

- **FR-009**: Code examples MUST use a hybrid approach: (1) Simple inline code snippets (≤20 lines, single-file demonstrations) embedded directly in chapter MDX for optimal learning flow, AND (2) Complex multi-file examples, full projects, or examples requiring multiple dependencies stored in `/examples/[chapter]/` directory structure with validation tests or scripts. All code in `/examples/` MUST be referenced from chapters with clear links.

- **FR-009a**: Code example validation MUST be performed via automated CI testing. Validation scripts MUST execute all code examples stored in `/examples/[chapter]/` directories in Docker containers with pinned dependencies matching the specified environment (Python 3.10+, ROS 2 Humble, etc.). Inline MDX code blocks MAY be validated by extracting and executing them or by manual review. CI pipeline MUST fail if code success rate falls below **90% aggregate threshold**, defined as: **At least 90% of all code examples across all 21 chapters must execute successfully** (minimum aggregate pass rate across all `/examples/` code files; if there are 100 total code examples, at least 90 must pass). This aligns with SC-002 requirement for "90% of code examples execute successfully on first attempt."

#### Docusaurus Site

- **FR-010**: The site MUST use Docusaurus 3.x with the classic theme and docs preset, configured for technical documentation with sidebar navigation, search functionality, and mobile-responsive layout.

- **FR-011**: Every Docusaurus page MUST include frontmatter metadata: `title`, `description`, `keywords`, `sidebar_position`.

- **FR-012**: The site MUST build successfully with zero errors using `npm run build`.

- **FR-013**: The site MUST render all pages in under 3 seconds on mobile devices (tested via Lighthouse).

- **FR-014**: The site MUST include SEO optimizations: OpenGraph metadata, `sitemap.xml`, and `robots.txt`.

- **FR-015**: The site MUST have zero broken links (internal or external) validated by automated link checker.

#### Deployment and CI/CD

- **FR-015a**: The site MUST be deployed to GitHub Pages automatically via GitHub Actions CI/CD pipeline triggered on every push to the main branch. The pipeline MUST build the Docusaurus site, run all quality gates (build success, code example validation in Docker containers per FR-009a, image optimization and size validation per FR-019, link checker, Lighthouse performance ≥90), and deploy only if all checks pass.

#### Educational Quality

- **FR-016**: Every chapter MUST start with 3-7 measurable learning objectives stated in active voice (e.g., "explain", "derive", "implement").

- **FR-017**: Prerequisites MUST be explicitly listed with links to prerequisite chapters (internal) or external resources (e.g., linear algebra, Lagrangian mechanics).

- **FR-018**: Exercises MUST include three types: conceptual (testing understanding), computational (requiring calculations), and implementation-focused (requiring code).

- **FR-019**: All diagrams MUST have meaningful alt-text for accessibility, be stored in `/static/img/[chapter]/`, and be ≤ 500KB each. Image optimization MUST be automated via GitHub Actions workflow that compresses images during CI checks. The pipeline MUST fail if any image exceeds 500KB after optimization, ensuring performance compliance without manual intervention. **Failure Remediation**: If CI fails due to oversized images (>500KB after automated lossless compression), the workflow outputs a list of affected files with current sizes. The developer MUST manually reduce image complexity by: (1) reducing image dimensions (e.g., 2000px → 1200px width), (2) simplifying diagram content (fewer elements/gradients), or (3) splitting complex diagrams into multiple smaller images. Re-run CI after manual optimization to verify compliance.

#### Hardware and Simulation Guidance

- **FR-020**: The textbook MUST provide hardware recommendations: RTX workstation, Jetson Orin Nano/NX, RealSense D435i camera, and robot platforms (Unitree Go2, G1, or OP3).

- **FR-021**: The textbook MUST provide cloud computing alternatives (AWS g5/g6 instances) for readers without local GPU workstations.

- **FR-022**: Simulation tools coverage MUST include Gazebo (for ROS 2 integration), Unity (for realistic rendering), and NVIDIA Isaac Sim (for AI/perception).

#### Workflow Compliance

- **FR-023**: All content generation MUST follow the workflow: `/sp.specify` → `/sp.plan` → `/sp.tasks` → Claude Code generates all chapter content (markdown, equations, code examples, exercises) following the approved spec and plan.

- **FR-024**: All architecturally significant decisions (simulation engine choice, ROS version, notation system, module ordering) MUST be documented in ADRs stored in `history/adr/`.

- **FR-025**: Every chapter MUST produce required artifacts: `spec.md`, `plan.md`, `tasks.md` in `specs/[chapter]/` and PHRs in `history/prompts/[chapter]/`.

#### Documentation Management

- **FR-026**: At the start of EVERY chapter generation, Claude Code MUST explicitly call MCP Context-7 tools to retrieve exact documentation for pinned versions (ROS 2 Humble, NVIDIA Isaac Sim 2024.x, Unity 2022.x LTS, Gazebo, Whisper) to guarantee up-to-date syntax and APIs. **Pre-fetching once at project start is INVALID** - Context-7 must be invoked fresh for each chapter to ensure accuracy. **Batch Retrieval**: Context-7 SHOULD retrieve all relevant tool documentations (e.g., ROS 2 + Isaac + Unity + Gazebo + Whisper) in a single batch call at the start of chapter generation to optimize efficiency while maintaining per-chapter freshness guarantee.

- **FR-026a**: Claude Code MUST retrieve Docusaurus 3.x (latest stable within v3) documentation via MCP Context-7 during: (1) initial Docusaurus site setup, AND (2) whenever creating or modifying custom React components, Docusaurus plugins, or site configuration files. Version pinning allows minor/patch updates within v3 while avoiding breaking changes from major version upgrades. This ensures all Docusaurus-specific code uses current v3 API patterns and best practices.

### Key Entities

- **Module**: Top-level organizational unit (6 total including capstone). Exact chapter counts: Module 0 (3 chapters), Module 1 (5 chapters), Module 2 (4 chapters), Module 3 (5 chapters), Module 4 (3 chapters), Capstone (1 chapter). Each module has defined learning outcomes, week mapping, and prerequisites. Module overview pages MUST include "Weekly Breakdown" table.

- **Chapter**: Individual learning unit within a module. Contains all required **12 sections** as defined in FR-002: Frontmatter, Learning Objectives, Prerequisites, Weekly Mapping, Motivating Scenario, Core Theory & Mathematics, Worked Example, Hands-On Code, Application to Humanoid Robots, Common Pitfalls & Debugging, Exercises, Further Reading & References. Each chapter is a standalone Docusaurus page with metadata.

- **Code Example**: Runnable code artifact with version-pinned dependencies, safety warnings, validation tests, and inline documentation. Stored in `/examples/[chapter]/`.

- **Exercise**: Assessment item at the end of each chapter. Three types: conceptual, computational, implementation. Designed to test learning objectives.

- **Reference**: Authoritative source citation (academic papers, technical documentation, standards). Used to support all technical claims and provide deeper reading.

- **Glossary Entry**: Standardized definition of technical term. Stored in global `docs/glossary.md` and linked from first use in each chapter.

- **Notation**: Mathematical symbol definition. Stored in global `docs/notation.md` to ensure consistency across chapters.

- **ADR (Architecture Decision Record)**: Document capturing major decisions (e.g., why ROS 2 Humble over Foxy, why Isaac Sim over Gazebo-only). Stored in `history/adr/`.

- **Hardware Specification**: Required or recommended hardware (GPU, robot platform, sensors). Includes minimum vs. recommended specs and cloud alternatives.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: A reader with undergraduate-level mathematics and basic programming skills can complete Module 0 and Module 1 within 10 hours and successfully run all code examples.

- **SC-002**: 90% of code examples execute successfully on first attempt in the specified environment (validated through automated testing in CI/CD).

- **SC-003**: The Docusaurus site builds in under 2 minutes and achieves a Lighthouse performance score ≥ 90 on desktop and mobile.

- **SC-004**: All chapters pass the 12-section template compliance check (automated validation confirms presence of all required sections as defined in FR-002).

- **SC-005**: Zero broken links across the entire site (validated by automated link checker running in CI/CD).

- **SC-006**: 100% of technical claims are backed by citations, derivations, or verifiable evidence (manual review checklist confirms no unsupported claims).

- **SC-007**: Readers can complete the capstone project in 20-40 hours and demonstrate a functional autonomous humanoid system integrating voice, navigation, perception, and manipulation.

- **SC-008**: The textbook receives positive validation from at least 3 independent reviewers (academics or industry practitioners) confirming technical accuracy and pedagogical quality.

- **SC-009**: All exercises include clear assessment criteria and align with stated learning objectives (validated through instructor review).

- **SC-010**: The site is accessible (WCAG AA compliant) with all images having meaningful alt-text and content readable by screen readers.

- **SC-011**: The "Weekly Breakdown" table is present and 100% accurate on every module overview page, mapping each chapter to specific weeks in the 13-week course structure.

- **SC-012**: The "Building Your Physical AI Lab – Hardware, Cloud, and Budget Options" section exists and contains all required hardware budget tables, cloud cost calculations, "Economy Jetson Student Kit" table, and latency trap warning.

### Assumptions

- Readers have access to either a local GPU workstation (NVIDIA RTX 3060+) or cloud GPU instances (AWS g5/g6).
- Readers are comfortable with command-line interfaces and basic Python programming.
- Readers are willing to work primarily in simulation before attempting hardware deployment.
- The project constitution (v1.1.0) remains stable throughout content development.
- ROS 2 Humble, NVIDIA Isaac Sim 4.x, and specified tool versions remain supported for at least 2 years post-publication.
- Readers have approximately 60-80 hours total to complete all modules and capstone project.

---

## Dependencies

### External Dependencies

- **ROS 2 Humble**: Middleware for robot control. Version-pinned to Humble LTS release for 5-year support window.
- **NVIDIA Isaac Sim 4.x**: Simulation and AI platform. Requires NVIDIA GPU and Isaac Sim license (free for educational use).
- **Gazebo Classic 11 / Gazebo Sim (Harmonic)**: Physics simulation for Digital Twin module.
- **Unity 2022.x LTS**: Game engine for high-fidelity rendering in Digital Twin module.
- **Whisper (OpenAI)**: Speech recognition for VLA module. Version-pinned to stable release.
- **Docusaurus 3.x**: Static site generator for textbook website. Version-pinned to latest stable within v3 (allows minor/patch updates, no major version changes).
- **Python 3.10+**: Programming language for all code examples.
- **Git / GitHub**: Version control and GitHub Pages deployment.

### Internal Dependencies

- Constitution v1.1.0 must be ratified and stable before content generation begins.
- Spec-Kit Plus workflow tools must be functional (`/sp.specify`, `/sp.plan`, `/sp.tasks`).
- MCP Context-7 must be available to retrieve versioned documentation at the start of EVERY chapter generation (ROS 2 Humble, NVIDIA Isaac Sim 4.x, Unity 2022.x LTS, Gazebo, Whisper). Context-7 is invoked per-chapter, not pre-fetched once at project start. Additionally, Context-7 must retrieve Docusaurus 3.x documentation during initial site setup and whenever modifying site components/config.
- Templates (spec, plan, tasks, ADR, PHR) must be validated and ready for use.

---

## Scope

### In Scope

- Complete textbook content for **exactly 21 chapters** across 6 modules total (Module 0-4 + Capstone) covering Physical AI and Humanoid Robotics.
- Runnable, validated code examples for all chapters with version-pinned dependencies.
- Docusaurus website with full SEO, accessibility, and performance optimizations.
- Capstone project (full standalone chapter) integrating voice commands, navigation, perception, and manipulation.
- "Building Your Physical AI Lab" section with hardware recommendations, budget tables (~$700 Jetson kit, RTX workstation, robot options), and cloud computing alternatives.
- "Weekly Breakdown" tables on all module overview pages mapping chapters to 13-week course structure.
- Glossary and notation reference system.
- ADRs for all major technical decisions.
- Deployment to GitHub Pages with automated CI/CD pipeline.

### Out of Scope

- Physical robot hardware provision (textbook provides specs and sourcing guidance only).
- Cloud computing cost reimbursement (readers responsible for AWS/cloud expenses).
- Interactive video content or animations (textbook is text, diagrams, and code).
- Custom simulation environments beyond Gazebo, Unity, and Isaac Sim.
- Support for ROS 1 (textbook focuses exclusively on ROS 2 Humble).
- Multi-language translations (English only for initial release).
- Instructor solution manuals (exercises are pedagogical; readers solve independently).
- Certification or formal accreditation (textbook is educational resource, not certification program).

---

## Risks and Mitigations

### Risk 1: External Tool Version Changes

**Description**: ROS 2, NVIDIA Isaac, Unity, or other dependencies release breaking changes during textbook development.

**Impact**: Code examples may fail, requiring significant rework.

**Mitigation**:
- Pin all versions at project start and maintain version lock throughout development.
- Document specific versions in constitution and spec.
- Plan for post-publication maintenance guide with migration paths for major version updates.

### Risk 2: Hardware Unavailability

**Description**: Recommended hardware (Jetson Orin, specific robot platforms) becomes unavailable or discontinued.

**Impact**: Readers cannot replicate hardware setups.

**Mitigation**:
- Provide cloud alternatives (AWS g5/g6) for all GPU-dependent tasks.
- Document multiple robot platform options (Unitree Go2, G1, OP3) rather than single platform.
- Emphasize simulation-first approach so hardware is optional for learning.

### Risk 3: Content Volume Underestimation

**Description**: Each chapter requires more depth than anticipated, causing timeline overruns.

**Impact**: Project delays or incomplete modules.

**Mitigation**:
- Use `/sp.tasks` to break each chapter into granular, estimable tasks.
- Prioritize modules (Module 0, 1, 2 before 3, 4) to ensure MVP delivery.
- Leverage Claude Code for deterministic, spec-compliant content generation.

### Risk 4: Technical Accuracy Challenges

**Description**: Complex robotics math (kinematics, dynamics, control theory) may contain errors.

**Impact**: Loss of credibility, incorrect reader understanding.

**Mitigation**:
- Mandate derivations or citations for all equations (FR-003).
- Implement peer review by robotics experts before publication.
- Include validation tests for all mathematical examples (computed values must match expected results).

---

**END OF SPECIFICATION**
