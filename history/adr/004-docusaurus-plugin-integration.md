# ADR-004: Docusaurus Plugin Integration for Chatbot UI

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** rag-chatbot-integration
- **Context:** Need to integrate RAG chatbot UI into existing Docusaurus textbook site. Requirements: (1) chatbot widget accessible from every page, (2) detect user text selection to enable "Ask about this" feature, (3) avoid rebuilding entire site infrastructure, (4) maintain Docusaurus theme consistency, (5) support both desktop and mobile browsers. Constraints: Docusaurus 3.0+ plugin system, React 18+ components, browser Selection API compatibility.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects user experience, browser compatibility, site maintainability
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Plugin vs standalone site vs browser extension vs iframe embed
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts all frontend development, Docusaurus upgrades, UX design
-->

## Decision

**Adopt Docusaurus plugin architecture with theme swizzling and browser Selection API:**

**Plugin Structure**:
- Create custom Docusaurus plugin: `src/plugins/chatbot-plugin/index.ts`
- Use theme swizzling to inject `ChatbotWrapper` component globally
- Mount persistent `ChatWidget` (floating button) on every page via plugin lifecycle hook

**Text Selection Mechanism**:
- Use browser `Selection API` to detect text highlights via `document.getSelection()`
- Implement `SelectionHandler.tsx` component with `mouseup` event listener
- Inject "Ask about this" button overlay at cursor position when text selected
- Store selected text + source location (`file_path`, `line_number`) in React state

**Component Architecture**:
```
ChatbotWrapper (theme swizzle - global mount point)
  └─ ChatWidget (persistent floating button)
       └─ ChatInterface (modal on click)
            ├─ MessageList (conversation display)
            ├─ InputBox (user input field)
            ├─ CitationLink (clickable sources)
            ├─ FeedbackButtons (thumbs up/down)
            └─ SelectionHandler (text selection logic)
```

**Browser Compatibility**: Target Chrome 90+, Firefox 88+, Safari 14+ (covers 95% of students).

**Styling**: Use CSS Modules (`styles.module.css`) to avoid conflicts with Docusaurus theme. Respect existing color scheme and dark mode.

## Consequences

### Positive

- **Seamless Integration**: Plugin hooks into Docusaurus build pipeline—no separate deployment needed
- **Theme Consistency**: Chatbot UI inherits Docusaurus theme (colors, fonts, dark mode) automatically
- **Native UX**: Text selection feels native to textbook—no context switching to external site
- **Lightweight**: ~15KB gzipped for chatbot components (negligible impact on page load time)
- **Maintainability**: Plugin isolated from textbook content—updates to chapters don't affect chatbot code
- **Reusability**: Plugin can be extracted to npm package for other Docusaurus projects
- **Mobile Support**: Selection API works on mobile browsers (long-press to select text)

### Negative

- **Docusaurus Coupling**: Tightly coupled to Docusaurus plugin API—migrating to different static site generator (Nextra, MkDocs) requires full rewrite
- **Theme Swizzling Risk**: Docusaurus theme upgrades may break swizzled components (requires testing on each Docusaurus version bump)
- **Browser API Dependency**: Selection API not supported in Internet Explorer 11 (acceptable—IE11 <1% market share in 2025)
- **Plugin Complexity**: Custom plugin adds build-time complexity—errors harder to debug than standalone React app
- **Performance Overhead**: Chatbot components loaded on every page even if user never clicks widget (mitigated by code splitting)
- **Limited Customization**: Constrained by Docusaurus plugin lifecycle—cannot customize page routing or SSR behavior

## Alternatives Considered

### Alternative 1: Separate Standalone Chatbot Site

**Description**: Build standalone Next.js chatbot site (`chat.textbook.com`) with iframe embed or popup link from main Docusaurus site.

**Tradeoffs**:
- ✅ Framework-agnostic—works with any textbook platform (Docusaurus, Nextra, MkDocs)
- ✅ Independent deployment—chatbot updates don't require rebuilding textbook site
- ✅ Full control over UI/UX—no Docusaurus constraints
- ❌ Poor UX—user must switch tabs/windows to ask questions (context loss)
- ❌ No text selection integration—cannot capture selected text from main site due to cross-origin restrictions
- ❌ Inconsistent theming—standalone site has different colors, fonts, layout
- ❌ Extra deployment—requires separate hosting, domain, CI/CD pipeline

**Rejection Rationale**: Fails core requirement (FR-014: context-aware Q&A with selected text). Cross-origin policies prevent standalone site from accessing textbook page selections. Integrated plugin provides superior UX.

### Alternative 2: Browser Extension

**Description**: Develop Chrome/Firefox browser extension that injects chatbot UI into textbook site via content script.

**Tradeoffs**:
- ✅ Works across any site (not limited to Docusaurus)
- ✅ Can access text selections without cross-origin issues
- ✅ User controls when to enable chatbot (privacy-conscious)
- ❌ Installation friction—students must install extension (reduces adoption by ~70%)
- ❌ Requires separate extension for Chrome, Firefox, Safari (3x maintenance burden)
- ❌ Extension permissions scary—"Read and change data on all websites" warning deters users
- ❌ Mobile not supported—browser extensions don't work on mobile Safari/Chrome

**Rejection Rationale**: Installation friction unacceptable for educational tool. Target users (students) expect zero-setup experience. Mobile support critical (40% of traffic on mobile).

### Alternative 3: Iframe Embed

**Description**: Build chatbot as standalone app, embed in Docusaurus pages via `<iframe>` tag in footer.

**Tradeoffs**:
- ✅ Framework-agnostic—chatbot code independent of Docusaurus
- ✅ Simpler deployment—chatbot app hosted separately, embedded via HTML tag
- ❌ Cannot access parent page selections—cross-origin restrictions block `window.parent.getSelection()`
- ❌ Styling conflicts—iframe has separate CSS context, difficult to match Docusaurus theme
- ❌ Performance overhead—iframe loads separate HTML/CSS/JS bundle even if unused
- ❌ Accessibility issues—screen readers struggle with nested iframes

**Rejection Rationale**: Same cross-origin limitation as standalone site—cannot access text selections. Iframe styling and accessibility issues provide poor student experience.

### Alternative 4: Headless CMS Approach

**Description**: Use Docusaurus as headless content source, build custom Next.js site with chatbot integrated natively.

**Tradeoffs**:
- ✅ Full control—no Docusaurus constraints on UI/UX
- ✅ Chatbot integration native (no plugin complexity)
- ✅ Modern framework—Next.js offers better performance (ISR, SSR) than static Docusaurus
- ❌ Massive rebuild—existing 21 chapters (34 files total) need migration to new site
- ❌ Loss of Docusaurus features—versioning, i18n, search require reimplementation
- ❌ Higher maintenance—Next.js upgrades, custom routing, SEO optimization
- ❌ Timeline impact—3-4 weeks to rebuild site vs 1 week for plugin

**Rejection Rationale**: Over-engineering for MVP. Existing Docusaurus site works well. Plugin approach achieves same chatbot functionality without discarding 21 chapters (34 files total) of infrastructure.

## References

- Feature Spec: [specs/002-rag-chatbot-integration/spec.md](../../specs/002-rag-chatbot-integration/spec.md) (FR-014: Text selection via Docusaurus hooks)
- Implementation Plan: [specs/002-rag-chatbot-integration/plan.md](../../specs/002-rag-chatbot-integration/plan.md) (Frontend Integration, Phase 0 Task 0.4)
- Related ADRs: ADR-001 (RAG Modular Skills), ADR-002 (Technology Stack - React 18+ components)
- Clarification Decision: Clarification Q1 (User chose Docusaurus plugin hooks for text selection)
- Component Structure: `src/components/RAGChatbot/` and `src/plugins/chatbot-plugin/` (defined in plan.md)
- Browser Compatibility: Success Criteria SC-008 (chatbot loads <2s p95 across desktop/mobile)
