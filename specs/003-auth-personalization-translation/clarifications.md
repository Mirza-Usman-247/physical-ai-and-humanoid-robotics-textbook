# Clarifications: Auth, Personalization & Translation Integration

**Feature**: 003-auth-personalization-translation
**Date**: 2025-12-16
**Status**: Resolved

This document addresses all ambiguities identified in the specification with explicit decisions and assumptions.

---

## 1. Signup Questionnaire: Exact Questions

### Decision

**Software Background Question:**
- **Label**: "What programming languages and technologies are you familiar with?"
- **Input Type**: Multi-select checkboxes
- **Options**:
  - [ ] Python
  - [ ] Machine Learning / Deep Learning
  - [ ] ROS / ROS2
  - [ ] Linux / Ubuntu
  - [ ] C++ / C
  - [ ] JavaScript / Web Development
  - [ ] None (I'm a beginner)
  - [ ] Other: [Free text input]

**Hardware Access Question:**
- **Label**: "What hardware do you have access to?"
- **Input Type**: Single-select radio buttons (mutually exclusive)
- **Options**:
  - ( ) RTX GPU (Local NVIDIA GPU for training/inference)
  - ( ) Jetson Device (Edge AI device like Nano, Xavier, Orin)
  - ( ) Physical Robot (Access to real humanoid or robotic hardware)
  - ( ) Cloud Only (No local hardware, will use cloud resources)

**Additional Context Question** (Optional):
- **Label**: "What's your primary goal with this textbook?" (Optional)
- **Input Type**: Free text (single line, max 200 characters)
- **Placeholder**: "e.g., Build a robot, Learn ROS2, Implement humanoid AI..."

### Rationale

- **Multi-select for software**: Users often know multiple languages/technologies
- **Single-select for hardware**: Primary constraint for tutorials (most users have one primary setup)
- **"None/Beginner" option**: Explicitly includes absolute beginners
- **Optional goal field**: Captures user intent without forcing completion

### Storage Format

```json
// Profile.software_skills (JSONB column)
{
  "languages": ["Python", "C++"],
  "technologies": ["Machine Learning", "ROS"],
  "other": "Rust and embedded systems",
  "is_beginner": false
}

// Profile.hardware_access (ENUM column)
"GPU" | "Jetson" | "Robot" | "CloudOnly"

// Profile.learning_goal (TEXT column, nullable)
"Build autonomous navigation system for mobile robots"
```

---

## 2. Personalization vs RAG: Interaction Model

### Decision

**Personalization = Static Chapter Rewrite (One-time transformation)**

- User clicks "Personalize Content" → System **rewrites the entire chapter** once
- Original chapter markdown is transformed into personalized version
- Personalized content **replaces** the page view (not an overlay)
- User can click "Show Original" to toggle back to standard content
- Personalization is **chapter-scoped**, not answer-scoped

**RAG = Dynamic Q&A (Real-time responses)**

- User asks questions in chatbot → RAG retrieves context and generates answers
- RAG operates independently of personalization
- RAG does NOT personalize chapter content

### Key Differences

| Aspect | Personalization | RAG Chatbot |
|--------|-----------------|-------------|
| **Trigger** | Manual button click | User question in chat |
| **Scope** | Entire chapter | Single Q&A exchange |
| **Output** | Rewritten markdown | Conversational answer |
| **Persistence** | Session-based (cached) | No caching (real-time) |
| **Profile Use** | Always uses profile | Does NOT use profile |
| **Content Type** | Explanations, examples, depth | Direct answers with citations |

### Example

**Original Chapter Section (Standard):**
```markdown
## Forward Kinematics

Forward kinematics computes the end-effector pose from joint angles.
```

**Personalized for Python + CloudOnly User:**
```markdown
## Forward Kinematics (Personalized for Python)

Forward kinematics computes the end-effector pose from joint angles.

### Python Example Using NumPy
Since you're familiar with Python, here's a NumPy implementation:
```python
import numpy as np
def forward_kinematics(theta1, theta2, l1, l2):
    x = l1*np.cos(theta1) + l2*np.cos(theta1+theta2)
    y = l1*np.sin(theta1) + l2*np.sin(theta1+theta2)
    return x, y
```

### Cloud Simulation Alternative
Since you don't have physical hardware, try this in Google Colab...
```

**RAG Chatbot (Unchanged by Personalization):**
- User asks: "What is forward kinematics?"
- RAG retrieves chunks from original chapter + generates answer
- Answer is NOT personalized (works same for all users)

### Rationale

- **Separation of concerns**: Personalization = content adaptation, RAG = Q&A assistance
- **No skill duplication**: Personalization uses `personalization-skill`, RAG uses `rag-answerer` (separate AI skills)
- **User control**: Explicit button click for personalization, not automatic

---

## 3. Personalization Caching Strategy

### Decision

**Session-based caching with 30-minute TTL**

**First Request (Cache Miss):**
1. User clicks "Personalize Content"
2. System invokes `personalization-skill` (takes ~10 seconds)
3. Personalized markdown is generated
4. Result is stored in Redis cache with key: `personalized:${userId}:${chapterId}`
5. TTL = 30 minutes
6. Display personalized content

**Subsequent Requests (Cache Hit):**
1. User refreshes page or returns to same chapter within 30 minutes
2. System checks Redis cache
3. If cached version exists → Display immediately (< 100ms)
4. If cache expired → Regenerate (step 1)

**Cache Invalidation Triggers:**
- User updates profile (software skills or hardware access)
- 30-minute TTL expires
- User clicks "Regenerate Personalization" button

### Cache Key Structure

```
personalized:{userId}:{chapterId}:{profileHash}
```

Where:
- `userId`: UUID
- `chapterId`: e.g., "module-1-ros2-chapter-1"
- `profileHash`: MD5 of user's profile JSON (software_skills + hardware_access)

### Example Flow

```
User A (Python + GPU) visits Chapter 1:
  Cache key: personalized:user-123:module-1-ch1:hash-abc
  Status: MISS → Generate → Store (TTL 30min) → Display

User A refreshes page (20 minutes later):
  Cache key: personalized:user-123:module-1-ch1:hash-abc
  Status: HIT → Display immediately

User A updates profile (adds "ROS" skill):
  Profile hash changes: hash-abc → hash-xyz
  Cache key: personalized:user-123:module-1-ch1:hash-xyz
  Status: MISS → Generate → Store → Display
```

### Storage Backend

**Redis Cache:**
- Fast lookup (<100ms)
- Automatic TTL expiration
- Low storage cost (personalized content ~50KB per chapter)
- Estimated cache size: 10,000 users × 50 chapters × 50KB = 25GB

**NOT stored in Postgres:**
- Personalized content is ephemeral (regeneratable)
- Reduces database bloat
- Cache invalidation is simpler

### Rationale

- **Performance**: 10 second wait only on first request
- **Freshness**: 30-minute TTL balances speed and profile updates
- **Cost**: Redis is cheaper than regenerating every request
- **User control**: Profile updates trigger regeneration automatically

---

## 4. Urdu Translation: Technical Term Handling

### Decision

**Technical terms are PRESERVED in English**

**Translation Rules:**

1. **Preserve English:**
   - Technical terms: `kinematics`, `forward kinematics`, `end-effector`, `DH parameters`, `Jacobian`, `ROS2`, `API`, `URDF`, `Gazebo`
   - Programming keywords: `def`, `class`, `import`, `if`, `for`, `while`
   - Code snippets: Entire code blocks remain in English
   - Mathematical symbols: `θ`, `α`, `Σ`, `∫`

2. **Translate to Urdu:**
   - Explanatory text: "The robot moves from point A to point B"
   - Instructions: "Follow these steps to install ROS2"
   - General concepts: "velocity", "position", "rotation" (with English term in parentheses)

3. **Hybrid Format (Technical term + Urdu explanation):**
   - "Forward kinematics (آگے کی حرکیات) وہ عمل ہے جو جوائنٹ اینگلز سے end-effector کی پوزیشن کا حساب لگاتا ہے۔"
   - Translation: "Forward kinematics (forward motion) is the process that calculates the end-effector position from joint angles."

### Example Translation

**Original English:**
```markdown
## Forward Kinematics

Forward kinematics computes the end-effector pose from joint angles using the DH convention.

### Python Implementation
```python
def forward_kinematics(theta, d, a, alpha):
    # Compute transformation matrix
    return transformation_matrix
```

The Jacobian matrix relates joint velocities to end-effector velocities.
```

**Urdu Translation:**
```markdown
## Forward Kinematics

Forward kinematics (آگے کی حرکیات) جوائنٹ اینگلز سے end-effector کی pose (پوزیشن اور زاویہ) کا حساب لگاتا ہے، DH convention استعمال کرتے ہوئے۔

### Python Implementation (پائتھون میں نفاذ)
```python
def forward_kinematics(theta, d, a, alpha):
    # Compute transformation matrix
    return transformation_matrix
```

Jacobian matrix جوائنٹ کی رفتار کو end-effector کی رفتار سے جوڑتا ہے۔
```

### Translation Quality Expectations

**Measurable Criteria:**

- **SC-TR-001**: 95% of technical terms remain in English unchanged
- **SC-TR-002**: 100% of code snippets remain unchanged
- **SC-TR-003**: Urdu text is grammatically correct and readable by native speakers
- **SC-TR-004**: Translation maintains original meaning (verified by bilingual review)
- **SC-TR-005**: Mathematical equations and symbols remain unchanged

### Rationale

- **Industry standard**: Technical documentation commonly uses English terms in Urdu text
- **Clarity**: Avoids confusion from poorly translated technical jargon
- **Searchability**: Users can search for English terms (SEO)
- **Code integrity**: No risk of breaking code with translation

---

## 5. User Control: Reverting to Original Content

### Decision

**Toggle mechanism with persistent preference**

**UI Controls:**

1. **Personalized Content View:**
   - Top banner: "Viewing personalized content for your profile"
   - Button: "Show Original Content" (toggles to standard)
   - Button: "Regenerate Personalization" (re-runs skill with updated profile)

2. **Translated Content View:**
   - Top banner: "Viewing Urdu translation"
   - Button: "Show Original English" (toggles to English)
   - Translation is NOT persistent (refresh → returns to English)

3. **Standard Content View:**
   - Buttons: "Personalize Content" | "Translate to Urdu"

### Toggle Behavior

**Personalization Toggle:**
- Clicking "Show Original" → Displays standard English content
- Preference saved in session: `viewing_mode = "standard"`
- Refreshing page respects preference (shows standard, not personalized)
- User must click "Personalize Content" again to re-enable

**Translation Toggle:**
- Clicking "Show Original English" → Displays standard English content
- Refreshing page → ALWAYS returns to English (translation not persistent)
- User must click "Translate to Urdu" again to re-translate

### Preference Storage

**Session Storage (not Postgres):**
```javascript
sessionStorage.setItem('chapter:module-1-ch1:viewMode', 'personalized');
sessionStorage.setItem('chapter:module-1-ch1:translated', 'false');
```

**Why session storage?**
- Temporary preference (doesn't clutter database)
- Cleared on browser close (fresh start each session)
- Fast lookup (client-side, no server call)

### Multiple Tabs Behavior

- Each tab has independent session storage
- User can view personalized in Tab A, standard in Tab B simultaneously
- No cross-tab synchronization (keep it simple)

### Rationale

- **User control**: Explicit toggle, not forced personalization
- **Transparency**: Clear banner shows current view mode
- **Flexibility**: Easy to compare personalized vs standard
- **No lock-in**: Refreshing page returns to standard (safe default)

---

## 6. Permission Boundaries: Logged-in vs Anonymous

### Decision

**Strict authentication requirement for personalization and translation**

**Permission Matrix:**

| Feature | Anonymous User | Logged-in User |
|---------|----------------|----------------|
| **Read Textbook** | ✅ Full access | ✅ Full access |
| **RAG Chatbot** | ✅ Available | ✅ Available |
| **Personalize Content** | ❌ Hidden | ✅ Visible |
| **Translate to Urdu** | ❌ Hidden | ✅ Visible |
| **View Profile** | ❌ Redirect to login | ✅ Accessible |

### UI Behavior for Anonymous Users

**Chapter Page (Not Logged In):**
- Personalize and Translate buttons are **completely hidden** (not disabled)
- No visual indicators (no greyed-out buttons or "Sign in to personalize" messages)
- Reason: Avoid clutter, keep UI clean for majority use case (reading)

**Optional: Promotional Banner (Dismissible):**
- After reading 3 chapters: "Want personalized content? Sign up to adapt explanations to your skill level"
- User can dismiss permanently (stored in localStorage)

**Enforcement Layer:**

1. **Frontend**: Buttons hidden via `{isAuthenticated && <PersonalizeButton />}`
2. **Backend API**: Authentication middleware on endpoints:
   - `POST /api/personalize` → 401 Unauthorized if no session
   - `POST /api/translate` → 401 Unauthorized if no session
3. **Skills**: Personalization and translation skills require `userId` parameter

### Error Handling

**If anonymous user somehow calls API (e.g., via Postman):**
```json
{
  "error": "authentication_required",
  "message": "You must be logged in to personalize content",
  "code": 401,
  "action": "redirect_to_login"
}
```

### Rationale

- **Privacy**: Personalization requires user profile data (must authenticate)
- **Abuse prevention**: Rate limiting by user ID (not IP)
- **Business value**: Authentication gate encourages signups
- **Clean UX**: No confusing disabled buttons for anonymous users

---

## 7. Skills vs RAG: Separation of Concerns

### Decision

**Distinct AI skills with no shared logic**

**Architecture:**

```
┌─────────────────────────────────────────────────┐
│          Textbook Platform (FastAPI)            │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌──────────────┐      ┌──────────────┐        │
│  │ RAG Chatbot  │      │ Chapter View │        │
│  │              │      │              │        │
│  │  [Ask Q&A]   │      │ [Personalize]│        │
│  │              │      │ [Translate]  │        │
│  └──────┬───────┘      └──────┬───────┘        │
│         │                     │                │
│         ▼                     ▼                │
│  ┌──────────────┐      ┌──────────────┐        │
│  │ RAG Pipeline │      │Content Skills│        │
│  │              │      │              │        │
│  │ • Retriever  │      │• Personalizer│        │
│  │ • Answerer   │      │• Translator  │        │
│  │ • Citations  │      │• Formatter   │        │
│  └──────────────┘      └──────────────┘        │
│         │                     │                │
│         ▼                     ▼                │
│  ┌──────────────────────────────────┐          │
│  │      OpenRouter / DeepSeek       │          │
│  │    (Shared LLM Infrastructure)   │          │
│  └──────────────────────────────────┘          │
└─────────────────────────────────────────────────┘
```

### Skill Boundaries

**RAG Skills (Existing - No Changes):**

1. **rag-retriever**: Semantic search in Qdrant vector DB
2. **rag-answerer**: Generate answers from retrieved chunks
3. **citation-mapper**: Extract and validate citations

**New Content Skills (This Feature):**

1. **personalization-skill**: Rewrite chapter markdown based on user profile
2. **translation-skill**: Translate chapter markdown to Urdu
3. **user-profile-skill**: CRUD operations on user profiles
4. **content-adapter-skill**: Format personalized/translated markdown

### No Shared Logic

**What is NOT shared:**

- **Prompts**: RAG uses Q&A prompts, personalization uses rewriting prompts
- **Context**: RAG uses retrieved chunks, personalization uses full chapter + profile
- **Output**: RAG returns conversational answers, personalization returns markdown
- **Caching**: RAG has no caching, personalization uses Redis cache
- **Profile**: RAG does NOT access user profile, personalization requires it

**What IS shared:**

- **LLM API**: Both use OpenRouter / DeepSeek (same infrastructure)
- **Auth**: Both check if user is logged in (shared session middleware)
- **Database**: Both access Neon Postgres (but different tables)

### Example Request Flows

**RAG Chatbot Request:**
```
User → POST /api/chat/query
     → RAG Orchestrator
     → rag-retriever (fetch chunks from Qdrant)
     → rag-answerer (LLM: generate answer from chunks)
     → citation-mapper (validate citations)
     → Response: {"answer": "...", "sources": [...]}
```

**Personalization Request:**
```
User → POST /api/personalize
     → Personalization Orchestrator
     → user-profile-skill (fetch profile from Postgres)
     → personalization-skill (LLM: rewrite chapter with profile context)
     → content-adapter-skill (format markdown)
     → Cache in Redis
     → Response: {"personalized_content": "...", "cache_key": "..."}
```

### Rationale

- **Modularity**: Each skill has single responsibility
- **Reusability**: Skills can be used in other books/platforms
- **Maintainability**: Bugs in one skill don't affect others
- **Testing**: Each skill can be tested independently
- **Scalability**: Can deploy skills separately (microservices future)

---

## Summary of Decisions

| Clarification | Decision | Key Points |
|---------------|----------|------------|
| **1. Signup Questions** | Multi-select software, single-select hardware, optional goal | 7 software options, 4 hardware options, stored as JSONB + ENUM |
| **2. Personalization vs RAG** | Separate systems: Personalization rewrites chapters, RAG answers questions | No skill overlap, different triggers, different outputs |
| **3. Caching** | Redis cache with 30-minute TTL, invalidated on profile update | Fast retrieval (<100ms), auto-expiration, profile-hash keying |
| **4. Urdu Translation** | Preserve English technical terms, translate explanations only | 95% term preservation, code unchanged, hybrid format |
| **5. User Control** | Toggle buttons with session storage, translation not persistent | "Show Original" button, refresh returns to English for translations |
| **6. Permissions** | Buttons hidden for anonymous, strict API authentication | 401 errors, no disabled UI elements, clean UX |
| **7. Skills vs RAG** | Completely separate skills, shared LLM infrastructure only | No logic duplication, modular architecture, independent testing |

---

## Impact on Functional Requirements

### Updated Requirements

- **FR-009** → Now specifies: Multi-select checkboxes for software, radio buttons for hardware
- **FR-017** → Clarifies: Personalization rewrites chapter once (not per-paragraph)
- **FR-029** → Explicit: Translation is session-only, refresh returns to English
- **FR-025** → Quantified: 95% of technical terms preserved in English
- **FR-036** → Architecture: Personalization skill operates independently of RAG

### New Requirements Added

- **FR-044**: System MUST cache personalized content in Redis with 30-minute TTL
- **FR-045**: System MUST provide "Show Original Content" toggle button when viewing personalized content
- **FR-046**: System MUST hide personalize and translate buttons for anonymous users (not disable)
- **FR-047**: System MUST invalidate personalization cache when user updates profile
- **FR-048**: Translation skill MUST preserve 95% of English technical terms (measured by term count)

---

## Next Steps

**Specification Status**: ✅ All clarifications resolved

**Ready for**: `/sp.plan` (implementation planning)

**No further ambiguities** - all decisions are explicit and measurable.
