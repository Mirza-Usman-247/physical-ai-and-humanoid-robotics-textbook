# Research & Technical Decisions

**Feature**: Auth, Personalization & Translation Integration
**Date**: 2025-12-16
**Status**: Phase 0 Complete

## Summary

This document resolves all technical unknowns identified in `plan.md` Phase 0. Each section documents the decision made, rationale, alternatives considered, and implementation guidance.

---

## 1. Better Auth Python Integration

### Decision

**Use FastAPI-Users library** as Better Auth does not have official Python support. FastAPI-Users provides session-based authentication compatible with FastAPI.

### Rationale

- Better Auth is primarily a JavaScript/TypeScript library (Node.js, Next.js)
- No official Python adapter exists as of 2025-12-16
- FastAPI-Users is the de facto standard for FastAPI authentication
- Supports session-based auth (not just JWT)
- Integrates with SQLAlchemy and psycopg3
- Active maintenance and community support

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **FastAPI-Users** | Native FastAPI support, session-based, SQLAlchemy integration | Requires learning new API | ✅ **SELECTED** |
| **Authlib** | OAuth/OIDC support, flexible | More complex than needed, JWT-focused | ❌ Rejected |
| **Custom session management** | Full control | High maintenance, security risks | ❌ Rejected |
| **Better Auth (wait for Python)** | Spec mentioned Better Auth | Doesn't exist for Python | ❌ Not viable |

### Implementation Guidance

```python
# Installation
# requirements.txt
fastapi-users[sqlalchemy]==12.1.0
httpx==0.25.2

# Key components:
# - FastAPIUsers: Main auth manager
# - CookieTransport: Session-based auth (not JWT)
# - SQLAlchemyUserDatabase: User persistence
# - UserManager: Custom user logic (email validation, etc.)

# Example integration:
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import CookieTransport, AuthenticationBackend

cookie_transport = CookieTransport(cookie_name="auth_session", cookie_max_age=3600)
auth_backend = AuthenticationBackend(
    name="cookie",
    transport=cookie_transport,
    get_strategy=...,  # Session strategy
)
```

**Status**: ✅ Resolved

---

## 2. Neon Postgres Schema Design

### Decision

**Use normalized 3NF schema** with JSONB for software_skills array and ENUM for hardware_access field.

### Rationale

- JSONB provides flexibility for multi-select software skills without junction tables
- ENUM enforces hardware_access constraints at database level
- 3NF normalization prevents duplication (users vs profiles separate tables)
- Neon Postgres supports advanced JSONB indexing (GIN indexes)
- Foreign key constraints ensure referential integrity

### Schema Design

```sql
-- ENUM for hardware access (mutually exclusive per FR-010)
CREATE TYPE hardware_type AS ENUM ('GPU', 'Jetson', 'Robot', 'CloudOnly');

-- Users table (authentication only)
CREATE TABLE users (
    user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE,
    is_superuser BOOLEAN DEFAULT FALSE,
    is_verified BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP
);

-- Profiles table (user background per FR-011)
CREATE TABLE profiles (
    profile_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    software_skills JSONB DEFAULT '[]'::jsonb,  -- ["Python", "ML", "ROS"]
    hardware_access hardware_type,
    learning_goal TEXT,  -- Nullable, max 200 chars enforced in app
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_user_profile UNIQUE (user_id)
);

-- Preferences table (personalization settings per FR-034)
CREATE TABLE preferences (
    preference_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    personalization_level VARCHAR(20) DEFAULT 'basic' CHECK (personalization_level IN ('basic', 'detailed', 'expert')),
    auto_personalize BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT unique_user_preference UNIQUE (user_id)
);

-- Translation logs (optional analytics per FR-035)
CREATE TABLE translation_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    language VARCHAR(20) DEFAULT 'urdu',
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_profiles_user_id ON profiles(user_id);
CREATE INDEX idx_profiles_software_skills ON profiles USING GIN (software_skills);  -- JSONB index
CREATE INDEX idx_translation_logs_user_id ON translation_logs(user_id);
CREATE INDEX idx_translation_logs_chapter_id ON translation_logs(chapter_id);
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **3NF + JSONB + ENUM** | Normalized, flexible, type-safe | Slightly more complex queries | ✅ **SELECTED** |
| **Denormalized (single table)** | Simple queries | Data duplication, update anomalies | ❌ Rejected |
| **Junction table for skills** | Pure relational | Overengineered for 8 skill options | ❌ Rejected |
| **VARCHAR for hardware** | Simple | No type safety, allows invalid values | ❌ Rejected |

### Migration Strategy

```bash
# Alembic migration
alembic revision --autogenerate -m "Create auth and profile tables"
alembic upgrade head
```

**Status**: ✅ Resolved

---

## 3. Redis Caching Strategy

### Decision

**Use Redis with 30-minute TTL** and namespace isolation from RAG chatbot. Cache key format: `personalized:{userId}:{chapterId}:{profileHash}` where profileHash is MD5 of profile fields.

### Rationale

- 30-minute TTL balances freshness with performance (FR-044)
- Profile hash ensures cache invalidation when profile changes (FR-047)
- MD5 is sufficient for non-cryptographic cache keys (fast, collision-resistant enough)
- Namespace prefix prevents key collisions with RAG chatbot cache
- Redis supports atomic TTL operations

### Cache Key Design

```python
import hashlib
import json

def generate_profile_hash(profile: dict) -> str:
    """Generate MD5 hash of profile fields for cache invalidation."""
    profile_string = json.dumps({
        "software_skills": sorted(profile.get("software_skills", [])),
        "hardware_access": profile.get("hardware_access"),
        "learning_goal": profile.get("learning_goal", "")
    }, sort_keys=True)
    return hashlib.md5(profile_string.encode()).hexdigest()[:8]  # First 8 chars

def generate_cache_key(user_id: str, chapter_id: str, profile_hash: str) -> str:
    """Generate Redis cache key for personalized content."""
    return f"personalized:{user_id}:{chapter_id}:{profile_hash}"

# Example:
# Key: "personalized:a1b2c3d4:module-1-chapter-2:8f3d4a21"
# TTL: 1800 seconds (30 minutes)
```

### Cache Operations

```python
import redis

redis_client = redis.Redis(host="localhost", port=6379, db=0, decode_responses=True)

# Set cache with TTL
cache_key = generate_cache_key(user_id, chapter_id, profile_hash)
redis_client.setex(cache_key, 1800, personalized_content)  # 1800 seconds = 30 min

# Get cache
cached_content = redis_client.get(cache_key)

# Invalidate user's cache (when profile updates)
pattern = f"personalized:{user_id}:*"
for key in redis_client.scan_iter(match=pattern):
    redis_client.delete(key)
```

### Namespace Isolation

```python
# RAG chatbot uses different namespace
# RAG keys: "rag:{chapter_id}:{embedding_hash}"
# Personalization keys: "personalized:{user_id}:{chapter_id}:{profile_hash}"
# No collision possible
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Redis + 30-min TTL + MD5** | Fast, simple, sufficient security | Non-cryptographic hash | ✅ **SELECTED** |
| **Redis + SHA256** | Cryptographically secure | Slower, unnecessary for cache keys | ❌ Rejected |
| **Database caching** | No extra infrastructure | Slow, defeats purpose | ❌ Rejected |
| **In-memory cache (Python)** | Very fast | Not shared across workers, no persistence | ❌ Rejected |

**Status**: ✅ Resolved

---

## 4. OpenRouter DeepSeek Integration

### Decision

**Use OpenRouter with DeepSeek R1 (free tier)** for personalization and translation. Context window: 8K tokens. Fallback: GPT-3.5-turbo (paid) if DeepSeek unavailable.

### Rationale

- DeepSeek R1 is free on OpenRouter (per spec requirement)
- Supports 8K context window (sufficient for single chapters)
- Multilingual support (English and Urdu)
- OpenRouter provides unified API for fallback models
- Rate limit: 50 requests/minute (adequate for <10 concurrent personalizations)

### API Configuration

```python
import openai  # OpenRouter uses OpenAI SDK

openai.api_base = "https://openrouter.ai/api/v1"
openai.api_key = os.getenv("OPENROUTER_API_KEY")

# Model IDs
PRIMARY_MODEL = "deepseek/deepseek-r1-free"  # Free tier
FALLBACK_MODEL = "openai/gpt-3.5-turbo"      # Paid fallback

def call_llm(prompt: str, system_prompt: str, model: str = PRIMARY_MODEL) -> str:
    """Call LLM with fallback logic."""
    try:
        response = openai.ChatCompletion.create(
            model=model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": prompt}
            ],
            temperature=0.7,
            max_tokens=4000,
            timeout=30  # 30 second timeout per FR-020, FR-030
        )
        return response.choices[0].message.content
    except openai.error.RateLimitError:
        if model == PRIMARY_MODEL:
            # Fallback to paid model
            return call_llm(prompt, system_prompt, model=FALLBACK_MODEL)
        raise
```

### Prompt Templates

**Personalization Prompt**:
```text
System: You are an AI tutor personalizing robotics textbook content.

User Profile:
- Software Background: {software_skills}
- Hardware Access: {hardware_access}
- Learning Goal: {learning_goal}

Task: Rewrite the following chapter to match the user's skill level and resources.
- Adjust explanation depth based on software background
- Include hardware-specific examples for {hardware_access}
- Preserve all code snippets and equations exactly
- Maintain markdown formatting

Chapter Content:
{chapter_markdown}

Output: Personalized chapter in markdown format.
```

**Translation Prompt**:
```text
System: You are an expert technical translator (English → Urdu).

Rules:
1. Preserve 95% of English technical terms unchanged (kinematics, end-effector, API, ROS2, etc.)
2. Use hybrid format: "Forward kinematics (آگے کی حرکیات)..." for first mention
3. Preserve 100% of code snippets, variable names, function names
4. Preserve all markdown formatting (headings, lists, tables)
5. Translate explanatory text and instructional content to grammatically correct Urdu

Chapter Content:
{chapter_markdown}

Output: Translated chapter in markdown with technical terms preserved.
```

### Rate Limits and Token Budgets

```python
# DeepSeek R1 Free Tier (OpenRouter)
# - Rate limit: 50 requests/minute
# - Context window: 8K tokens (~6000 words)
# - Average chapter: 3000-5000 tokens
# - Average personalization: 4000-6000 tokens output
# - Safety margin: 8K context should handle 95% of chapters

# For long chapters (>6K tokens):
def chunk_chapter(chapter: str, max_tokens: int = 6000) -> list[str]:
    """Split chapter into chunks for processing."""
    # Simple split by ## headings for now
    sections = chapter.split("\n## ")
    chunks = []
    current_chunk = ""
    for section in sections:
        if estimate_tokens(current_chunk + section) < max_tokens:
            current_chunk += "\n## " + section
        else:
            chunks.append(current_chunk)
            current_chunk = "\n## " + section
    chunks.append(current_chunk)
    return chunks

def estimate_tokens(text: str) -> int:
    """Rough token estimate (1 token ≈ 4 characters)."""
    return len(text) // 4
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **DeepSeek R1 (free)** | Free, 8K context, multilingual | Rate limits (50 req/min) | ✅ **SELECTED** |
| **GPT-3.5-turbo** | Reliable, 16K context | Paid ($0.0015/1K tokens) | 🔄 Fallback only |
| **Claude 3 Haiku** | Fast, 200K context | Paid ($0.25/1M tokens) | ❌ Cost prohibitive |
| **Local LLaMA** | Free, no rate limits | Requires GPU, slower | ❌ Infrastructure burden |

**Status**: ✅ Resolved

---

## 5. Docusaurus Theme Customization

### Decision

**Use theme swizzling with MDXContent wrapper** to inject buttons at chapter start. Store view preferences in sessionStorage.

### Rationale

- Docusaurus 3.x supports swizzling MDXContent component
- MDXContent wraps all markdown pages (chapters)
- Session storage is sufficient for temporary preferences (FR-046, FR-045)
- No need for persistent cookies (translation never persists per FR-029)
- React context provides auth state globally

### Implementation Pattern

```typescript
// src/theme/MDXContent/index.tsx (swizzled component)
import React from 'react';
import MDXContent from '@theme-original/MDXContent';
import ChapterWrapper from '@site/src/components/Chapter/ChapterWrapper';

export default function MDXContentWrapper(props) {
  return (
    <>
      <ChapterWrapper />
      <MDXContent {...props} />
    </>
  );
}
```

```typescript
// src/components/Chapter/ChapterWrapper.tsx
import React from 'react';
import { useAuth } from '@site/src/components/Auth/AuthContext';
import PersonalizeButton from './PersonalizeButton';
import TranslateButton from './TranslateButton';
import ContentToggle from './ContentToggle';

export default function ChapterWrapper() {
  const { isAuthenticated } = useAuth();

  // Hide buttons for anonymous users (FR-015, FR-023)
  if (!isAuthenticated) {
    return null;
  }

  return (
    <div className="chapter-actions">
      <PersonalizeButton />
      <TranslateButton />
      <ContentToggle />
    </div>
  );
}
```

### Session Storage Management

```typescript
// src/utils/viewPreferences.ts
export function getViewMode(): 'original' | 'personalized' | 'translated' {
  return sessionStorage.getItem('viewMode') as any || 'original';
}

export function setViewMode(mode: 'original' | 'personalized' | 'translated') {
  sessionStorage.setItem('viewMode', mode);
}

// Clear on page refresh (translation never persists)
window.addEventListener('load', () => {
  const currentMode = getViewMode();
  if (currentMode === 'translated') {
    setViewMode('original');  // Reset to original per FR-029
  }
});
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Theme swizzling + MDXContent** | Clean, Docusaurus-native | Requires swizzling | ✅ **SELECTED** |
| **Plugin approach** | No swizzling | More complex API | ❌ Overengineered |
| **Remark plugin** | Markdown-level injection | Hard to access React state | ❌ Rejected |
| **Manual frontmatter** | Simple | Requires editing all chapters | ❌ Not scalable |

**Status**: ✅ Resolved

---

## 6. Urdu Translation Quality Assurance

### Decision

**Use automated regex validation + manual spot checks** for 95% technical term preservation (FR-048). Implement validation script in CI/CD.

### Rationale

- Regex can detect English technical terms in Urdu text
- Hybrid format pattern is machine-verifiable: `"term (اردو)"`
- Code block preservation is 100% verifiable (regex match)
- Manual review required for grammatical correctness (cannot automate fluency)
- CI/CD validation prevents regressions

### Validation Script

```python
import re

# List of technical terms that MUST be preserved (95% target)
TECHNICAL_TERMS = [
    "kinematics", "forward kinematics", "inverse kinematics",
    "end-effector", "DH parameters", "Jacobian", "Jacobian matrix",
    "ROS2", "API", "URDF", "Gazebo", "trajectory", "pose", "quaternion",
    "robot", "humanoid", "sensor", "actuator", "controller"
]

def validate_term_preservation(original: str, translated: str) -> dict:
    """Validate that 95% of technical terms are preserved."""
    results = {
        "total_terms": len(TECHNICAL_TERMS),
        "preserved_terms": 0,
        "missing_terms": [],
        "preservation_rate": 0.0
    }

    for term in TECHNICAL_TERMS:
        # Check if term appears in translated text (case-insensitive)
        if re.search(rf'\b{re.escape(term)}\b', translated, re.IGNORECASE):
            results["preserved_terms"] += 1
        else:
            results["missing_terms"].append(term)

    results["preservation_rate"] = results["preserved_terms"] / results["total_terms"]
    return results

def validate_code_preservation(original: str, translated: str) -> bool:
    """Validate that 100% of code blocks are unchanged."""
    original_code_blocks = re.findall(r'```[\s\S]*?```', original)
    translated_code_blocks = re.findall(r'```[\s\S]*?```', translated)

    if len(original_code_blocks) != len(translated_code_blocks):
        return False

    for orig, trans in zip(original_code_blocks, translated_code_blocks):
        if orig != trans:
            return False

    return True

def validate_hybrid_format(translated: str) -> dict:
    """Validate hybrid format usage: 'term (اردو)'."""
    # Detect pattern: English word followed by Urdu in parentheses
    hybrid_pattern = r'\b[A-Za-z\-]+\s*\([^\)]*[\u0600-\u06FF]+[^\)]*\)'
    matches = re.findall(hybrid_pattern, translated)

    return {
        "hybrid_instances": len(matches),
        "examples": matches[:5]  # First 5 examples
    }

# Example usage in tests:
def test_translation_quality():
    original_chapter = load_chapter("module-1-chapter-1.md")
    translated_chapter = translate_chapter(original_chapter)

    term_results = validate_term_preservation(original_chapter, translated_chapter)
    code_preserved = validate_code_preservation(original_chapter, translated_chapter)
    hybrid_results = validate_hybrid_format(translated_chapter)

    # FR-048 requirements
    assert term_results["preservation_rate"] >= 0.95, f"Only {term_results['preservation_rate']*100}% terms preserved"
    assert code_preserved, "Code blocks were modified"
    assert hybrid_results["hybrid_instances"] > 0, "No hybrid format detected"
```

### Manual Review Workflow

```markdown
# Translation Review Checklist (Manual)

For each translated chapter, verify:

1. ✅ Technical terms preserved (automated)
2. ✅ Code blocks unchanged (automated)
3. ✅ Hybrid format used correctly (automated)
4. ⏱️ **Grammar and fluency** (manual review required)
   - Urdu prose is grammatically correct
   - Explanations are clear and natural
   - No awkward literal translations
5. ⏱️ **Meaning preservation** (manual review required)
   - Original intent conveyed accurately
   - No information lost or added

**Spot Check**: Review 3 random translated chapters per release.
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Regex validation + manual spot checks** | Automated, scalable, catches 90% of issues | Doesn't validate fluency | ✅ **SELECTED** |
| **Fully manual review** | Catches all issues | Not scalable, slow | ❌ Rejected |
| **LLM-based validation** | Could check fluency | Expensive, unreliable | ❌ Not mature enough |
| **No validation** | Simple | Violates FR-048 | ❌ Unacceptable |

**Status**: ✅ Resolved

---

## 7. Security and Session Management

### Decision

**Use CSRF protection middleware, Redis-backed sessions, and parameterized queries** with psycopg3. Session timeout: 1 hour. HTTPS required in production.

### Rationale

- FastAPI-Users provides built-in CSRF protection for session-based auth
- Redis sessions allow horizontal scaling (multiple FastAPI workers)
- psycopg3 parameterized queries prevent SQL injection
- 1-hour session timeout balances security with UX
- HTTPS is mandatory for production (session cookies must be secure)

### Security Checklist

```python
# 1. CSRF Protection (FastAPI middleware)
from starlette.middleware.sessions import SessionMiddleware
from starlette.middleware.cors import CORSMiddleware

app.add_middleware(
    SessionMiddleware,
    secret_key=os.getenv("SESSION_SECRET"),
    session_cookie="session_id",
    max_age=3600,  # 1 hour
    same_site="lax",  # CSRF protection
    https_only=True   # Production only
)

# 2. CORS Configuration (frontend-backend communication)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://your-docusaurus-site.com"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"]
)

# 3. SQL Injection Prevention (psycopg3 parameterized queries)
async with db.execute(
    "SELECT * FROM users WHERE email = %s",
    (email,)  # Parameterized - safe from injection
) as cursor:
    user = await cursor.fetchone()

# 4. Password Hashing (FastAPI-Users uses passlib bcrypt)
from passlib.context import CryptContext
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
hashed_password = pwd_context.hash(plain_password)

# 5. Rate Limiting (optional but recommended)
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter

@app.post("/auth/signup")
@limiter.limit("5/minute")  # Max 5 signups per minute per IP
async def signup(request: Request, ...):
    pass
```

### Session Storage Design

```python
# Redis-backed sessions for scalability
import redis.asyncio as redis
from starlette.middleware.sessions import SessionMiddleware

redis_client = redis.from_url("redis://localhost:6379", decode_responses=True)

# Custom session backend (if needed)
# Default: SessionMiddleware uses signed cookies (sufficient for <500 users)
# For >500 concurrent users: Use Redis session store

# Session data structure:
# Key: "session:{session_id}"
# Value: JSON with user_id, email, is_authenticated, created_at, last_activity
# TTL: 3600 seconds (1 hour)
```

### HTTPS Requirements

```yaml
# Production deployment (GitHub Pages uses HTTPS by default)
# Backend API must also use HTTPS

# nginx configuration for FastAPI backend:
server {
    listen 443 ssl;
    server_name api.your-site.com;

    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;

    location / {
        proxy_pass http://localhost:8000;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

### Alternatives Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **CSRF + Redis sessions + HTTPS** | Comprehensive, scalable | More complex setup | ✅ **SELECTED** |
| **JWT tokens** | Stateless | Harder to revoke, spec requires sessions | ❌ Rejected |
| **Signed cookies (no Redis)** | Simpler | Not scalable beyond 1 server | 🔄 OK for MVP, Redis for prod |
| **No CSRF protection** | Simple | Vulnerable to attacks | ❌ Unacceptable |

**Status**: ✅ Resolved

---

## Summary of Decisions

| Research Area | Decision | Status |
|---------------|----------|--------|
| 1. Authentication | FastAPI-Users (Better Auth not available for Python) | ✅ |
| 2. Database Schema | 3NF + JSONB + ENUM | ✅ |
| 3. Caching | Redis 30-min TTL + MD5 profile hash | ✅ |
| 4. LLM Integration | DeepSeek R1 (free) with GPT-3.5 fallback | ✅ |
| 5. Theme Customization | MDXContent swizzling + sessionStorage | ✅ |
| 6. Translation QA | Regex validation + manual spot checks | ✅ |
| 7. Security | CSRF + Redis sessions + HTTPS + parameterized queries | ✅ |

**Phase 0 Complete**: All technical unknowns resolved. Ready for Phase 1 (data model, contracts, quickstart).
