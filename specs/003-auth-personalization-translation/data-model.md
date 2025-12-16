# Data Model Design

**Feature**: Auth, Personalization & Translation Integration
**Date**: 2025-12-16
**Source**: Entities from spec.md + research.md decisions

## Entity-Relationship Diagram

```
┌─────────────────┐
│     Users       │
│ (Authentication)│
├─────────────────┤
│ user_id (PK)    │◄─────┐
│ email           │      │
│ hashed_password │      │ 1:1
│ is_active       │      │
│ is_verified     │      │
│ created_at      │      │
│ last_login      │      │
└─────────────────┘      │
                         │
                         │
┌─────────────────┐      │
│    Profiles     │      │
│ (User Background)      │
├─────────────────┤      │
│ profile_id (PK) │      │
│ user_id (FK)    │──────┘
│ software_skills │ (JSONB: ["Python", "ML", "ROS"])
│ hardware_access │ (ENUM: GPU/Jetson/Robot/CloudOnly)
│ learning_goal   │ (TEXT, nullable)
│ updated_at      │
└─────────────────┘
        ▲
        │ 1:1
        │
┌─────────────────┐
│  Preferences    │
│(Personalization)│
├─────────────────┤
│ preference_id PK│
│ user_id (FK)    │
│ personalization │ (ENUM: basic/detailed/expert)
│ auto_personalize│ (BOOLEAN)
│ created_at      │
└─────────────────┘

┌─────────────────┐
│ TranslationLogs │
│   (Analytics)   │
├─────────────────┤
│ log_id (PK)     │
│ user_id (FK)    │──────┐
│ chapter_id      │      │
│ language        │      │ N:1
│ timestamp       │      │
└─────────────────┘      │
                         │
                         └──────► Users (user_id)
```

**Relationships**:
- User → Profile: 1:1 (one profile per user)
- User → Preferences: 1:1 (one preference set per user)
- User → TranslationLogs: 1:N (many log entries per user)

---

## Table Schemas

### 1. Users Table (Authentication)

```sql
CREATE TABLE users (
    user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    is_active BOOLEAN DEFAULT TRUE,
    is_superuser BOOLEAN DEFAULT FALSE,
    is_verified BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP,

    -- Constraints
    CONSTRAINT email_format CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$'),
    CONSTRAINT password_length CHECK (length(hashed_password) >= 60)  -- bcrypt hash length
);

-- Indexes
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_active ON users(is_active) WHERE is_active = TRUE;
```

**Field Validation Rules**:
- `email`: Must be valid email format (RFC 5322 simplified), unique
- `hashed_password`: Minimum 60 characters (bcrypt hash), never store plaintext
- `is_active`: Soft delete mechanism (inactive users can't log in)
- `is_verified`: Email verification flag (optional Phase 2)
- `created_at`: Automatically set on insert
- `last_login`: Updated on successful signin

**State Transitions**:
1. **New User**: `is_active=TRUE`, `is_verified=FALSE`
2. **Email Verified** (Phase 2): `is_verified=TRUE`
3. **Account Deactivated**: `is_active=FALSE` (soft delete)

---

### 2. Profiles Table (User Background)

```sql
-- ENUM for hardware access (mutually exclusive per FR-010)
CREATE TYPE hardware_type AS ENUM ('GPU', 'Jetson', 'Robot', 'CloudOnly');

CREATE TABLE profiles (
    profile_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    software_skills JSONB DEFAULT '[]'::jsonb,
    hardware_access hardware_type,
    learning_goal TEXT,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT unique_user_profile UNIQUE (user_id),
    CONSTRAINT learning_goal_length CHECK (length(learning_goal) <= 200),
    CONSTRAINT software_skills_array CHECK (jsonb_typeof(software_skills) = 'array')
);

-- Indexes
CREATE INDEX idx_profiles_user_id ON profiles(user_id);
CREATE INDEX idx_profiles_software_skills ON profiles USING GIN (software_skills);
CREATE INDEX idx_profiles_hardware_access ON profiles(hardware_access);
```

**Field Validation Rules**:
- `software_skills`: JSONB array of strings from predefined set (FR-009):
  - Valid values: `["Python", "ML", "ROS", "Linux", "C++", "JavaScript", "None", "Other"]`
  - Example: `["Python", "ML", "ROS"]`
  - Application-level validation ensures only valid skills
- `hardware_access`: ENUM enforces one of four values (FR-010):
  - `GPU`: RTX GPU available
  - `Jetson`: Jetson device available
  - `Robot`: Physical robot available
  - `CloudOnly`: Cloud-only access
- `learning_goal`: Optional free text, max 200 characters (FR-011)
- `updated_at`: Triggers cache invalidation (profile hash changes)

**State Transitions**:
1. **Profile Created**: After signup questionnaire (FR-008)
2. **Profile Updated**: User modifies in settings (FR-013)
3. **Profile Incomplete**: User skipped questionnaire (FR-012)

**Cache Invalidation Trigger**:
```sql
-- Trigger to update updated_at on any change
CREATE OR REPLACE FUNCTION update_profile_timestamp()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER trigger_update_profile_timestamp
BEFORE UPDATE ON profiles
FOR EACH ROW
EXECUTE FUNCTION update_profile_timestamp();
```

---

### 3. Preferences Table (Personalization Settings)

```sql
-- ENUM for personalization level
CREATE TYPE personalization_level_type AS ENUM ('basic', 'detailed', 'expert');

CREATE TABLE preferences (
    preference_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    personalization_level personalization_level_type DEFAULT 'basic',
    auto_personalize BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT unique_user_preference UNIQUE (user_id)
);

-- Indexes
CREATE INDEX idx_preferences_user_id ON preferences(user_id);
```

**Field Validation Rules**:
- `personalization_level`: Controls depth of personalization (FR-034)
  - `basic`: Simplified explanations, fewer examples
  - `detailed`: Standard depth (default)
  - `expert`: Advanced topics, fewer hand-holding
- `auto_personalize`: If TRUE, automatically personalize chapters on load
  - Phase 1: Always FALSE (manual button click only)
  - Phase 2: User can enable auto-personalization

**State Transitions**:
1. **Default Preferences**: Created with user account
2. **User Modified**: Updated via settings page

---

### 4. TranslationLogs Table (Analytics - Optional)

```sql
CREATE TABLE translation_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(user_id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    language VARCHAR(20) DEFAULT 'urdu',
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT chapter_id_format CHECK (chapter_id ~* '^module-[0-9]+-chapter-[0-9]+$')
);

-- Indexes
CREATE INDEX idx_translation_logs_user_id ON translation_logs(user_id);
CREATE INDEX idx_translation_logs_chapter_id ON translation_logs(chapter_id);
CREATE INDEX idx_translation_logs_timestamp ON translation_logs(timestamp);
```

**Field Validation Rules**:
- `chapter_id`: Format `module-{N}-chapter-{M}` (e.g., `module-1-chapter-2`)
- `language`: Currently only `urdu` (Phase 2: expand to other languages)
- `timestamp`: Automatically set on log entry

**Purpose**: Analytics only (FR-035)
- Track which chapters are most frequently translated
- Identify users who frequently use translation
- Monitor feature usage for ROI analysis

**Privacy Note**: No translation content is stored, only metadata.

---

## Data Access Patterns

### 1. User Signup Flow

```python
# 1. Create user account (FR-001, FR-002)
user = User(
    email="user@example.com",
    hashed_password=hash_password("SecurePass123"),
    is_active=True,
    is_verified=False
)
db.add(user)
db.commit()

# 2. Create default profile (FR-008)
profile = Profile(
    user_id=user.user_id,
    software_skills=[],  # Empty until questionnaire
    hardware_access=None,
    learning_goal=None
)
db.add(profile)

# 3. Create default preferences (FR-034)
preference = Preference(
    user_id=user.user_id,
    personalization_level="basic",
    auto_personalize=False
)
db.add(preference)
db.commit()
```

### 2. Questionnaire Submission Flow

```python
# Update profile with questionnaire answers (FR-009, FR-010, FR-011)
profile = db.query(Profile).filter(Profile.user_id == user_id).first()
profile.software_skills = ["Python", "ML", "ROS"]  # JSONB array
profile.hardware_access = "GPU"
profile.learning_goal = "Build autonomous humanoid robots"
profile.updated_at = datetime.utcnow()  # Triggers cache invalidation
db.commit()

# Profile hash will change, invalidating all cached personalizations
```

### 3. Personalization Request Flow

```python
# 1. Fetch user profile
profile = db.query(Profile).filter(Profile.user_id == user_id).first()

# 2. Generate profile hash
profile_hash = generate_profile_hash({
    "software_skills": profile.software_skills,
    "hardware_access": profile.hardware_access,
    "learning_goal": profile.learning_goal
})

# 3. Check Redis cache
cache_key = f"personalized:{user_id}:{chapter_id}:{profile_hash}"
cached_content = redis.get(cache_key)

if cached_content:
    return cached_content  # Cache hit (<100ms per FR-044)
else:
    # Cache miss: Generate personalization (FR-016, FR-017)
    personalized_content = personalization_skill(chapter_markdown, profile)
    redis.setex(cache_key, 1800, personalized_content)  # 30-min TTL
    return personalized_content
```

### 4. Translation Request Flow

```python
# 1. Verify user is authenticated (FR-023)
if not is_authenticated(request):
    raise HTTPException(status_code=401, detail="Authentication required")

# 2. Translate chapter (FR-024, FR-025, FR-026)
translated_content = translation_skill(chapter_markdown)

# 3. Log translation request (FR-035 - optional analytics)
log_entry = TranslationLog(
    user_id=user_id,
    chapter_id=chapter_id,
    language="urdu",
    timestamp=datetime.utcnow()
)
db.add(log_entry)
db.commit()

# 4. Return translated content (NO CACHING per FR-029)
return translated_content
```

### 5. Profile Update Flow

```python
# User updates profile in settings (FR-013)
profile = db.query(Profile).filter(Profile.user_id == user_id).first()
profile.software_skills = updated_skills
profile.hardware_access = updated_hardware
profile.updated_at = datetime.utcnow()
db.commit()

# Invalidate all cached personalizations for this user (FR-047)
cache_pattern = f"personalized:{user_id}:*"
for key in redis.scan_iter(match=cache_pattern):
    redis.delete(key)
```

---

## Migration Scripts

### Initial Migration (Alembic)

```python
"""Create auth and profile tables

Revision ID: 001_create_tables
Revises:
Create Date: 2025-12-16
"""

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID, JSONB, ENUM

def upgrade():
    # Create ENUMs
    hardware_type = ENUM('GPU', 'Jetson', 'Robot', 'CloudOnly', name='hardware_type')
    personalization_level_type = ENUM('basic', 'detailed', 'expert', name='personalization_level_type')

    hardware_type.create(op.get_bind())
    personalization_level_type.create(op.get_bind())

    # Create users table
    op.create_table(
        'users',
        sa.Column('user_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        sa.Column('hashed_password', sa.String(255), nullable=False),
        sa.Column('is_active', sa.Boolean(), default=True),
        sa.Column('is_superuser', sa.Boolean(), default=False),
        sa.Column('is_verified', sa.Boolean(), default=False),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.text('CURRENT_TIMESTAMP')),
        sa.Column('last_login', sa.TIMESTAMP(), nullable=True)
    )
    op.create_index('idx_users_email', 'users', ['email'])

    # Create profiles table
    op.create_table(
        'profiles',
        sa.Column('profile_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', UUID(as_uuid=True), sa.ForeignKey('users.user_id', ondelete='CASCADE'), nullable=False),
        sa.Column('software_skills', JSONB, server_default="'[]'::jsonb"),
        sa.Column('hardware_access', hardware_type, nullable=True),
        sa.Column('learning_goal', sa.TEXT(), nullable=True),
        sa.Column('updated_at', sa.TIMESTAMP(), server_default=sa.text('CURRENT_TIMESTAMP'))
    )
    op.create_index('idx_profiles_user_id', 'profiles', ['user_id'])
    op.create_index('idx_profiles_software_skills', 'profiles', ['software_skills'], postgresql_using='gin')

    # Create preferences table
    op.create_table(
        'preferences',
        sa.Column('preference_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', UUID(as_uuid=True), sa.ForeignKey('users.user_id', ondelete='CASCADE'), nullable=False),
        sa.Column('personalization_level', personalization_level_type, server_default='basic'),
        sa.Column('auto_personalize', sa.Boolean(), default=False),
        sa.Column('created_at', sa.TIMESTAMP(), server_default=sa.text('CURRENT_TIMESTAMP'))
    )
    op.create_index('idx_preferences_user_id', 'preferences', ['user_id'])

    # Create translation_logs table
    op.create_table(
        'translation_logs',
        sa.Column('log_id', UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', UUID(as_uuid=True), sa.ForeignKey('users.user_id', ondelete='CASCADE'), nullable=False),
        sa.Column('chapter_id', sa.String(100), nullable=False),
        sa.Column('language', sa.String(20), server_default='urdu'),
        sa.Column('timestamp', sa.TIMESTAMP(), server_default=sa.text('CURRENT_TIMESTAMP'))
    )
    op.create_index('idx_translation_logs_user_id', 'translation_logs', ['user_id'])
    op.create_index('idx_translation_logs_chapter_id', 'translation_logs', ['chapter_id'])

def downgrade():
    op.drop_table('translation_logs')
    op.drop_table('preferences')
    op.drop_table('profiles')
    op.drop_table('users')

    op.execute('DROP TYPE IF EXISTS personalization_level_type')
    op.execute('DROP TYPE IF EXISTS hardware_type')
```

---

## Testing Strategy

### 1. Unit Tests (Database Layer)

```python
def test_user_creation():
    """Test user can be created with valid email and password."""
    user = User(email="test@example.com", hashed_password="$2b$12$...")
    assert user.email == "test@example.com"
    assert user.is_active == True

def test_profile_jsonb_validation():
    """Test profile software_skills is valid JSONB array."""
    profile = Profile(user_id=user_id, software_skills=["Python", "ML"])
    assert isinstance(profile.software_skills, list)
    assert "Python" in profile.software_skills

def test_hardware_enum_constraint():
    """Test hardware_access only accepts valid ENUM values."""
    with pytest.raises(ValueError):
        Profile(user_id=user_id, hardware_access="InvalidValue")

def test_profile_cache_invalidation():
    """Test updated_at triggers profile hash change."""
    original_hash = generate_profile_hash(profile)
    profile.software_skills = ["Python", "ML", "ROS"]
    new_hash = generate_profile_hash(profile)
    assert original_hash != new_hash
```

### 2. Integration Tests (Database + Redis)

```python
def test_personalization_cache_flow():
    """Test personalization caching with profile hash."""
    # First request: Cache miss
    content1 = personalize_chapter(user_id, chapter_id)
    assert redis.exists(cache_key)

    # Second request: Cache hit
    content2 = personalize_chapter(user_id, chapter_id)
    assert content1 == content2

    # Profile update: Cache invalidated
    update_profile(user_id, new_skills=["Python", "ML", "ROS"])
    assert not redis.exists(cache_key)
```

---

## Summary

**Tables Created**: 4 (users, profiles, preferences, translation_logs)
**ENUMs Created**: 2 (hardware_type, personalization_level_type)
**Indexes Created**: 9 (optimized for common queries)
**Constraints**: Foreign keys, CHECK constraints, UNIQUE constraints
**State Management**: Trigger for updated_at, cache invalidation logic

**Next Phase**: Generate API contracts (OpenAPI schemas)
