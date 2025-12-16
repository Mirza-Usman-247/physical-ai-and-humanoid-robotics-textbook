# Quickstart Guide: Auth, Personalization & Translation

**Feature**: Auth, Personalization & Translation Integration
**Date**: 2025-12-16
**Audience**: Developers setting up local environment

---

## Prerequisites

Before starting, ensure you have:

- **Python 3.11+** (check: `python --version`)
- **Node.js 18+** and npm (check: `node --version`)
- **Redis 7.x** running locally or remotely (check: `redis-cli ping` returns `PONG`)
- **Neon Postgres** credentials (free tier: [neon.tech](https://neon.tech))
- **OpenRouter API key** (free tier: [openrouter.ai](https://openrouter.ai))
- **Git** for version control

---

## Step 1: Clone Repository and Checkout Feature Branch

```bash
# Clone repository
git clone https://github.com/your-org/Humanoid-physical-ai-textbook.git
cd Humanoid-physical-ai-textbook

# Checkout feature branch
git checkout 003-auth-personalization-translation

# Verify you're on the correct branch
git branch --show-current
# Expected output: 003-auth-personalization-translation
```

---

## Step 2: Backend Setup (FastAPI)

### 2.1 Install Python Dependencies

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Expected packages:
# - fastapi==0.104.1
# - fastapi-users[sqlalchemy]==12.1.0
# - psycopg[binary]==3.1.12
# - redis==5.0.1
# - openai==1.3.0 (for OpenRouter)
# - alembic==1.12.1
# - uvicorn==0.24.0
```

### 2.2 Configure Environment Variables

```bash
# Copy example environment file
cp .env.example .env

# Edit .env with your credentials
nano .env  # or use your preferred editor
```

**`.env` File Contents**:

```bash
# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@ep-your-project.us-east-2.aws.neon.tech/neondb?sslmode=require

# Redis (Cache)
REDIS_HOST=localhost
REDIS_PORT=6379
REDIS_DB=0
REDIS_PASSWORD=  # Leave empty if no password

# OpenRouter (LLM)
OPENROUTER_API_KEY=sk-or-v1-your-api-key-here
OPENROUTER_MODEL=deepseek/deepseek-r1-free

# FastAPI
SECRET_KEY=your-secret-key-generate-with-openssl-rand-hex-32
SESSION_SECRET=another-secret-key-for-sessions
ALLOWED_ORIGINS=http://localhost:3000

# Environment
ENV=development
DEBUG=True
```

**Generate Secrets**:
```bash
# Generate SECRET_KEY
openssl rand -hex 32

# Generate SESSION_SECRET
openssl rand -hex 32
```

### 2.3 Run Database Migrations

```bash
# Initialize Alembic (first time only)
alembic init alembic

# Generate migration
alembic revision --autogenerate -m "Create auth and profile tables"

# Apply migration
alembic upgrade head

# Verify tables created
psql $DATABASE_URL -c "\dt"
# Expected tables: users, profiles, preferences, translation_logs, alembic_version
```

### 2.4 Start Backend Server

```bash
# Start FastAPI server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Expected output:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Started reloader process [12345] using StatReload
# INFO:     Started server process [12346]
# INFO:     Waiting for application startup.
# INFO:     Application startup complete.
```

**Verify Backend**:
```bash
# In another terminal, test health endpoint
curl http://localhost:8000/health

# Expected response:
# {"status":"ok","database":"connected","redis":"connected"}
```

---

## Step 3: Frontend Setup (Docusaurus)

### 3.1 Install Node Dependencies

```bash
# Navigate to frontend directory (from repo root)
cd frontend

# Install dependencies
npm install

# Expected packages:
# - @docusaurus/core@3.x
# - @docusaurus/preset-classic@3.x
# - react@18.x
# - react-dom@18.x
```

### 3.2 Configure Environment Variables

```bash
# Copy example environment file
cp .env.example .env.local

# Edit .env.local
nano .env.local
```

**`.env.local` File Contents**:

```bash
# Backend API
REACT_APP_API_URL=http://localhost:8000

# Environment
REACT_APP_ENV=development
```

### 3.3 Start Frontend Development Server

```bash
# Start Docusaurus dev server
npm start

# Expected output:
# [SUCCESS] Serving "website" at http://localhost:3000/
# [INFO] Docusaurus website is running at: http://localhost:3000/
```

**Verify Frontend**:
- Open browser: http://localhost:3000
- You should see the Docusaurus homepage
- Check console for errors (F12 → Console tab)

---

## Step 4: Verification Steps

### 4.1 Test Authentication Flow

1. **Signup**:
   ```bash
   curl -X POST http://localhost:8000/auth/signup \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"SecurePass123!"}'

   # Expected: 201 Created with user object
   ```

2. **Signin**:
   ```bash
   curl -X POST http://localhost:8000/auth/signin \
     -H "Content-Type: application/json" \
     -d '{"email":"test@example.com","password":"SecurePass123!"}'

   # Expected: 200 OK with session cookie
   ```

3. **Verify in UI**:
   - Navigate to http://localhost:3000
   - Click "Sign Up" button
   - Complete signup form
   - Complete questionnaire
   - Verify redirect to homepage

### 4.2 Test Profile Management

```bash
# Submit questionnaire
curl -X POST http://localhost:8000/user/questionnaire \
  -H "Content-Type: application/json" \
  -H "Cookie: auth_session=YOUR_SESSION_COOKIE" \
  -d '{
    "software_skills": ["Python", "ML", "ROS"],
    "hardware_access": "GPU",
    "learning_goal": "Build autonomous robots"
  }'

# Get profile
curl http://localhost:8000/user/profile \
  -H "Cookie: auth_session=YOUR_SESSION_COOKIE"

# Expected: Profile with software_skills, hardware_access, learning_goal
```

### 4.3 Test Personalization

```bash
# Personalize chapter
curl -X POST http://localhost:8000/content/personalize \
  -H "Content-Type: application/json" \
  -H "Cookie: auth_session=YOUR_SESSION_COOKIE" \
  -d '{
    "chapter_id": "module-1-chapter-1",
    "chapter_markdown": "# Chapter 1: Kinematics\n\nForward kinematics..."
  }'

# Expected: Personalized markdown with X-Cache-Status: MISS
# Second request: X-Cache-Status: HIT (served from Redis)
```

### 4.4 Test Translation

```bash
# Translate chapter
curl -X POST http://localhost:8000/content/translate \
  -H "Content-Type: application/json" \
  -H "Cookie: auth_session=YOUR_SESSION_COOKIE" \
  -d '{
    "chapter_id": "module-1-chapter-1",
    "chapter_markdown": "# Chapter 1: Forward Kinematics\n\nThe end-effector...",
    "target_language": "urdu"
  }'

# Expected: Urdu translation with technical terms preserved
# Check translation_quality.preservation_rate >= 0.95
```

### 4.5 Verify Redis Caching

```bash
# Connect to Redis CLI
redis-cli

# Check cached personalizations
127.0.0.1:6379> KEYS personalized:*

# Expected: personalized:{user_id}:{chapter_id}:{profile_hash}

# Check TTL
127.0.0.1:6379> TTL personalized:a1b2c3d4:module-1-chapter-1:8f3d4a21

# Expected: ~1800 seconds (30 minutes)
```

---

## Step 5: Run Tests

### 5.1 Backend Tests

```bash
# Navigate to backend directory
cd backend

# Run all tests
pytest

# Run specific test suites
pytest tests/unit/          # Unit tests
pytest tests/integration/   # Integration tests
pytest tests/contract/      # Contract validation

# Expected: All tests pass
```

### 5.2 Frontend Tests

```bash
# Navigate to frontend directory
cd frontend

# Run Jest tests
npm test

# Run E2E tests (requires backend running)
npm run test:e2e

# Expected: All tests pass
```

---

## Step 6: Build and Deploy (Optional)

### 6.1 Build Frontend

```bash
cd frontend

# Build for production
npm run build

# Serve built site locally
npm run serve

# Expected: Production build at build/ directory
```

### 6.2 Deploy to GitHub Pages

```bash
# Deploy to GitHub Pages (from frontend directory)
GIT_USER=your-github-username npm run deploy

# Expected: Site deployed to https://your-username.github.io/Humanoid-physical-ai-textbook/
```

---

## Troubleshooting

### Issue: Database Connection Error

**Error**: `psycopg.OperationalError: connection to server failed`

**Solution**:
1. Verify DATABASE_URL in .env is correct
2. Check Neon Postgres instance is active (not paused)
3. Test connection manually:
   ```bash
   psql $DATABASE_URL -c "SELECT version();"
   ```

### Issue: Redis Connection Error

**Error**: `redis.exceptions.ConnectionError: Error 111 connecting to localhost:6379`

**Solution**:
1. Start Redis server:
   ```bash
   # macOS (Homebrew)
   brew services start redis

   # Linux
   sudo systemctl start redis

   # Windows (WSL)
   redis-server
   ```
2. Verify Redis is running:
   ```bash
   redis-cli ping
   # Expected: PONG
   ```

### Issue: OpenRouter API Rate Limit

**Error**: `openai.error.RateLimitError: Rate limit exceeded`

**Solution**:
1. Wait 1 minute (DeepSeek free tier: 50 requests/minute)
2. Check API key is valid: https://openrouter.ai/keys
3. Fallback to GPT-3.5-turbo (paid):
   ```bash
   # In .env
   OPENROUTER_MODEL=openai/gpt-3.5-turbo
   ```

### Issue: Personalization Timeout

**Error**: `504 Gateway Timeout`

**Solution**:
1. Check chapter length (must be <8000 tokens)
2. Split long chapters into sections
3. Increase timeout in backend config:
   ```python
   # src/services/personalization_service.py
   timeout=30  # Increase from 10 to 30 seconds
   ```

### Issue: Frontend Cannot Connect to Backend

**Error**: `Network Error: Failed to fetch`

**Solution**:
1. Verify backend is running: `curl http://localhost:8000/health`
2. Check CORS configuration in backend:
   ```python
   # src/main.py
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://localhost:3000"],
       allow_credentials=True
   )
   ```
3. Check REACT_APP_API_URL in frontend .env.local

---

## Next Steps

1. **Implement AI Skills**: Develop personalization_skill.py and translation_skill.py
2. **Add Frontend Components**: Create PersonalizeButton, TranslateButton, ChapterWrapper
3. **Write E2E Tests**: Playwright tests for full authentication and personalization flows
4. **Deploy to Staging**: Test in staging environment before production

**Need help?** Check:
- [Feature Specification](./spec.md)
- [Implementation Plan](./plan.md)
- [Data Model](./data-model.md)
- [API Contracts](./contracts/)

---

**Last Updated**: 2025-12-16
