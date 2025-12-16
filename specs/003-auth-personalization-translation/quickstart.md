# Quickstart: Authentication, Personalization, and Translation

**Feature**: 003-auth-personalization-translation
**Date**: 2025-12-17
**Status**: Setup Guide

## Overview

This guide provides step-by-step instructions for setting up the authentication, personalization, and translation feature in the development environment.

---

## Prerequisites

### Required Software

- **Node.js**: v18+ (for Docusaurus frontend)
- **Python**: 3.11+ (for FastAPI backend)
- **PostgreSQL**: N/A (using Neon Serverless Postgres cloud instance)
- **Git**: Latest version

### Required Accounts

1. **Neon Serverless Postgres**: Sign up at https://neon.tech/
2. **OpenRouter API**: Sign up at https://openrouter.ai/ (free tier)

---

## Setup Instructions

### 1. Clone Repository

```bash
git clone https://github.com/your-org/Humanoid-physical-ai-textbook.git
cd Humanoid-physical-ai-textbook
git checkout 003-auth-personalization-translation
```

### 2. Backend Setup (FastAPI)

#### 2.1 Install Python Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

**Add to `requirements.txt`**:
```
# Existing dependencies
fastapi
uvicorn[standard]
sqlalchemy>=2.0.0
alembic
psycopg2-binary
pydantic[email]
python-dotenv

# NEW: Auth dependencies
passlib[bcrypt]
python-jose[cryptography]
python-multipart
```

#### 2.2 Configure Environment Variables

Create `backend/.env`:

```bash
# Database (Neon Serverless Postgres)
NEON_DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/dbname

# JWT Configuration
JWT_SECRET_KEY=<generate-with-openssl-rand-base64-32>
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7

# OpenRouter API
OPENROUTER_API_KEY=<your-openrouter-api-key>

# Existing RAG configuration
QDRANT_URL=<your-qdrant-url>
QDRANT_API_KEY=<your-qdrant-api-key>
QDRANT_COLLECTION=physical-ai-textbook

# Application
ENVIRONMENT=development
DEBUG=true
LOG_LEVEL=DEBUG
```

**Generate JWT Secret Key**:
```bash
openssl rand -base64 32
```

#### 2.3 Run Database Migrations

```bash
# Create initial migration
alembic revision --autogenerate -m "Add user authentication and profile tables"

# Apply migration
alembic upgrade head
```

#### 2.4 Start Backend Server

```bash
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

**Verify Backend**:
- Open http://localhost:8000/docs (Swagger UI)
- Endpoints should include: `/api/auth/signup`, `/api/auth/signin`, `/api/personalize`, `/api/translate`

---

### 3. Frontend Setup (Docusaurus)

#### 3.1 Install Node Dependencies

```bash
cd ..  # Back to project root
npm install
```

**Add to `package.json` dependencies**:
```json
{
  "dependencies": {
    "@docusaurus/core": "3.x",
    "@docusaurus/preset-classic": "3.x",
    "react": "^18.0.0",
    "react-dom": "^18.0.0",

    // NEW: Auth and state management
    "better-auth": "^latest",
    "zod": "^3.22.0",
    "idb": "^8.0.0"
  }
}
```

#### 3.2 Configure Better Auth

Create `src/lib/auth.ts`:

```typescript
import { createAuth } from 'better-auth';

export const auth = createAuth({
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3000',
  secret: process.env.BETTER_AUTH_SECRET,
  database: {
    type: 'postgres',
    url: process.env.NEON_DATABASE_URL,
  },
  emailAndPassword: {
    enabled: true,
  },
});
```

Create `.env.local` (frontend):

```bash
BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=<same-as-jwt-secret-or-generate-new>
NEON_DATABASE_URL=<same-as-backend>
```

#### 3.3 Start Docusaurus

```bash
npm start
```

**Verify Frontend**:
- Open http://localhost:3000
- Should see existing Docusaurus site

---

## Development Workflow

### 1. Create a New User (Test)

**Using Swagger UI** (http://localhost:8000/docs):

1. Navigate to `/api/auth/signup`
2. Click "Try it out"
3. Use example payload:
   ```json
   {
     "email": "test@example.com",
     "password": "TestPass123",
     "skillLevels": {
       "ai": 3,
       "ml": 3,
       "ros": 2,
       "python": 4,
       "linux": 3
     },
     "hardwareAccess": {
       "gpu": true,
       "jetson": false,
       "robot": false
     }
   }
   ```
4. Click "Execute"
5. Copy `accessToken` from response

### 2. Test Personalization API

**Using Swagger UI**:

1. Click "Authorize" button (top right)
2. Enter: `Bearer <your-access-token>`
3. Navigate to `/api/personalize`
4. Try it out with:
   ```json
   {
     "chapterId": "module-1-chapter-1",
     "chapterContent": "# Chapter 1: Introduction to Physical AI\n\nPhysical AI combines...",
     "userProfile": {
       "userId": "<from-signup-response>",
       "skillLevels": {
         "ai": 3,
         "ml": 3,
         "ros": 2,
         "python": 4,
         "linux": 3
       },
       "hardwareAccess": {
         "gpu": true,
         "jetson": false,
         "robot": false,
         "cloudOnly": false
       }
     }
   }
   ```

### 3. Test Translation API

Same as personalization, but use `/api/translate` endpoint with:
```json
{
  "chapterId": "module-1-chapter-1",
  "chapterContent": "# Chapter 1: Introduction\n...",
  "targetLanguage": "ur",
  "mode": "focus"
}
```

---

## Troubleshooting

### Database Connection Issues

**Error**: `could not connect to server`

**Solution**:
1. Verify Neon database URL in `.env`
2. Check Neon dashboard for connection string
3. Ensure IP is whitelisted in Neon project settings

### JWT Token Errors

**Error**: `Invalid token` or `Token expired`

**Solution**:
1. Verify `JWT_SECRET_KEY` matches between backend `.env` and frontend `.env.local`
2. Check token expiry settings (15 minutes for access token)
3. Use `/api/auth/refresh` to get new access token

### OpenRouter API Errors

**Error**: `Rate limit exceeded`

**Solution**:
1. Free tier limit: 20 requests/minute
2. Wait 60 seconds before retrying
3. Consider implementing request queuing (see research.md section 3)

### Better Auth Setup Issues

**Error**: `Better Auth configuration error`

**Solution**:
1. Ensure `BETTER_AUTH_SECRET` is set in `.env.local`
2. Verify database connection (Better Auth uses same Neon DB)
3. Check Better Auth docs: https://www.better-auth.com/docs

---

## Testing Checklist

Before proceeding to implementation:

- [ ] Backend server starts without errors
- [ ] Swagger UI accessible at http://localhost:8000/docs
- [ ] Database migration successful (`alembic upgrade head`)
- [ ] Can create user via `/api/auth/signup`
- [ ] Can sign in via `/api/auth/signin` and receive JWT tokens
- [ ] JWT token validates on protected endpoints
- [ ] Personalization API returns transformed content
- [ ] Translation API returns Urdu content
- [ ] Frontend Docusaurus site loads at http://localhost:3000

---

## Next Steps

After completing this quickstart:

1. **Read Planning Documents**:
   - `plan.md` - Full implementation plan
   - `research.md` - Technology research and decisions
   - `data-model.md` - Database schema and data flows

2. **Review API Contracts**:
   - `contracts/auth.openapi.yaml` - Authentication API
   - `contracts/personalization.openapi.yaml` - Personalization API
   - `contracts/translation.openapi.yaml` - Translation API
   - `contracts/profile.openapi.yaml` - Profile API

3. **Run `/sp.tasks`**: Generate granular task breakdown for implementation

4. **Implement Tasks**: Follow TDD approach (Red → Green → Refactor)

---

## Support

- **Issues**: Create GitHub issue with `auth-personalization-translation` label
- **Questions**: Ask in team Slack channel `#physical-ai-dev`
- **Documentation**: See `/specs/003-auth-personalization-translation/` directory

**Status**: ✅ Setup guide complete
