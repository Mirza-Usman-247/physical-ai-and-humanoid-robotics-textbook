# Vercel Deployment Guide

This guide walks you through deploying both the **frontend (Docusaurus)** and **backend (FastAPI)** to Vercel.

## Overview

- **Frontend**: Docusaurus static site deployed from the root directory
- **Backend**: FastAPI serverless API deployed from the `backend/` directory

## Prerequisites

1. [Vercel Account](https://vercel.com/signup)
2. [Vercel CLI](https://vercel.com/docs/cli) (optional, for local testing)
3. All required API keys and database credentials

---

## Part 1: Deploy Backend API

### Step 1: Create New Vercel Project for Backend

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Click **"Add New..."** → **"Project"**
3. Import your repository
4. Configure the project:
   - **Project Name**: `physical-ai-backend` (or your choice)
   - **Framework Preset**: Other
   - **Root Directory**: `backend`
   - **Build Command**: Leave empty
   - **Output Directory**: Leave empty

### Step 2: Configure Backend Environment Variables

Go to **Project Settings** → **Environment Variables** and add the following:

#### Required Variables (MUST be set)

```bash
# Database Configuration
NEON_DATABASE_URL=postgresql://user:password@your-neon-host.neon.tech/dbname?sslmode=require

# Vector Database (Qdrant)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION=physical-ai-textbook

# OpenRouter API
OPENROUTER_API_KEY=sk-or-v1-your-openrouter-api-key-here
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1

# Security Keys (IMPORTANT: Generate strong random keys!)
JWT_SECRET_KEY=your-jwt-secret-minimum-32-characters-CHANGE-THIS
SECRET_KEY=your-app-secret-minimum-32-characters-CHANGE-THIS
```

#### Application Settings (Pre-configured, can be customized)

```bash
# Environment
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO

# AI Models
QWEN_EMBEDDING_MODEL=text-embedding-3-small
OPENROUTER_LLM_MODEL=anthropic/claude-3.5-sonnet
OPENROUTER_LLM_MODEL_PERSONALIZE=google/gemini-2.0-flash-exp:free
OPENROUTER_LLM_MODEL_TRANSLATE=google/gemini-2.0-flash-exp:free

# Security & Rate Limiting
RATE_LIMIT_PER_MINUTE=20

# JWT Configuration
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7

# Token Limits
MAX_INPUT_TOKENS=8000
MAX_OUTPUT_TOKENS=2000

# Retrieval Settings
SIMILARITY_THRESHOLD=0.7
TOP_K_CHUNKS=5
CONVERSATION_CONTEXT_WINDOW=5
```

### Step 3: Deploy Backend

1. Click **"Deploy"**
2. Wait for deployment to complete
3. Note your backend URL: `https://your-backend.vercel.app`

### Step 4: Test Backend API

```bash
# Health check
curl https://your-backend.vercel.app/health

# Expected response:
# {"status": "healthy", "timestamp": "..."}
```

---

## Part 2: Deploy Frontend (Docusaurus)

### Step 1: Create New Vercel Project for Frontend

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Click **"Add New..."** → **"Project"**
3. Import the **same repository** (or create a separate one)
4. Configure the project:
   - **Project Name**: `physical-ai-textbook` (or your choice)
   - **Framework Preset**: Other (or Docusaurus if available)
   - **Root Directory**: `.` (root)
   - **Build Command**: `docusaurus build --config docusaurus.config.vercel.js`
   - **Output Directory**: `build`

### Step 2: Configure Frontend Environment Variables

Go to **Project Settings** → **Environment Variables** and add:

```bash
# Backend API URL (use your deployed backend URL from Part 1)
NEXT_PUBLIC_API_URL=https://your-backend.vercel.app

# Better Auth Configuration (optional, if using Better Auth)
BETTER_AUTH_URL=https://your-backend.vercel.app/api/auth
BETTER_AUTH_SECRET=your-better-auth-secret-minimum-32-characters-CHANGE-THIS
```

### Step 3: Update CORS in Backend

After deploying frontend, you need to update the backend's CORS settings to allow requests from your frontend URL.

1. Go to your backend Vercel project
2. Add environment variable:
   ```bash
   FRONTEND_URL=https://your-frontend.vercel.app
   ```

3. Update `backend/src/api/main.py` (lines 73-86) to include your frontend URL:
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=[
           "https://your-frontend.vercel.app",  # Add your frontend URL
           "http://localhost:3000",  # For local development
       ],
       allow_credentials=True,
       allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
       allow_headers=["*"],
   )
   ```

4. Redeploy the backend

### Step 4: Deploy Frontend

1. Click **"Deploy"**
2. Wait for deployment to complete
3. Your site will be live at: `https://your-frontend.vercel.app`

---

## Part 3: Database Setup

### Neon Postgres Setup

1. Go to [Neon Console](https://console.neon.tech/)
2. Create a new project
3. Copy the connection string
4. Add it to backend environment variables as `NEON_DATABASE_URL`

### Run Database Migrations

After deploying the backend, you need to run migrations:

```bash
# Install Vercel CLI if not already installed
npm i -g vercel

# Link to your backend project
cd backend
vercel link

# Run migration command
vercel env pull .env.production
python -m alembic upgrade head
```

**Alternative**: Set up a GitHub Action to run migrations automatically on deploy.

### Qdrant Vector Database Setup

1. Go to [Qdrant Cloud](https://cloud.qdrant.io/)
2. Create a new cluster
3. Create a collection named `physical-ai-textbook`
4. Copy the cluster URL and API key
5. Add them to backend environment variables

---

## Part 4: Connect Frontend to Backend

Update your frontend code to use the production backend URL:

1. Create or update `src/utils/api.js`:
   ```javascript
   const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

   export async function chatWithAI(message) {
     const response = await fetch(`${API_URL}/api/chat`, {
       method: 'POST',
       headers: {
         'Content-Type': 'application/json',
       },
       body: JSON.stringify({ message }),
       credentials: 'include',
     });
     return response.json();
   }
   ```

---

## Security Checklist

- [ ] Generated strong random keys for `JWT_SECRET_KEY` and `SECRET_KEY`
- [ ] Set `DEBUG=false` in production
- [ ] Configured proper CORS origins (no wildcards in production)
- [ ] Database connection uses SSL (`?sslmode=require`)
- [ ] API keys are stored as environment variables (not in code)
- [ ] Rate limiting is enabled
- [ ] HTTPS is enforced (Vercel does this by default)

---

## How to Generate Secret Keys

Use these commands to generate secure random keys:

```bash
# Method 1: Using Python
python -c "import secrets; print(secrets.token_urlsafe(32))"

# Method 2: Using OpenSSL
openssl rand -base64 32

# Method 3: Using Node.js
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"
```

---

## Troubleshooting

### Backend Issues

**Problem**: `500 Internal Server Error`
- Check Vercel Function Logs in dashboard
- Verify all environment variables are set
- Check database connectivity

**Problem**: `CORS Error`
- Verify frontend URL is in backend's CORS allow_origins
- Check credentials setting in frontend fetch requests

**Problem**: `Database connection failed`
- Verify `NEON_DATABASE_URL` is correct
- Ensure `?sslmode=require` is in the connection string
- Check if migrations have been run

### Frontend Issues

**Problem**: `Failed to fetch from backend`
- Verify `NEXT_PUBLIC_API_URL` is set correctly
- Check backend is deployed and healthy
- Verify CORS is configured

**Problem**: Build fails
- Check Node.js version (requires >=20.0.0)
- Clear Docusaurus cache: `docusaurus clear`
- Check for TypeScript errors

---

## Monitoring & Logs

### View Backend Logs
1. Go to Vercel Dashboard → Your Backend Project
2. Click on **"Logs"** tab
3. Filter by function invocations

### View Frontend Logs
1. Go to Vercel Dashboard → Your Frontend Project
2. Click on **"Logs"** tab
3. Check build and runtime logs

---

## Environment Variables Quick Reference

### Backend Required Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `NEON_DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@host/db?sslmode=require` |
| `QDRANT_URL` | Qdrant cluster URL | `https://abc.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `your-api-key` |
| `QDRANT_COLLECTION` | Collection name | `physical-ai-textbook` |
| `OPENROUTER_API_KEY` | OpenRouter API key | `sk-or-v1-...` |
| `JWT_SECRET_KEY` | JWT signing key (32+ chars) | Generated random string |
| `SECRET_KEY` | App secret key (32+ chars) | Generated random string |

### Frontend Required Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `NEXT_PUBLIC_API_URL` | Backend API URL | `https://your-backend.vercel.app` |

---

## Next Steps

1. ✅ Set up custom domain (optional)
2. ✅ Enable analytics in Vercel
3. ✅ Set up monitoring with Sentry or similar
4. ✅ Configure automatic deployments from Git
5. ✅ Set up preview deployments for PRs

---

## Support & Resources

- [Vercel Documentation](https://vercel.com/docs)
- [FastAPI on Vercel](https://vercel.com/guides/using-fastapi-with-vercel)
- [Docusaurus Deployment](https://docusaurus.io/docs/deployment)
- [Neon Documentation](https://neon.tech/docs)
- [Qdrant Documentation](https://qdrant.tech/documentation/)

---

## Cost Estimates

**Vercel Free Tier Includes:**
- 100 GB bandwidth/month
- Serverless function executions
- Automatic HTTPS
- Preview deployments

**Note**: Monitor your usage as AI API calls (OpenRouter) and database costs (Neon, Qdrant) are separate.
