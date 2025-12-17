# Deployment Guide

Complete guide for deploying the Physical AI & Humanoid Robotics Textbook application.

## Architecture Overview

- **Frontend**: Docusaurus static site (GitHub Pages)
- **Backend**: Python FastAPI server (Render/Railway/Fly.io)
- **Database**: Neon Serverless Postgres
- **Vector DB**: Qdrant Cloud
- **LLM**: OpenRouter API

---

## Part 1: Frontend Deployment (GitHub Pages)

### Status: ✅ Already Configured

Your frontend is automatically deployed via GitHub Actions when you push to `main`.

**Live URL**: https://mirza-usman-247.github.io/physical-ai-and-humanoid-robotics-textbook/

### How it works:
1. Push code to `main` branch
2. GitHub Actions workflow (`.github/workflows/deploy.yml`) triggers
3. Builds the Docusaurus site
4. Deploys to GitHub Pages

### Monitor deployments:
https://github.com/Mirza-Usman-247/physical-ai-and-humanoid-robotics-textbook/actions

---

## Part 2: Backend Deployment Options

GitHub Pages cannot host dynamic backends. Choose one of these platforms:

### Option A: Render (Recommended - Free Tier Available)

#### Step 1: Create Render Account
1. Go to https://render.com
2. Sign up with your GitHub account
3. Authorize Render to access your repository

#### Step 2: Create Web Service
1. Click "New +" → "Web Service"
2. Connect your GitHub repository
3. Configure:
   - **Name**: `physical-ai-backend`
   - **Region**: Choose closest to your users
   - **Branch**: `main`
   - **Root Directory**: `backend`
   - **Runtime**: `Python 3`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`

#### Step 3: Add Environment Variables
In Render dashboard, add these environment variables:

```env
# Database
NEON_DATABASE_URL=postgresql://neondb_owner:npg_ky1iOeAxoF5p@ep-wild-king-abtiu3h3-pooler.eu-west-2.aws.neon.tech/neondb?sslmode=require

# Qdrant
QDRANT_URL=https://641e571e-d0f4-4a44-bb54-bac35c63f936.eu-west-2-0.aws.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.fIhA-9lseR6loxKpMXGpWwDXInP9NoPvCDEcSnfQbU8
QDRANT_COLLECTION=physical-ai-textbook

# OpenRouter
OPENROUTER_API_KEY=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
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
QWEN_EMBEDDING_MODEL=text-embedding-3-small
OPENROUTER_LLM_MODEL=deepseek/deepseek-chat
OPENROUTER_LLM_MODEL_PERSONALIZE=google/gemini-2.0-flash-exp:free
OPENROUTER_LLM_MODEL_TRANSLATE=deepseek/deepseek-chat

# Application
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO

# Security
JWT_SECRET_KEY=fBLAyVckwAobaRsAwiBsdhXUs8Bo9B/1l2XV8xA9JhNti97oIKXkLlvbgMARQsz2
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_DAYS=7
SECRET_KEY=Or5WB03EK0_moWhq0X-31PydaYZyOVR4f5JfwC-A4aR0iatFhSwNqAkMGkSLJ5Jb1rjJj4xNLWTusmvTGL_W9w
RATE_LIMIT_PER_MINUTE=20

# Token Limits
MAX_INPUT_TOKENS=8000
MAX_OUTPUT_TOKENS=2000

# Retrieval
SIMILARITY_THRESHOLD=0.3
TOP_K_CHUNKS=5
CONVERSATION_CONTEXT_WINDOW=5
```

#### Step 4: Deploy
- Click "Create Web Service"
- Render will automatically build and deploy
- Note your backend URL: `https://physical-ai-backend.onrender.com`

---

### Option B: Railway (Alternative)

#### Step 1: Create Railway Account
1. Go to https://railway.app
2. Sign up with GitHub
3. Create new project

#### Step 2: Deploy from GitHub
1. Click "Deploy from GitHub repo"
2. Select your repository
3. Railway auto-detects Python
4. Set root directory: `backend`

#### Step 3: Configure
1. Add environment variables (same as Render)
2. Set start command: `uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`

#### Step 4: Deploy
- Railway deploys automatically
- Note your backend URL: `https://your-app.up.railway.app`

---

### Option C: Fly.io (For Advanced Users)

#### Prerequisites
```bash
# Install Fly CLI
# Windows (PowerShell)
iwr https://fly.io/install.ps1 -useb | iex

# Mac/Linux
curl -L https://fly.io/install.sh | sh
```

#### Step 1: Login and Initialize
```bash
cd backend
fly auth login
fly launch
```

#### Step 2: Configure `fly.toml`
Fly will create `fly.toml`. Update it:

```toml
app = "physical-ai-backend"

[build]
  builder = "paketobuildpacks/builder:base"

[env]
  PORT = "8080"

[[services]]
  http_checks = []
  internal_port = 8080
  processes = ["app"]
  protocol = "tcp"

  [[services.ports]]
    force_https = true
    handlers = ["http"]
    port = 80

  [[services.ports]]
    handlers = ["tls", "http"]
    port = 443

[http_service]
  internal_port = 8080
  force_https = true
  auto_stop_machines = true
  auto_start_machines = true
```

#### Step 3: Set Secrets
```bash
fly secrets set NEON_DATABASE_URL="your-database-url"
fly secrets set QDRANT_URL="your-qdrant-url"
fly secrets set QDRANT_API_KEY="your-qdrant-key"
# ... set all other env vars
```

#### Step 4: Deploy
```bash
fly deploy
```

---

## Part 3: Connect Frontend to Backend

### Step 1: Update Backend CORS

Edit `backend/src/api/main.py`:

```python
# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "http://localhost:8000",
        "https://mirza-usman-247.github.io",  # GitHub Pages
        "https://*.render.com",  # If using Render
        "https://*.railway.app",  # If using Railway
        "https://*.fly.dev",  # If using Fly.io
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"]
)
```

### Step 2: Update Frontend Environment Variables

Update `.env`:

```env
# Replace with your actual backend URL
BETTER_AUTH_URL=https://physical-ai-backend.onrender.com/api/auth
NEXT_PUBLIC_API_URL=https://physical-ai-backend.onrender.com
BETTER_AUTH_SECRET=1780a277882dc9520e28bb252471648e13f385e896161fb4f29c9b322777c115
```

### Step 3: Rebuild and Deploy

```bash
# Commit backend CORS changes
git add backend/src/api/main.py
git commit -m "Update CORS for production deployment"

# Commit frontend env changes (if not in .gitignore)
git add .env
git commit -m "Update API URLs for production"

# Push to trigger deployments
git push origin main
```

---

## Part 4: Verify Deployment

### Test Backend
```bash
# Check health endpoint
curl https://your-backend-url.onrender.com/health

# Expected response:
# {"status": "healthy"}
```

### Test Frontend
1. Visit: https://mirza-usman-247.github.io/physical-ai-and-humanoid-robotics-textbook/
2. Open browser console (F12)
3. Check for API connection errors
4. Test authentication features

---

## Part 5: GitHub Actions Setup (Optional CI/CD)

### Backend CI/CD Workflow

Create `.github/workflows/backend-deploy.yml`:

```yaml
name: Deploy Backend

on:
  push:
    branches: [main]
    paths:
      - 'backend/**'
  workflow_dispatch:

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Deploy to Render
        run: |
          curl -X POST "https://api.render.com/deploy/srv-YOUR-SERVICE-ID?key=${{ secrets.RENDER_DEPLOY_HOOK }}"
```

**Setup:**
1. Go to Render Dashboard → Your Service → Settings
2. Copy the Deploy Hook URL
3. Add to GitHub Secrets as `RENDER_DEPLOY_HOOK`

---

## Part 6: Monitoring and Maintenance

### Backend Monitoring
- **Render**: View logs in dashboard
- **Railway**: Real-time logs in web UI
- **Fly.io**: `fly logs`

### Frontend Monitoring
- Check GitHub Actions for build status
- Monitor GitHub Pages deployment

### Common Issues

#### Issue: CORS Errors
**Solution**: Verify backend CORS includes your frontend URL

#### Issue: API Connection Failed
**Solution**:
1. Check backend is running: `curl https://your-backend-url/health`
2. Verify `.env` has correct API URL
3. Check browser console for specific errors

#### Issue: Authentication Not Working
**Solution**:
1. Verify `BETTER_AUTH_SECRET` matches between frontend and backend
2. Check JWT tokens are being sent in requests
3. Verify database connection

---

## Part 7: Environment-Specific Configurations

### Development
```env
# .env.local
BETTER_AUTH_URL=http://localhost:8000/api/auth
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### Production (GitHub Pages)
```env
# .env
BETTER_AUTH_URL=https://physical-ai-backend.onrender.com/api/auth
NEXT_PUBLIC_API_URL=https://physical-ai-backend.onrender.com
```

---

## Quick Reference

| Service | URL | Status |
|---------|-----|--------|
| Frontend | https://mirza-usman-247.github.io/physical-ai-and-humanoid-robotics-textbook/ | ✅ Deployed |
| Backend | (Choose deployment platform) | ⏳ Pending |
| Database | Neon Postgres | ✅ Configured |
| Vector DB | Qdrant Cloud | ✅ Configured |

---

## Next Steps

1. ✅ Frontend is deployed to GitHub Pages
2. ⏳ Deploy backend to Render/Railway/Fly.io
3. ⏳ Update CORS in backend
4. ⏳ Update API URLs in frontend
5. ⏳ Test full application flow
6. ⏳ Set up monitoring

---

## Support

For issues:
- Check deployment logs
- Verify environment variables
- Test backend health endpoint
- Check browser console for frontend errors

**Need help?** Review the logs from your chosen deployment platform for specific error messages.
