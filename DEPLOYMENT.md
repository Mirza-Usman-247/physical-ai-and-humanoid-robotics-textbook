# Production Deployment Guide

## Overview

This guide covers deploying the Physical AI Textbook with Authentication, Personalization, and Translation features.

## Pre-Deployment Checklist

### 1. Environment Configuration ✅

**Backend (.env)**:
```bash
# Database (Neon Serverless PostgreSQL)
DATABASE_URL=postgresql://user:password@host/database

# JWT Configuration
JWT_SECRET_KEY=<generate-secure-random-string-min-32-chars>
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRE_MINUTES=10080  # 7 days

# OpenRouter API (FREE models)
OPENROUTER_API_KEY=<your-openrouter-api-key>
OPENROUTER_LLM_MODEL_PERSONALIZE=google/gemini-2.0-flash-exp:free
OPENROUTER_LLM_MODEL_TRANSLATE=google/gemini-2.0-flash-exp:free

# Environment
ENVIRONMENT=production
DEBUG=False
```

**Frontend (.env.local)**:
```bash
REACT_APP_API_URL=https://your-api-domain.com
```

### 2. Security Hardening 🔒

- [ ] **HTTPS Only**: Ensure all endpoints use HTTPS
- [ ] **CORS**: Restrict CORS to your frontend domain only
- [ ] **JWT Secret**: Use cryptographically secure random string (min 32 chars)
- [ ] **Rate Limiting**: Enabled on all endpoints
- [ ] **Input Validation**: All Pydantic models validate inputs
- [ ] **SQL Injection**: Using SQLAlchemy ORM (no raw SQL)
- [ ] **XSS**: React escapes all user input automatically

### 3. Database Setup 📊

**Run Migrations**:
```bash
cd backend
alembic upgrade head
```

**Verify Tables**:
- auth_users
- user_profiles
- conversations (if RAG enabled)

### 4. Performance Optimization ⚡

**Backend**:
- [ ] Connection pooling enabled (SQLAlchemy default)
- [ ] Uvicorn workers: 4-8 (based on CPU cores)
- [ ] OpenRouter rate limits: Monitor usage

**Frontend**:
- [ ] Production build: `npm run build`
- [ ] Static file serving: CDN recommended
- [ ] IndexedDB: Automatic cleanup on logout

**API Response Times** (with FREE models):
- Authentication: <1s
- Personalization: 10-20s (acceptable for free tier)
- Translation: 10-20s (acceptable for free tier)
- RAG Chatbot: 2-5s

### 5. Monitoring & Logging 📈

**Backend Logging**:
```python
# Logs automatically written to stdout
# Capture in production:
# - Auth events (signup, signin, token refresh)
# - API errors (500s, 400s)
# - Personalization/Translation requests
```

**Recommended Tools**:
- Application: Datadog, New Relic, or CloudWatch
- Database: Neon dashboard
- API Usage: OpenRouter dashboard

## Deployment Steps

### Backend (FastAPI)

**Option 1: Docker**
```bash
cd backend
docker build -t physical-ai-backend .
docker run -p 8000:8000 --env-file .env physical-ai-backend
```

**Option 2: Railway/Render/Fly.io**
```bash
# Set environment variables in dashboard
# Deploy command: uvicorn src.api.main:app --host 0.0.0.0 --port $PORT
```

### Frontend (Docusaurus)

**Build**:
```bash
npm run build
# Output: build/
```

**Option 1: Vercel/Netlify**
```bash
# Build command: npm run build
# Publish directory: build
# Environment variables: Set in dashboard
```

**Option 2: Static Hosting (S3, Cloudflare Pages)**
```bash
# Upload build/ directory
# Configure: index.html as default document
```

## Post-Deployment Validation

### 1. Smoke Tests ✅

**Authentication**:
```bash
curl -X POST https://your-api.com/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test1234","ai_level":3,...}'
# Expected: 201 Created
```

**Personalization**:
```bash
curl -X POST https://your-api.com/api/personalize \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"test","chapter_content":"# Test",...}'
# Expected: 200 OK (10-20s response time)
```

**Translation**:
```bash
curl -X POST https://your-api.com/api/translate \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"test","chapter_content":"# Test","focus_mode":true}'
# Expected: 200 OK (10-20s response time)
```

### 2. Frontend Tests 🌐

- [ ] Navigate to https://your-domain.com
- [ ] Sign Up → Verify email/password validation
- [ ] Sign In → Verify JWT token stored
- [ ] Navigate to chapter → See ChapterActions component
- [ ] Click "Personalize" → Wait 10-20s → See personalized content
- [ ] Click "Translate to Urdu" → Wait 10-20s → See RTL Urdu text
- [ ] Verify one-transformation-at-a-time constraint
- [ ] Logout → Verify session cleared

## Scaling Considerations

### Current Capacity (FREE tier)
- **Database**: Neon Free tier (10GB, sufficient for 10K+ users)
- **OpenRouter**: Free models (rate limited but no cost)
- **Concurrent Users**: ~100 (bottleneck: free LLM rate limits)

### Scaling Path
1. **100-1,000 users**: Current setup sufficient
2. **1,000-10,000 users**:
   - Upgrade to paid OpenRouter models (faster response)
   - Add Redis for session caching
   - Use CDN for frontend
3. **10,000+ users**:
   - Horizontal scaling (multiple backend instances)
   - Load balancer
   - Database read replicas

## Cost Breakdown

**Current (FREE tier)**:
- Backend hosting: $0-$5/month (Railway/Render free tier)
- Frontend hosting: $0 (Vercel/Netlify free tier)
- Database: $0 (Neon free tier)
- OpenRouter API: $0 (free models)
- **Total: $0-$5/month**

**Production (paid tier for 1K users)**:
- Backend: $7-15/month (Railway Pro)
- Frontend: $0 (Vercel free tier sufficient)
- Database: $19/month (Neon Pro for better perf)
- OpenRouter: $20-50/month (paid models, faster)
- **Total: $46-84/month**

## Troubleshooting

### Issue: Personalization/Translation timeouts
**Solution**: Free models can be slow. Set timeout to 30s, inform users to wait.

### Issue: JWT token expired errors
**Solution**: Refresh token flow should handle automatically. If not, users can sign in again.

### Issue: IndexedDB quota exceeded
**Solution**: Clear old personalizations. Implement auto-cleanup for content >30 days old.

### Issue: CORS errors
**Solution**: Verify backend CORS allows your frontend domain.

## Maintenance

**Weekly**:
- [ ] Review error logs
- [ ] Check API response times
- [ ] Monitor OpenRouter usage

**Monthly**:
- [ ] Database backup verification
- [ ] Security patches (npm audit, pip list --outdated)
- [ ] Performance optimization review

## Support

For issues or questions:
- Backend API docs: https://your-api.com/docs
- Feature spec: `specs/003-auth-personalization-translation/`
- PHR history: `history/prompts/003-auth-personalization-translation/`

---

**Last Updated**: December 2025
**Version**: 1.0.0
**Status**: Production Ready ✅
