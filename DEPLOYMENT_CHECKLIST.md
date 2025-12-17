# Vercel Deployment Checklist

Use this checklist to deploy your Physical AI Textbook project to Vercel.

## 📋 Pre-Deployment Preparation

### 1. Generate Security Keys
- [ ] Generate `JWT_SECRET_KEY` (32+ characters)
  ```bash
  python -c "import secrets; print(secrets.token_urlsafe(32))"
  ```
- [ ] Generate `SECRET_KEY` (32+ characters)
  ```bash
  python -c "import secrets; print(secrets.token_urlsafe(32))"
  ```
- [ ] Generate `BETTER_AUTH_SECRET` (32+ characters) - if using Better Auth
  ```bash
  python -c "import secrets; print(secrets.token_urlsafe(32))"
  ```

### 2. Set Up External Services

#### Neon Serverless Postgres
- [ ] Create account at https://console.neon.tech/
- [ ] Create new project
- [ ] Copy connection string (should include `?sslmode=require`)
- [ ] Save as `NEON_DATABASE_URL`

#### Qdrant Vector Database
- [ ] Create account at https://cloud.qdrant.io/
- [ ] Create new cluster
- [ ] Create collection named `physical-ai-textbook`
  - Vector size: 1536 (for text-embedding-3-small)
  - Distance: Cosine
- [ ] Copy cluster URL and API key
- [ ] Save `QDRANT_URL` and `QDRANT_API_KEY`

#### OpenRouter API
- [ ] Create account at https://openrouter.ai/
- [ ] Add credits to account
- [ ] Generate API key
- [ ] Save as `OPENROUTER_API_KEY`

### 3. Prepare Repository
- [ ] Commit all changes to Git
- [ ] Push to GitHub/GitLab/Bitbucket
- [ ] Verify `.env` files are in `.gitignore` (they should be!)

---

## 🚀 Backend Deployment (Step 1)

### 1. Create Vercel Project
- [ ] Go to https://vercel.com/new
- [ ] Click "Import Git Repository"
- [ ] Select your repository
- [ ] Click "Import"

### 2. Configure Project Settings
- [ ] **Project Name**: `physical-ai-backend` (or your choice)
- [ ] **Framework Preset**: Other
- [ ] **Root Directory**: `backend`
- [ ] **Build Command**: Leave empty
- [ ] **Output Directory**: Leave empty
- [ ] Click "Continue"

### 3. Add Environment Variables

Go to "Environment Variables" section and add:

#### Critical Variables (Copy from your notes)
- [ ] `NEON_DATABASE_URL` = `postgresql://...?sslmode=require`
- [ ] `QDRANT_URL` = `https://your-cluster.qdrant.io`
- [ ] `QDRANT_API_KEY` = `your-qdrant-api-key`
- [ ] `QDRANT_COLLECTION` = `physical-ai-textbook`
- [ ] `OPENROUTER_API_KEY` = `sk-or-v1-...`
- [ ] `JWT_SECRET_KEY` = (generated secret)
- [ ] `SECRET_KEY` = (generated secret)

#### Application Variables (Use these defaults)
- [ ] `ENVIRONMENT` = `production`
- [ ] `DEBUG` = `false`
- [ ] `LOG_LEVEL` = `INFO`
- [ ] `OPENROUTER_BASE_URL` = `https://openrouter.ai/api/v1`
- [ ] `QWEN_EMBEDDING_MODEL` = `text-embedding-3-small`
- [ ] `OPENROUTER_LLM_MODEL` = `anthropic/claude-3.5-sonnet`
- [ ] `OPENROUTER_LLM_MODEL_PERSONALIZE` = `google/gemini-2.0-flash-exp:free`
- [ ] `OPENROUTER_LLM_MODEL_TRANSLATE` = `google/gemini-2.0-flash-exp:free`
- [ ] `JWT_ALGORITHM` = `HS256`
- [ ] `JWT_ACCESS_TOKEN_EXPIRE_MINUTES` = `15`
- [ ] `JWT_REFRESH_TOKEN_EXPIRE_DAYS` = `7`
- [ ] `RATE_LIMIT_PER_MINUTE` = `20`
- [ ] `MAX_INPUT_TOKENS` = `8000`
- [ ] `MAX_OUTPUT_TOKENS` = `2000`
- [ ] `SIMILARITY_THRESHOLD` = `0.7`
- [ ] `TOP_K_CHUNKS` = `5`
- [ ] `CONVERSATION_CONTEXT_WINDOW` = `5`

### 4. Deploy Backend
- [ ] Click "Deploy"
- [ ] Wait for deployment (2-5 minutes)
- [ ] Copy deployed URL: `https://_____.vercel.app`
- [ ] Save this URL for frontend configuration

### 5. Test Backend
- [ ] Visit `https://your-backend.vercel.app/health`
- [ ] Should see: `{"status": "healthy", ...}`
- [ ] Visit `https://your-backend.vercel.app/` for API info
- [ ] If errors, check Logs in Vercel dashboard

### 6. Run Database Migrations
```bash
# Install Vercel CLI
npm i -g vercel

# Navigate to backend
cd backend

# Link to your project
vercel link

# Pull production environment variables
vercel env pull .env.production

# Run migrations
python -m alembic upgrade head
```

- [ ] Migrations completed successfully
- [ ] No errors in output

---

## 🌐 Frontend Deployment (Step 2)

### 1. Create New Vercel Project
- [ ] Go to https://vercel.com/new
- [ ] Import the **same repository** again
- [ ] Click "Import"

### 2. Configure Project Settings
- [ ] **Project Name**: `physical-ai-textbook` (or your choice)
- [ ] **Framework Preset**: Other (or Docusaurus if available)
- [ ] **Root Directory**: `.` (leave as root, don't change)
- [ ] **Build Command**: `docusaurus build --config docusaurus.config.vercel.js`
- [ ] **Output Directory**: `build`
- [ ] Click "Continue"

### 3. Add Environment Variables

- [ ] `NEXT_PUBLIC_API_URL` = `https://your-backend.vercel.app` (from Step 1.4)
- [ ] `BETTER_AUTH_URL` = `https://your-backend.vercel.app/api/auth` (optional)
- [ ] `BETTER_AUTH_SECRET` = (generated secret, optional)

### 4. Deploy Frontend
- [ ] Click "Deploy"
- [ ] Wait for build (3-5 minutes)
- [ ] Copy frontend URL: `https://_____.vercel.app`

### 5. Test Frontend
- [ ] Visit your frontend URL
- [ ] Browse documentation pages
- [ ] Test chat functionality
- [ ] Test authentication (if enabled)
- [ ] Check browser console for errors

---

## 🔗 Connect Backend & Frontend

### 1. Update Backend CORS
Go to backend project settings:
- [ ] Add environment variable: `FRONTEND_URL` = `https://your-frontend.vercel.app`
- [ ] Redeploy backend (Deployments tab → Click "..." → Redeploy)

### 2. Verify Connection
- [ ] Open frontend in browser
- [ ] Open browser DevTools (F12)
- [ ] Try chat feature
- [ ] Check Network tab for API requests
- [ ] Should see successful requests to backend
- [ ] No CORS errors in console

---

## ✅ Post-Deployment Verification

### Backend Checks
- [ ] `/health` endpoint returns healthy status
- [ ] `/` endpoint shows API information
- [ ] `/docs` is disabled (should return 404 in production)
- [ ] Database connection works (no errors in logs)
- [ ] Qdrant connection works
- [ ] OpenRouter API works

### Frontend Checks
- [ ] Homepage loads correctly
- [ ] Navigation works
- [ ] Documentation pages render
- [ ] Images load
- [ ] Search functionality works (if enabled)
- [ ] Chat interface appears
- [ ] Can send messages and get responses
- [ ] Authentication works (if enabled)

### Security Checks
- [ ] HTTPS is enabled (should be automatic)
- [ ] No API keys in frontend code
- [ ] CORS only allows your frontend domain
- [ ] Rate limiting is active
- [ ] JWT tokens expire correctly
- [ ] Passwords are hashed (test registration)

### Performance Checks
- [ ] First page load < 3 seconds
- [ ] API response time < 2 seconds
- [ ] No console errors
- [ ] No broken images/links

---

## 📊 Monitoring Setup

### Vercel Dashboard
- [ ] Bookmark backend project URL
- [ ] Bookmark frontend project URL
- [ ] Enable deployment notifications (Settings → Notifications)
- [ ] Review Analytics tab

### External Monitoring (Optional)
- [ ] Set up Sentry for error tracking
- [ ] Configure uptime monitoring (UptimeRobot, Pingdom)
- [ ] Set up log aggregation (Logtail, Papertrail)

---

## 🎉 Optional Enhancements

### Custom Domains
- [ ] Purchase domain (Namecheap, Google Domains, etc.)
- [ ] Add to frontend project (Settings → Domains)
- [ ] Add to backend project (Settings → Domains)
- [ ] Configure DNS records
- [ ] Wait for SSL certificate (automatic)

### CI/CD
- [ ] Enable automatic deployments from Git
- [ ] Set up preview deployments for PRs
- [ ] Configure deployment protection (production only from main branch)

### Performance
- [ ] Enable Edge caching (Settings → Edge Config)
- [ ] Configure image optimization
- [ ] Review and optimize bundle size

### Analytics
- [ ] Enable Vercel Analytics
- [ ] Set up Google Analytics (optional)
- [ ] Configure user tracking (respecting privacy)

---

## 🆘 Troubleshooting

If something goes wrong, check:

1. **Vercel Function Logs**
   - Backend Project → Logs tab
   - Look for errors in function execution

2. **Build Logs**
   - Deployments tab → Click on deployment → View build logs

3. **Browser Console**
   - Press F12 → Console tab
   - Look for JavaScript errors

4. **Common Issues**
   - Missing environment variables → Add in dashboard
   - CORS errors → Update FRONTEND_URL in backend
   - Database errors → Check connection string
   - Build failures → Review build logs

5. **Get Help**
   - Check `VERCEL_DEPLOYMENT.md` for detailed guide
   - Review `backend/README.vercel.md` for backend-specific help
   - Vercel Discord: https://vercel.com/discord
   - Vercel Support: https://vercel.com/support

---

## 📝 Keep These URLs Handy

After deployment, save these:

```
Backend URL: https://_____.vercel.app
Frontend URL: https://_____.vercel.app

Vercel Backend Dashboard: https://vercel.com/[username]/[backend-project]
Vercel Frontend Dashboard: https://vercel.com/[username]/[frontend-project]

Neon Console: https://console.neon.tech/
Qdrant Cloud: https://cloud.qdrant.io/
OpenRouter: https://openrouter.ai/
```

---

## 🎓 Success!

Once all checkboxes are complete, your Physical AI Textbook is live! 🚀

Share your deployment:
- Tweet about it
- Add to your portfolio
- Share with students/colleagues
- Submit to educational directories

---

**Need Help?** See `VERCEL_DEPLOYMENT.md` for comprehensive documentation.
