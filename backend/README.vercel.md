# Backend Deployment on Vercel

## Quick Start

This FastAPI backend is configured to deploy on Vercel as a serverless function.

### Prerequisites

1. Vercel account
2. All API keys and database credentials ready
3. Frontend deployed (or at least know the URL for CORS)

### Deployment Steps

1. **Push to Git** (GitHub, GitLab, or Bitbucket)

2. **Import to Vercel**
   - Go to https://vercel.com/new
   - Import your repository
   - Set **Root Directory** to `backend`
   - Framework Preset: Other

3. **Configure Environment Variables**
   - Copy all variables from `.env.vercel.example`
   - Go to Project Settings → Environment Variables
   - Add each variable (see detailed list in `VERCEL_DEPLOYMENT.md`)

4. **Deploy**
   - Click "Deploy"
   - Wait for build to complete

5. **Update CORS**
   - After frontend deployment, add frontend URL to environment variables
   - Redeploy to apply CORS changes

## File Structure for Vercel

```
backend/
├── api/
│   └── index.py          # Vercel entry point (serverless function)
├── src/
│   └── api/
│       └── main.py       # FastAPI app
├── vercel.json           # Vercel configuration
├── requirements.prod.txt # Production dependencies
└── .env.vercel.example   # Environment variables template
```

## How It Works

1. **Entry Point**: `api/index.py`
   - Vercel looks for Python files in the `api/` directory
   - `api/index.py` imports and exports the FastAPI app

2. **Serverless Execution**
   - Each request spawns a serverless function
   - Cold starts may occur after inactivity (~1-2 seconds)
   - Function timeout: 60 seconds (configurable in vercel.json)

3. **Environment Variables**
   - Set in Vercel dashboard
   - Loaded automatically by the app
   - Never commit real values to Git

## Testing Locally with Vercel CLI

```bash
# Install Vercel CLI
npm i -g vercel

# Login
vercel login

# Link to your project
vercel link

# Pull environment variables
vercel env pull .env.local

# Run locally
vercel dev
```

## Database Migrations

Migrations need to run separately (Vercel functions are stateless):

```bash
# Option 1: Run locally after pulling env vars
vercel env pull .env.production
python -m alembic upgrade head

# Option 2: Use GitHub Actions (recommended)
# See .github/workflows/migrate.yml (create this file)
```

## Monitoring

1. **Function Logs**
   - Vercel Dashboard → Your Project → Logs
   - Real-time log streaming

2. **Error Tracking**
   - Consider Sentry integration
   - Add `sentry-sdk` to requirements

3. **Performance**
   - Monitor cold start times
   - Check function execution duration
   - Review memory usage

## Limitations & Considerations

### Vercel Free Tier Limits
- 100 GB bandwidth/month
- 100 GB-hours serverless function execution
- 60-second function timeout
- 50 MB max function size

### Serverless Considerations
- **Cold Starts**: First request after inactivity is slower
- **Stateless**: No persistent file storage (use databases)
- **Connection Pooling**: Limited due to function lifecycle
- **Background Jobs**: Not suitable (use cron jobs or separate service)

### Database Connections
- Serverless functions create new connections frequently
- Use connection pooling (SQLAlchemy does this)
- Neon Serverless Postgres is optimized for serverless
- Consider Neon's pooler for better performance

## Troubleshooting

### Build Fails

**Problem**: `Module not found`
```bash
# Solution: Ensure all imports use absolute paths
# Wrong: from api.routes import health
# Right: from src.api.routes import health
```

**Problem**: `Package installation failed`
```bash
# Solution: Check requirements.prod.txt for incompatible versions
# Remove dev dependencies (pytest, mypy, etc.)
```

### Runtime Errors

**Problem**: `Database connection timeout`
```bash
# Solution:
# 1. Verify NEON_DATABASE_URL is correct
# 2. Add ?sslmode=require to connection string
# 3. Check Neon instance is running
```

**Problem**: `CORS errors from frontend`
```bash
# Solution:
# 1. Add frontend URL to CORS allow_origins
# 2. Ensure credentials=True in frontend fetch
# 3. Redeploy after CORS changes
```

**Problem**: `Function timeout`
```bash
# Solution:
# 1. Optimize slow database queries
# 2. Reduce API response times
# 3. Increase timeout in vercel.json (max 60s on free tier)
```

### Environment Variables

**Problem**: `Config validation error`
```bash
# Solution: Check all required env vars are set in Vercel dashboard
# Use .env.vercel.example as reference
```

## Security Best Practices

- ✅ Never commit `.env` files
- ✅ Use strong random keys (32+ characters)
- ✅ Set `DEBUG=false` in production
- ✅ Configure specific CORS origins (no wildcards)
- ✅ Enable rate limiting
- ✅ Use SSL for database connections
- ✅ Rotate secrets periodically
- ✅ Monitor function logs for suspicious activity

## Performance Optimization

1. **Reduce Cold Starts**
   - Keep dependencies minimal
   - Use lighter packages when possible
   - Consider serverless function warming

2. **Database Optimization**
   - Use connection pooling
   - Add database indexes
   - Optimize query performance
   - Use Neon's pooler

3. **Caching**
   - Cache vector embeddings
   - Use Redis for session storage (optional)
   - Cache AI model responses (be cautious)

## Cost Optimization

**Vercel**: Free tier is generous, but monitor:
- Bandwidth usage
- Function execution time
- Number of invocations

**External Services** (where costs accrue):
- **OpenRouter**: Pay per API call
  - Use cheaper models for non-critical tasks
  - Cache responses when appropriate
  - Set token limits
- **Neon Database**: Free tier available
  - Monitor storage usage
  - Optimize queries
- **Qdrant Cloud**: Free tier available
  - Monitor vector count
  - Optimize collection size

## Support

- Vercel Documentation: https://vercel.com/docs
- FastAPI on Vercel: https://vercel.com/guides/using-fastapi-with-vercel
- Issues: Check main repository issues

## Next Steps

After successful deployment:
1. ✅ Test all API endpoints
2. ✅ Run database migrations
3. ✅ Configure custom domain (optional)
4. ✅ Set up monitoring/alerts
5. ✅ Configure automatic deployments
6. ✅ Update frontend to use production backend URL
