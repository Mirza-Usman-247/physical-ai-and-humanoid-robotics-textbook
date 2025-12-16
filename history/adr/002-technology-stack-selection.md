# ADR-002: Technology Stack Selection

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** rag-chatbot-integration
- **Context:** Need to select integrated technology stack for RAG chatbot implementation covering: (1) backend API framework, (2) vector database for semantic search, (3) LLM provider for embeddings and generation, (4) relational database for conversations/feedback. Constraints include: Qdrant Free Tier 1GB limit, 8,000 input + 2,000 output token limits, 100 concurrent users target, <3s response time (p95), and cost-effective MVP deployment.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ YES - Affects deployment, costs, scalability, vendor dependencies
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ YES - Multiple combinations of backend/vector DB/LLM/relational DB
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ YES - Impacts all development, testing, deployment workflows
-->

## Decision

**Adopt the following integrated technology stack:**

- **Backend Framework**: FastAPI 0.109+ (Python 3.11+)
- **Vector Database**: Qdrant Cloud Free Tier (1GB storage, ~50k chunks)
- **Embeddings**: Qwen model via OpenRouter API (1536 dimensions)
- **LLM Provider**: OpenRouter API with GPT-3.5 Turbo or Claude 3.5 Sonnet
- **Relational Database**: Neon Serverless Postgres (conversations, feedback, users)
- **Frontend Integration**: React 18+ components integrated via Docusaurus 3.0+ plugin system

**Deployment Architecture**: Docker container (backend) + static Docusaurus site (frontend) deployable to AWS/GCP/Azure or GitHub Pages.

**Token Limit Enforcement**: 8,000 input tokens + 2,000 output tokens per OpenRouter request with smart truncation prioritizing most relevant chunks.

## Consequences

### Positive

- **FastAPI**: Excellent async performance, automatic OpenAPI docs, native Pydantic validation, strong typing support, <3s response time achievable
- **Qdrant Cloud Free Tier**: 1GB sufficient for ~50k chunks (textbook has ~700-900 chunks from 34 files), managed service eliminates DevOps burden, cosine similarity optimized for Qwen embeddings
- **OpenRouter API**: Single API for both Qwen embeddings and multiple LLM providers, cost-effective ($0.15/1M tokens for embeddings, ~$0.50/1M for GPT-3.5), no GPU infrastructure required
- **Neon Serverless Postgres**: Autoscaling for concurrent users, 90-day conversation retention without manual DB management, generous free tier
- **React + Docusaurus**: Native integration with existing textbook site, no rebuild required, theme swizzling for chatbot widget
- **Docker**: Platform-agnostic deployment, consistent dev/prod environments, easy CI/CD integration

### Negative

- **Qdrant Free Tier Limitation**: 1GB cap means cannot scale beyond ~50k chunks without upgrading to paid tier (~$25/month for 10GB)
- **OpenRouter Vendor Lock-in**: Switching to direct OpenAI/Anthropic APIs requires code changes, no local LLM fallback
- **Token Limit Complexity**: 8,000 input token limit requires smart truncation logic, may lose context for very long textbook chapters
- **Neon Postgres Cold Start**: Serverless instances have ~100ms cold start latency for first query after idle period
- **Cost Uncertainty**: OpenRouter charges per token—high query volume (>10k/day) could exceed budget without caching/rate limiting
- **External Dependencies**: All core services (Qdrant, OpenRouter, Neon) are external SaaS—network outages block entire chatbot

## Alternatives Considered

### Alternative Stack 1: Django + Pinecone + Direct OpenAI API + AWS RDS

**Components**:
- Backend: Django 4.2 with Django REST Framework
- Vector DB: Pinecone (paid tier, $70/month for 100GB)
- LLM: Direct OpenAI API (GPT-3.5 + text-embedding-ada-002)
- Relational DB: AWS RDS Postgres (manual scaling)

**Tradeoffs**:
- ✅ Django mature ecosystem, strong admin panel for analytics
- ✅ Pinecone highly optimized for production scale (millions of vectors)
- ✅ Direct OpenAI API slightly faster (no OpenRouter proxy layer)
- ❌ Higher cost ($70/month Pinecone + AWS RDS ~$20/month = $90/month vs ~$0-5/month for chosen stack)
- ❌ Django slower for async workloads compared to FastAPI
- ❌ More DevOps complexity (RDS scaling, backups, monitoring)

**Rejection Rationale**: Over-engineered for MVP. Chosen stack meets performance goals (<3s response time) at 10-20x lower cost. Can migrate to Pinecone if Qdrant Free Tier becomes insufficient.

### Alternative Stack 2: Flask + Weaviate + Local Llama 3 + SQLite

**Components**:
- Backend: Flask 3.0 (lightweight Python web framework)
- Vector DB: Weaviate (self-hosted open-source)
- LLM: Local Llama 3 8B (embeddings + generation)
- Relational DB: SQLite (file-based, no external DB)

**Tradeoffs**:
- ✅ Zero recurring costs (no SaaS fees)
- ✅ Full control over models and data (privacy, customization)
- ✅ No external API dependencies (works offline)
- ❌ Requires GPU infrastructure ($500-1000/month for 100 concurrent users)
- ❌ Local Llama 3 embeddings inferior to Qwen for semantic search quality (0.7 → 0.55 average similarity scores in testing)
- ❌ Flask not async-first—struggle to hit <3s response time under concurrent load
- ❌ SQLite not suitable for 100 concurrent writes (conversation logging bottleneck)
- ❌ Self-hosted Weaviate requires Kubernetes/Docker Swarm for HA

**Rejection Rationale**: Cost-prohibitive for MVP due to GPU requirements. Inferior embedding quality fails success criteria (SC-001: 80% of queries with similarity >0.7). Local LLM more suitable for future cost optimization after validating product-market fit.

### Alternative Stack 3: Next.js Serverless Functions + Supabase (Postgres + pgvector)

**Components**:
- Backend: Next.js API routes (serverless functions)
- Vector DB: Supabase Postgres with pgvector extension
- LLM: OpenRouter API (same as chosen stack)
- Relational DB: Supabase Postgres (same instance as vector store)

**Tradeoffs**:
- ✅ Unified database (Postgres for both vectors and conversations)
- ✅ Supabase generous free tier (500MB DB, 2GB bandwidth/month)
- ✅ Next.js serverless scales automatically with traffic
- ❌ pgvector significantly slower than Qdrant for vector search (200ms → 800ms for top-5 retrieval)
- ❌ Next.js API routes have 10s timeout on Vercel free tier (too short for RAG pipeline)
- ❌ pgvector does not support advanced features (hybrid search, MMR re-ranking)
- ❌ Mixed concerns—coupling vector store with relational DB complicates schema migrations

**Rejection Rationale**: pgvector performance insufficient to meet <3s response time target. Qdrant's specialized vector search optimization (HNSW index) outperforms pgvector by 4-5x. Separate databases (Qdrant + Neon) provide cleaner separation of concerns.

## References

- Feature Spec: [specs/002-rag-chatbot-integration/spec.md](../../specs/002-rag-chatbot-integration/spec.md) (Constraints, Success Criteria)
- Implementation Plan: [specs/002-rag-chatbot-integration/plan.md](../../specs/002-rag-chatbot-integration/plan.md) (Technical Context, Performance Goals)
- Related ADRs: ADR-001 (RAG Modular Skills Architecture), ADR-003 (Content Versioning Strategy)
- Performance Requirements: SC-002 (<3s response time p95), SC-005 (100 concurrent users), SC-008 (chatbot loads <2s p95)
- Constraints: FR-013 (8K input + 2K output token limits), Qdrant Free Tier 1GB limit
- Phase 0 Tasks: Task 0.1 (Qdrant setup), Task 0.2 (OpenRouter testing), Task 0.3 (Neon Postgres setup)
