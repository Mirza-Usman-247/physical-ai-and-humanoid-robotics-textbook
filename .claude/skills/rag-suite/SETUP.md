# RAG Suite Setup & Configuration

Complete setup guide for the RAG (Retrieval-Augmented Generation) skills suite for the Physical AI & Humanoid Robotics textbook.

## Overview

The RAG Suite consists of four interconnected skills:

1. **rag-indexer** - Index Docusaurus content into Qdrant
2. **rag-retriever** - Semantic search and retrieval
3. **rag-answerer** - Question answering with citations
4. **rag-manager** - Collection management and maintenance

## Prerequisites

### System Requirements

- **Operating System**: Linux, macOS, or Windows with WSL2
- **Memory**: Minimum 8GB RAM (16GB recommended)
- **Disk Space**: Minimum 10GB free
- **Network**: Stable internet connection for embedding API calls

### Required Software

1. **Docker** (for Qdrant)
   ```bash
   # Install Docker: https://docs.docker.com/get-docker/
   docker --version  # Verify installation
   ```

2. **Node.js** (v18+) - Already installed for Docusaurus
   ```bash
   node --version  # Should be >= 18.0.0
   ```

3. **Python** (3.8+) - For embedding and indexing scripts
   ```bash
   python3 --version  # Should be >= 3.8
   # or
   python --version
   ```

## Installation Steps

### Step 1: Install Qdrant Vector Database

**Option A: Docker (Recommended)**

```bash
# Pull latest Qdrant image
docker pull qdrant/qdrant

# Run Qdrant with persistent storage
docker run -d \
  --name qdrant \
  -p 6333:6333 \
  -p 6334:6334 \
  -v $(pwd)/qdrant_storage:/qdrant/storage \
  qdrant/qdrant

# Verify Qdrant is running
curl http://localhost:6333/
# Should return: {"title":"qdrant - vector search engine",...}
```

**Option B: Qdrant Cloud**

1. Sign up at https://qdrant.to/cloud
2. Create a cluster
3. Get API key and cluster URL
4. Use in environment variables (see Step 3)

**Option C: Local Binary (Linux/macOS)**

```bash
# Download and install Qdrant
curl -L https://github.com/qdrant/qdrant/releases/latest/download/qdrant-x86_64-unknown-linux-musl.tar.gz | tar xz
./qdrant --config-path ./config.yaml
```

### Step 2: Install Python Dependencies

Create a virtual environment and install required packages:

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
# On Linux/macOS:
source venv/bin/activate
# On Windows:
venv\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install qdrant-client openai tiktoken markdown pyyaml python-dotenv

# Optional: Install additional embedding providers
pip install cohere sentence-transformers anthropic

# Optional: Install for re-ranking
pip install sentence-transformers torch
```

### Step 3: Configure Environment Variables

Create a `.env` file in your project root:

```bash
# .env file for RAG Suite

# ═══════════════════════════════════════════════════════
# Qdrant Configuration
# ═══════════════════════════════════════════════════════

# For local Docker Qdrant
QDRANT_URL=http://localhost:6333

# For Qdrant Cloud
# QDRANT_URL=https://your-cluster-url.qdrant.io
# QDRANT_API_KEY=your-api-key-here

# ═══════════════════════════════════════════════════════
# Embedding Provider Configuration (choose one or multiple)
# ═══════════════════════════════════════════════════════

# OpenAI (recommended for best quality)
OPENAI_API_KEY=sk-your-openai-api-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small  # or text-embedding-3-large
OPENAI_EMBEDDING_DIMENSIONS=1536               # 1536 for small, 3072 for large

# Cohere
# COHERE_API_KEY=your-cohere-api-key-here
# COHERE_EMBEDDING_MODEL=embed-english-v3.0

# Anthropic (for answer generation with Claude)
# ANTHROPIC_API_KEY=your-anthropic-api-key-here

# Local embeddings (no API key needed, runs on your machine)
# EMBEDDING_PROVIDER=sentence-transformers
# EMBEDDING_MODEL=all-MiniLM-L6-v2  # 384 dimensions, fast
# or
# EMBEDDING_MODEL=all-mpnet-base-v2  # 768 dimensions, higher quality

# ═══════════════════════════════════════════════════════
# RAG Suite Defaults
# ═══════════════════════════════════════════════════════

RAG_DEFAULT_COLLECTION=physical-ai-textbook
RAG_DOCS_PATH=./docs
RAG_CHUNK_SIZE=512                    # tokens per chunk
RAG_CHUNK_OVERLAP=128                 # overlap between chunks
RAG_BATCH_SIZE=10                     # documents per batch

# Retrieval defaults
RAG_DEFAULT_TOP_K=5                   # number of results
RAG_SIMILARITY_THRESHOLD=0.7          # minimum similarity score
RAG_SEARCH_MODE=semantic              # semantic | hybrid | keyword

# Answer generation defaults
RAG_ANSWER_LENGTH=brief               # brief | medium | detailed | comprehensive
RAG_ANSWER_STYLE=technical            # technical | beginner-friendly | tutorial | reference
RAG_CONFIDENCE_THRESHOLD=0.7          # minimum confidence for answers
RAG_LLM_TEMPERATURE=0.2               # temperature for answer generation
RAG_LLM_MODEL=gpt-4                   # or gpt-3.5-turbo, claude-3-sonnet, etc.

# Cleanup defaults
RAG_CLEANUP_AGE_THRESHOLD=90          # days for stale data
RAG_BACKUP_DIR=./backups              # backup location

# ═══════════════════════════════════════════════════════
# Performance Tuning (Optional)
# ═══════════════════════════════════════════════════════

# HNSW indexing parameters
QDRANT_HNSW_M=16                      # edges per node (default: 16)
QDRANT_HNSW_EF_CONSTRUCT=100          # construction depth (default: 100)

# Storage thresholds
QDRANT_INDEXING_THRESHOLD=20000       # when to start indexing
QDRANT_MEMMAP_THRESHOLD=50000         # when to use memory mapping

# Rate limiting
EMBEDDING_API_MAX_RETRIES=3
EMBEDDING_API_RETRY_DELAY=2           # seconds

# ═══════════════════════════════════════════════════════
# Logging and Debugging
# ═══════════════════════════════════════════════════════

LOG_LEVEL=INFO                        # DEBUG | INFO | WARNING | ERROR
RAG_VERBOSE=false                     # detailed logging
RAG_DRY_RUN=false                     # simulate operations without executing
```

### Step 4: Verify Installation

Run the verification script:

```bash
# Create verification script
cat > verify_setup.py << 'EOF'
#!/usr/bin/env python3
"""Verify RAG Suite setup"""

import sys
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

print("RAG Suite Setup Verification\n" + "="*50)

# Check Python version
print("\n[1] Python Version")
print(f"    {sys.version}")
if sys.version_info < (3, 8):
    print("    ❌ Python 3.8+ required")
    sys.exit(1)
print("    ✓ OK")

# Check required packages
print("\n[2] Python Dependencies")
required_packages = [
    'qdrant_client',
    'openai',
    'tiktoken',
    'markdown',
    'yaml',
    'dotenv'
]

for package in required_packages:
    try:
        __import__(package)
        print(f"    ✓ {package}")
    except ImportError:
        print(f"    ❌ {package} not installed")

# Check Qdrant connection
print("\n[3] Qdrant Connection")
try:
    from qdrant_client import QdrantClient

    qdrant_url = os.getenv('QDRANT_URL', 'http://localhost:6333')
    client = QdrantClient(url=qdrant_url)

    # Try to get collections (will work even if empty)
    collections = client.get_collections()
    print(f"    ✓ Connected to Qdrant at {qdrant_url}")
    print(f"    Collections: {len(collections.collections)}")
except Exception as e:
    print(f"    ❌ Cannot connect to Qdrant: {e}")
    print(f"    Make sure Qdrant is running at {os.getenv('QDRANT_URL', 'http://localhost:6333')}")

# Check embedding API
print("\n[4] Embedding API")
openai_key = os.getenv('OPENAI_API_KEY')
if openai_key:
    if openai_key.startswith('sk-'):
        print(f"    ✓ OpenAI API key configured")
        # Optionally test the API
        try:
            from openai import OpenAI
            client = OpenAI(api_key=openai_key)
            # Small test
            response = client.embeddings.create(
                input="test",
                model=os.getenv('OPENAI_EMBEDDING_MODEL', 'text-embedding-3-small')
            )
            print(f"    ✓ OpenAI API working (dimension: {len(response.data[0].embedding)})")
        except Exception as e:
            print(f"    ⚠️  OpenAI API key set but test failed: {e}")
    else:
        print(f"    ⚠️  OpenAI API key format looks incorrect")
else:
    print(f"    ⚠️  No OpenAI API key found in environment")

# Check docs directory
print("\n[5] Documentation Directory")
docs_path = os.getenv('RAG_DOCS_PATH', './docs')
if os.path.isdir(docs_path):
    md_files = len([f for f in os.listdir(docs_path) if f.endswith('.md')])
    print(f"    ✓ Docs directory exists: {docs_path}")
    print(f"    Markdown files found: {md_files}")
else:
    print(f"    ❌ Docs directory not found: {docs_path}")

print("\n" + "="*50)
print("\nSetup verification complete!")
print("\nNext steps:")
print("1. Create collection: Use /rag-manager create")
print("2. Index content: Use /rag-indexer")
print("3. Test retrieval: Use /rag-retriever")
print("4. Ask questions: Use /rag-answerer")
EOF

chmod +x verify_setup.py
python3 verify_setup.py
```

Expected output:
```
RAG Suite Setup Verification
==================================================

[1] Python Version
    3.10.12 (main, ...)
    ✓ OK

[2] Python Dependencies
    ✓ qdrant_client
    ✓ openai
    ✓ tiktoken
    ✓ markdown
    ✓ yaml
    ✓ dotenv

[3] Qdrant Connection
    ✓ Connected to Qdrant at http://localhost:6333
    Collections: 0

[4] Embedding API
    ✓ OpenAI API key configured
    ✓ OpenAI API working (dimension: 1536)

[5] Documentation Directory
    ✓ Docs directory exists: ./docs
    Markdown files found: 42

==================================================

Setup verification complete!

Next steps:
1. Create collection: Use /rag-manager create
2. Index content: Use /rag-indexer
3. Test retrieval: Use /rag-retriever
4. Ask questions: Use /rag-answerer
```

## Quick Start Guide

### 1. Create Qdrant Collection

```bash
# Using rag-manager skill in Claude Code
/rag-manager create --collection physical-ai-textbook --vector-size 1536
```

Or manually:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

client = QdrantClient(url="http://localhost:6333")

client.create_collection(
    collection_name="physical-ai-textbook",
    vectors_config=VectorParams(
        size=1536,  # text-embedding-3-small
        distance=Distance.COSINE
    )
)
```

### 2. Index Textbook Content

```bash
# Using rag-indexer skill in Claude Code
/rag-indexer --collection physical-ai-textbook --docs-path ./docs
```

This will:
- Scan all `.mdx` files in `./docs`
- Chunk content semantically (512 tokens with 128 overlap)
- Generate embeddings using OpenAI
- Upload to Qdrant with metadata

Expected time: ~3-5 minutes for 21 chapters (34 files total: 21 chapters + 5 module indexes + 6 docs)

### 3. Test Retrieval

```bash
# Using rag-retriever skill
/rag-retriever "What is forward kinematics?" --top-k 5
```

Expected output: 5 relevant chunks with scores and source references

### 4. Ask Questions

```bash
# Using rag-answerer skill
/rag-answerer "How do I create a ROS 2 publisher in Python?"
```

Expected output: Answer with inline citations and source list

## Advanced Configuration

### Choosing an Embedding Model

**OpenAI text-embedding-3-small** (Default, Recommended)
- Dimensions: 1536
- Cost: $0.02 per 1M tokens
- Quality: Excellent
- Speed: Fast (via API)

**OpenAI text-embedding-3-large**
- Dimensions: 3072
- Cost: $0.13 per 1M tokens
- Quality: Best
- Speed: Fast (via API)

**Cohere embed-english-v3.0**
- Dimensions: 1024
- Cost: Free tier available
- Quality: Good
- Speed: Fast (via API)

**Sentence-Transformers (Local)**
- Dimensions: 384-768
- Cost: Free (local)
- Quality: Good
- Speed: Depends on hardware

### Optimizing Chunk Size

**Small chunks (128-256 tokens)**
- More precise retrieval
- Better for specific facts
- More storage, more API calls
- Higher costs

**Medium chunks (512-1024 tokens)** ✓ Recommended
- Good balance
- Maintains context
- Reasonable costs
- Works well for textbook content

**Large chunks (1024-2048 tokens)**
- More context per chunk
- Fewer API calls
- Less precise retrieval
- Risk of context overflow

### Hybrid Search Configuration

For best retrieval quality, use hybrid search (vector + keyword):

```python
# In rag-retriever configuration
search_mode = "hybrid"
semantic_weight = 0.7  # 70% semantic
keyword_weight = 0.3   # 30% keyword (BM25)
```

Adjust weights based on query type:
- Technical definitions: Higher semantic (0.8)
- Code examples: Higher keyword (0.5-0.5)
- General concepts: Balanced (0.7-0.3)

### Re-Ranking Options

**Cross-Encoder Re-Ranking** (Highest Quality)
```bash
# Requires: pip install sentence-transformers
RAG_RERANK_MODE=cross-encoder
RAG_RERANK_MODEL=cross-encoder/ms-marco-MiniLM-L-6-v2
```

**MMR (Diversity)** (Best for Broad Topics)
```bash
RAG_RERANK_MODE=mmr
RAG_MMR_LAMBDA=0.7  # 70% relevance, 30% diversity
```

**Cohere Re-Rank API** (Easiest)
```bash
RAG_RERANK_MODE=cohere
COHERE_API_KEY=your-key
```

## Production Deployment

### Qdrant Cloud Setup

1. **Create Cluster**
   - Go to https://qdrant.to/cloud
   - Create account and cluster
   - Choose region closest to users

2. **Get Credentials**
   ```bash
   QDRANT_URL=https://xxxxx.us-east.aws.cloud.qdrant.io
   QDRANT_API_KEY=your-api-key
   ```

3. **Migrate Data**
   ```bash
   # Export from local
   /rag-manager export --collection physical-ai-textbook --path backup.snapshot

   # Import to cloud (update QDRANT_URL in .env)
   /rag-manager import --collection physical-ai-textbook --path backup.snapshot
   ```

### Scaling Considerations

**For Large Collections (>100k chunks):**

1. **Enable Memory Mapping**
   ```python
   QDRANT_MEMMAP_THRESHOLD=50000
   ```

2. **Adjust HNSW Parameters**
   ```python
   QDRANT_HNSW_M=32          # More edges (default: 16)
   QDRANT_HNSW_EF_CONSTRUCT=200  # Deeper search (default: 100)
   ```

3. **Use Quantization** (Qdrant Cloud)
   - Reduces memory by 4x
   - Minimal quality loss
   - Enable in Qdrant Cloud console

### Monitoring & Logging

Enable comprehensive logging:

```bash
# .env
LOG_LEVEL=INFO
RAG_VERBOSE=true

# Optional: Send logs to file
RAG_LOG_FILE=./logs/rag-suite.log
```

Track these metrics:
- Indexing throughput (chunks/second)
- Search latency (p50, p95, p99)
- Embedding API costs
- Collection size growth
- Cache hit rates

## Troubleshooting

### Common Issues

**Issue: "Cannot connect to Qdrant"**
```bash
# Check if Qdrant is running
docker ps | grep qdrant

# Start Qdrant if not running
docker start qdrant

# Or run new container
docker run -d -p 6333:6333 --name qdrant qdrant/qdrant
```

**Issue: "Embedding API rate limit"**
```bash
# Reduce batch size
RAG_BATCH_SIZE=5  # Default is 10

# Add retry delays
EMBEDDING_API_MAX_RETRIES=5
EMBEDDING_API_RETRY_DELAY=5  # seconds
```

**Issue: "Poor retrieval quality"**
```bash
# Try hybrid search
RAG_SEARCH_MODE=hybrid

# Lower similarity threshold
RAG_SIMILARITY_THRESHOLD=0.6

# Re-index with smaller chunks
RAG_CHUNK_SIZE=256
```

**Issue: "Out of memory during indexing"**
```bash
# Reduce batch size
RAG_BATCH_SIZE=5

# Enable incremental mode
/rag-indexer --incremental

# Use memory mapping (Qdrant Cloud)
QDRANT_MEMMAP_THRESHOLD=10000
```

### Debug Mode

Enable debug mode for detailed logging:

```bash
LOG_LEVEL=DEBUG
RAG_VERBOSE=true
RAG_DRY_RUN=true  # Test without actually modifying data
```

## Maintenance Schedule

### Daily
- [ ] Health check: `/rag-manager health`
- [ ] Monitor search latency
- [ ] Check error logs

### Weekly
- [ ] Incremental re-index: `/rag-indexer --incremental`
- [ ] Review "I don't know" responses
- [ ] Update stale content

### Monthly
- [ ] Full cleanup: `/rag-manager cleanup --mode all`
- [ ] Optimize collection: `/rag-manager optimize`
- [ ] Backup: `/rag-manager export`
- [ ] Review and adjust chunk sizes

### Quarterly
- [ ] Full re-index with latest content
- [ ] Evaluate embedding model performance
- [ ] Consider re-ranking strategies
- [ ] Archive old collections

## Security Best Practices

1. **API Keys**
   - Never commit `.env` to git
   - Use environment variables in production
   - Rotate keys regularly

2. **Qdrant Access**
   - Use API keys for Qdrant Cloud
   - Restrict network access
   - Enable TLS/SSL

3. **Data Privacy**
   - Ensure textbook content can be indexed
   - Don't index sensitive information
   - Comply with data retention policies

## Cost Estimation

### OpenAI Embedding Costs (text-embedding-3-small)

**Initial Indexing (21 chapters, 34 files total, ~250k tokens):**
- Cost: ~$0.005 (250k tokens × $0.02/1M)
- One-time expense

**Incremental Updates (weekly, ~50k tokens):**
- Cost: ~$0.001/week
- ~$0.05/year

**Monthly Re-Index:**
- Cost: ~$0.01/month
- ~$0.12/year

**Answer Generation (GPT-4, 100 queries/day):**
- Input: ~500 tokens/query (retrieved context)
- Output: ~200 tokens/query (answer)
- Cost: ~$0.70/day (~$255/year)

**Total Estimated Cost:**
- Indexing: ~$0.15/year
- Answers: ~$255/year (with GPT-4)
- **Total: ~$255/year** (or ~$25/year with GPT-3.5-turbo)

### Qdrant Costs

**Local (Docker):**
- Free
- Requires server/hardware

**Qdrant Cloud:**
- Free tier: 1GB storage, sufficient for textbook
- Paid: ~$25/month for production use

## Support & Resources

### Documentation
- Qdrant Docs: https://qdrant.tech/documentation/
- OpenAI Embeddings: https://platform.openai.com/docs/guides/embeddings
- Sentence Transformers: https://www.sbert.net/

### Community
- Qdrant Discord: https://qdrant.to/discord
- GitHub Issues: Submit issues for this project

### Getting Help
1. Check troubleshooting section above
2. Review skill-specific SKILL.md files
3. Run health checks: `/rag-manager health`
4. Enable debug logging
5. Open an issue with logs and reproduction steps

## Next Steps

Once setup is complete:

1. **Index your textbook**: `/rag-indexer`
2. **Test retrieval**: `/rag-retriever "test query"`
3. **Ask questions**: `/rag-answerer "how do I..."`
4. **Explore advanced features**: See EXAMPLES.md
5. **Set up monitoring**: Track metrics and costs
6. **Schedule maintenance**: Implement backup and cleanup routines

Happy querying! 🚀
