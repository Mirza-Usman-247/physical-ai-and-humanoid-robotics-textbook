# RAG Indexer - Complete Indexing Guide

## Overview

This guide provides complete instructions for indexing the Physical AI & Humanoid Robotics textbook content into Qdrant for the RAG chatbot.

## Quick Start

### Prerequisites

1. **Python 3.11+** installed
2. **Qdrant Cloud account** with a cluster created
3. **OpenRouter API key** for Qwen embeddings
4. **Git** for version tracking

### Installation

```bash
# Install dependencies
pip install qdrant-client openai tiktoken tenacity

# Or from requirements.txt
pip install -r requirements.txt
```

### Environment Setup

Create a `.env` file with your credentials:

```bash
# Qdrant Configuration
QDRANT_URL="https://your-cluster-id.qdrant.io"
QDRANT_API_KEY="your-qdrant-api-key"

# OpenRouter Configuration
OPENROUTER_API_KEY="sk-or-v1-your-openrouter-key"

# Collection Configuration
COLLECTION_NAME="physical-ai-textbook"
```

## Running the Indexer

### Method 1: Using the run_indexing.py Script (Recommended)

```bash
# Set environment variables
export QDRANT_URL="https://your-cluster.qdrant.io"
export QDRANT_API_KEY="your-key"
export OPENROUTER_API_KEY="sk-or-v1-your-key"

# Index all docs (creates collection if needed)
python .claude/skills/rag-indexer/run_indexing.py \
  --directory docs \
  --create-collection

# Verify indexing
python .claude/skills/rag-indexer/run_indexing.py --verify-only
```

### Method 2: Using indexer.py Directly

```bash
cd .claude/skills/rag-indexer

python indexer.py \
  --qdrant-url "https://your-cluster.qdrant.io" \
  --qdrant-api-key "your-key" \
  --openrouter-api-key "sk-or-v1-your-key" \
  --collection "physical-ai-textbook" \
  --directory "../../../docs" \
  --commit-hash "$(git rev-parse HEAD)" \
  --create-collection
```

### Method 3: Using rag-manager Skill

```bash
cd .claude/skills/rag-manager

python manager.py \
  --qdrant-url "https://your-cluster.qdrant.io" \
  --qdrant-api-key "your-key" \
  --openrouter-api-key "sk-or-v1-your-key" \
  --collection "physical-ai-textbook" \
  index \
  --directory "../../../docs" \
  --commit-hash "$(git rev-parse HEAD)" \
  --force-recreate
```

## Indexing Full Textbook Content

### Actual Content Structure

The textbook is organized into 5 modules with 21 chapters:

```
docs/
├── intro.md, glossary.md, notation.md, references.md, hardware-lab.md, about.md (6 files)
├── module-0-foundations/
│   ├── index.mdx
│   ├── chapter-1/introduction-to-physical-ai.mdx
│   ├── chapter-2/robotics-simulation-fundamentals.mdx
│   └── chapter-3/control-algorithms-for-physical-ai.mdx (3 chapters)
├── module-1-ros2/
│   ├── index.mdx
│   ├── chapter-1/ros2-architecture.mdx
│   ├── chapter-2/publisher-subscriber.mdx
│   ├── chapter-3/services-actions.mdx
│   ├── chapter-4/tf-transforms.mdx
│   └── chapter-5/robot-control.mdx (5 chapters)
├── module-2-digital-twin/
│   ├── index.mdx
│   ├── chapter-1/simulation-principles.mdx
│   ├── chapter-2/gazebo-basics.mdx
│   ├── chapter-3/unity-integration.mdx
│   └── chapter-4/physics-accuracy.mdx (4 chapters)
├── module-3-isaac/
│   ├── index.mdx
│   ├── chapter-1/isaac-overview.mdx
│   ├── chapter-2/perception-pipelines.mdx
│   ├── chapter-3/reinforcement-learning.mdx
│   ├── chapter-4/sim-to-real-transfer.mdx
│   └── chapter-5/advanced-ai-integration.mdx (5 chapters)
└── module-4-vla-humanoids/
    ├── index.mdx
    ├── chapter-1/vla-overview.mdx
    ├── chapter-2/humanoid-control.mdx
    └── chapter-3/system-integration.mdx (3 chapters)

Total: 21 chapters + 5 module indexes + 6 docs = 34 files
```

### Indexing Process

1. **Initial Indexing** (First Time):
   ```bash
   python run_indexing.py \
     --directory docs \
     --create-collection
   ```

   **Expected Output**:
   - Files indexed: 21 chapters + 5 module indexes + 6 docs (34 files)
   - Total chunks: ~700-900 chunks
   - Storage: ~80-100 MB in Qdrant
   - Time: ~3-5 minutes
   - Cost: ~$0.30-0.40 (Qwen embeddings)

2. **Incremental Re-indexing** (After Updates):
   ```bash
   # Re-index specific file
   python indexer.py \
     --qdrant-url "$QDRANT_URL" \
     --qdrant-api-key "$QDRANT_API_KEY" \
     --openrouter-api-key "$OPENROUTER_API_KEY" \
     --collection "physical-ai-textbook" \
     --directory "docs/module-1-ros2/chapter-3.md" \
     --commit-hash "$(git rev-parse HEAD)"
   ```

3. **Verification**:
   ```bash
   python run_indexing.py --verify-only
   ```

   **Expected Output**:
   ```
   ✅ Collection verified!
      - Vectors count: 700-900
      - Points count: 700-900
      - Status: green
   ```

## Current Repository Content

The repository contains **complete textbook content**:
- **21 chapters** (.mdx files in module-*/chapter-* directories)
- **5 module index files** (.mdx files)
- **6 documentation files** (.md files: intro, glossary, notation, references, hardware-lab, about)
- **Total: 34 files**
- **Expected chunks: ~700-900**

To index all content:

```bash
python run_indexing.py \
  --directory docs \
  --create-collection
```

**Note**: The indexer currently supports .md files only. To index .mdx files, update the `file_pattern` parameter to `**/*.{md,mdx}` in indexer.py or convert .mdx to .md first.

## Chunk Metadata Structure

Each indexed chunk includes:

```json
{
  "chunk_id": "docs/module-1/chapter-3.md:45",
  "content": "Full chunk text (512 tokens avg)",
  "file_path": "docs/module-1-ros2/chapter-3.md",
  "section_h1": "Introduction to ROS2",
  "section_h2": "Core Concepts",
  "section_h3": "Nodes and Topics",
  "line_start": 45,
  "line_end": 67,
  "last_indexed": "2025-12-16T14:30:00Z",
  "commit_hash": "abc123def456",
  "version_tag": "textbook-v1-0",
  "token_count": 487,
  "module": "module-1-ros2",
  "chapter": "chapter-3",
  "tags": ["ros", "nodes", "topics"]
}
```

## Chunking Strategy

- **Method**: Semantic chunking by H2/H3 headings
- **Chunk size**: 512 tokens (average)
- **Overlap**: 128 tokens between chunks
- **Encoding**: tiktoken cl100k_base
- **Vector size**: 1536 dimensions (Qwen embeddings)
- **Distance metric**: Cosine similarity

## CI/CD Integration

### GitHub Actions Workflow

Create `.github/workflows/reindex.yml`:

```yaml
name: Re-index Textbook

on:
  push:
    branches: [main]
    paths:
      - 'docs/**/*.md'

jobs:
  reindex:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 2

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: pip install qdrant-client openai tiktoken tenacity

      - name: Get changed files
        id: changed-files
        run: |
          git diff --name-only HEAD~1 HEAD | grep '\.md$' > changed_files.txt || true

      - name: Re-index changed files
        if: steps.changed-files.outputs.files != ''
        run: |
          while IFS= read -r file; do
            if [ -f "$file" ]; then
              python .claude/skills/rag-indexer/indexer.py \
                --qdrant-url "${{ secrets.QDRANT_URL }}" \
                --qdrant-api-key "${{ secrets.QDRANT_API_KEY }}" \
                --openrouter-api-key "${{ secrets.OPENROUTER_API_KEY }}" \
                --collection "physical-ai-textbook" \
                --directory "$file" \
                --commit-hash "${{ github.sha }}"
            fi
          done < changed_files.txt

      - name: Verify indexing
        run: |
          python .claude/skills/rag-indexer/run_indexing.py --verify-only
```

## Troubleshooting

### Common Issues

**1. "Collection not found"**
```
Solution: Add --create-collection flag
```

**2. "API key not set or invalid"**
```
Solution: Check OPENROUTER_API_KEY environment variable
```

**3. "Token limit exceeded"**
```
Solution: Reduce chunk_size in chunker.py (default: 512)
```

**4. "Connection refused"**
```
Solution: Check QDRANT_URL is correct and cluster is running
```

### Performance Optimization

1. **Batch Processing**: Use `index_directory()` instead of individual files
2. **Caching**: Results are cached for 24 hours
3. **Parallel Processing**: Set `workers=4` for faster indexing
4. **Rate Limiting**: Default 500ms delay between batches

## Cost Estimates

**Qwen Embeddings via OpenRouter**:
- Input tokens: ~250K tokens (full textbook: 21 chapters + 5 module indexes + 6 docs)
- Cost: ~$0.30-0.40 per full index
- Re-index (1 chapter): ~$0.01

**Qdrant Cloud Free Tier**:
- Storage: 1GB (enough for ~8,000 chunks)
- Vectors: ~700-900 chunks = ~80-100 MB
- Cost: $0 (within free tier)

**Total Cost**:
- Initial indexing: ~$0.30-0.40
- Monthly maintenance: ~$0-5

## Monitoring

### Check Collection Health

```python
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
info = client.get_collection("physical-ai-textbook")

print(f"Vectors: {info.vectors_count}")
print(f"Status: {info.status}")
```

### Query Test

```bash
cd .claude/skills/rag-retriever

python retriever.py \
  --qdrant-url "$QDRANT_URL" \
  --qdrant-api-key "$QDRANT_API_KEY" \
  --openrouter-api-key "$OPENROUTER_API_KEY" \
  --collection "physical-ai-textbook" \
  --query "What is forward kinematics?" \
  --top-k 5
```

## Next Steps

After successful indexing:

1. **Start Backend API**:
   ```bash
   cd backend
   uvicorn src.api.main:app --reload
   ```

2. **Test RAG Pipeline**:
   ```bash
   curl -X POST http://localhost:8000/chat/query \
     -H "Content-Type: application/json" \
     -d '{"query": "What is forward kinematics?"}'
   ```

3. **Deploy Frontend**: Integrate ChatWidget component into Docusaurus

## References

- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenRouter API](https://openrouter.ai/docs)
- [tiktoken](https://github.com/openai/tiktoken)
- [ADR-003: Git-Based Incremental Re-indexing](../../../history/adr/003-git-based-incremental-reindexing.md)
