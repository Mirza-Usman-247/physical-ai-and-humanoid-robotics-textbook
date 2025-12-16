# RAG Indexer Skill

## Overview

The RAG Indexer skill ingests markdown textbook content, chunks it semantically, generates embeddings, and stores them in Qdrant vector database for retrieval-augmented generation (RAG).

**Purpose**: Enable semantic search over textbook chapters by indexing content with proper chunking and embedding strategies.

**Use Cases**:
- Initial indexing of all textbook chapters
- Incremental re-indexing when content changes (git-based versioning)
- Re-indexing specific files or directories
- Deleting outdated chunks from removed files

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                   RAGIndexer                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────┐ │
│  │   Chunker    │  │  Embeddings  │  │  Qdrant  │ │
│  │  (512 tok)   │→ │ (Qwen 1536d) │→ │  Client  │ │
│  └──────────────┘  └──────────────┘  └──────────┘ │
└─────────────────────────────────────────────────────┘
         ↓                    ↓                 ↓
    H2/H3 splits      OpenRouter API     Cosine search
```

## Components

### 1. indexer.py - Main Orchestration

**Class**: RAGIndexer

**Responsibilities**:
- Coordinate chunking, embedding, and storage
- Create and manage Qdrant collections
- Index individual files or entire directories
- Support incremental re-indexing with git commit hashes
- Delete outdated chunks

**Key Methods**:
```python
RAGIndexer(qdrant_url, qdrant_api_key, collection_name, embedding_generator)
create_collection(vector_size=1536, force_recreate=False)
index_file(file_path, commit_hash=None, module=None, chapter=None) -> List[str]
index_directory(directory_path, file_pattern="**/*.md", commit_hash=None) -> Dict
delete_file_chunks(file_path) -> int
```

### 2. chunker.py - Semantic Chunking

**Class**: MarkdownChunker

**Strategy**: Split by heading structure (H2/H3) to preserve semantic boundaries.

**Parameters**:
- chunk_size: Target chunk size in tokens (default: 512)
- chunk_overlap: Overlapping tokens between chunks (default: 128)
- encoding: tiktoken cl100k_base for accurate token counting

**Features**:
- Preserves heading hierarchy (H1, H2, H3) in metadata
- Extracts relevant tags (kinematics, ROS, deep learning, etc.)
- Tracks line ranges for precise citation
- Handles sections larger than chunk_size by splitting intelligently

### 3. embeddings.py - Embedding Generation

**Class**: EmbeddingGenerator

**Model**: Qwen-2-Embedding via OpenRouter API (1536 dimensions)

**Features**:
- Batch processing (100 texts per request)
- Automatic retry with exponential backoff
- Rate limiting protection (500ms delay between batches)
- Embedding validation

## Usage

### Installation

```bash
# Install dependencies
pip install qdrant-client openai tiktoken tenacity

# Set environment variables
export QDRANT_URL="https://your-cluster.qdrant.io"
export QDRANT_API_KEY="your-api-key"
export OPENROUTER_API_KEY="sk-or-v1-your-key"
```

### Basic Usage

#### Index a Single File

```python
from indexer import RAGIndexer
from embeddings import EmbeddingGenerator

# Initialize
embedding_gen = EmbeddingGenerator(
    api_key="sk-or-v1-your-key",
    model="qwen/qwen-2-embedding"
)

indexer = RAGIndexer(
    qdrant_url="https://your-cluster.qdrant.io",
    qdrant_api_key="your-api-key",
    collection_name="physical-ai-textbook",
    embedding_generator=embedding_gen
)

# Create collection (first time only)
indexer.create_collection()

# Index a file
chunk_ids = indexer.index_file(
    file_path="docs/module-1-ros2/chapter-3.md",
    commit_hash="abc123def456"
)
```

#### Index Entire Directory

```python
result = indexer.index_directory(
    directory_path="docs",
    file_pattern="**/*.md",
    commit_hash="abc123def456"
)
```

### Command-Line Usage

```bash
python indexer.py \
  --qdrant-url "https://your-cluster.qdrant.io" \
  --qdrant-api-key "your-key" \
  --openrouter-api-key "sk-or-v1-your-key" \
  --collection "physical-ai-textbook" \
  --directory "docs" \
  --commit-hash "$(git rev-parse HEAD)" \
  --create-collection
```

## Configuration

### Environment Variables

```bash
# Required
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
OPENROUTER_API_KEY=sk-or-v1-your-openrouter-key

# Optional (with defaults)
COLLECTION_NAME=physical-ai-textbook
CHUNK_SIZE=512
CHUNK_OVERLAP=128
```

## Qdrant Schema

### Point Structure

```python
{
    "id": 123456789,
    "vector": [0.123, -0.456, ...],  # 1536 dimensions
    "payload": {
        "chunk_id": "docs/module-1/chapter-3.md:45",
        "content": "Full chunk text...",
        "file_path": "docs/module-1/chapter-3.md",
        "section_h1": "Introduction to ROS2",
        "section_h2": "Core Concepts",
        "section_h3": "Nodes and Topics",
        "line_start": 45,
        "line_end": 67,
        "commit_hash": "abc123def456",
        "token_count": 487,
        "module": "module-1-ros2",
        "chapter": "chapter-3",
        "tags": ["ros", "nodes", "topics"]
    }
}
```

## Performance

**Textbook corpus**: 21 chapters + 5 module indexes + 6 docs = 34 files

- Total chunks: ~700-900
- Avg chunk size: 487 tokens
- Indexing time: ~3-5 minutes (full corpus)
- Storage: ~80-100 MB
- Cost: ~$0.30-0.40 (one-time)

**Re-indexing** (1 chapter): ~10 seconds, ~$0.01

## Error Handling

### Common Errors

**Rate Limit Exceeded**: Reduce batch_size or increase delay

**Qdrant Connection Failed**: Check QDRANT_URL and QDRANT_API_KEY

**Embedding Dimension Mismatch**: Retry the request

### Retry Logic

Automatic retries on:
- Rate limit errors (3 attempts with exponential backoff)
- Timeout errors (3 attempts)
- Network errors

## Reusability

This skill is **fully reusable** for any markdown documentation:

1. Generic Markdown Chunking
2. Configurable parameters
3. Pluggable embeddings
4. Standard interfaces
5. No hardcoded paths

## Related Skills

- **rag-retriever**: Search indexed chunks
- **rag-answerer**: Generate answers with citations
- **rag-manager**: Orchestrate pipeline

## References

- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenRouter API](https://openrouter.ai/docs)
- [ADR-001: RAG Modular Skills Architecture](../../../history/adr/001-rag-modular-skills-architecture.md)
- [ADR-003: Git-Based Incremental Re-indexing](../../../history/adr/003-git-based-incremental-reindexing.md)
