# RAG Retriever Skill

## Summary

The **rag-retriever** skill performs semantic search across indexed book content in Qdrant. It retrieves relevant text chunks using vector similarity, supports hybrid search combining semantic and keyword matching, applies re-ranking algorithms, and returns results with source references and relevance scores.

**Core Function:** Query Qdrant for semantically relevant content chunks with configurable retrieval strategies, filtering, and attribution.

## Inputs

The skill accepts the following inputs:

1. **Query** (required)
   - Natural language search query
   - Can be question or topic description
   - Length: 5-500 words
   - Examples: "What is forward kinematics?", "explain ROS 2 nodes"

2. **Collection Name** (required)
   - Qdrant collection to search (e.g., `physical-ai-textbook`)
   - Must exist in Qdrant
   - Error if collection not found

3. **Top K** (optional)
   - Number of results to return (default: 5)
   - Range: 1-50
   - Higher values = more context but may include noise

4. **Similarity Threshold** (optional)
   - Minimum similarity score (default: 0.7)
   - Range: 0.0-1.0
   - Higher = stricter relevance filtering

5. **Search Mode** (optional)
   - `semantic` - Vector similarity only (default)
   - `hybrid` - Combine vector + keyword search
   - `keyword` - BM25 keyword search only
   - Hybrid typically performs best

6. **Metadata Filters** (optional)
   - Filter by module: `{"module": "module-0-foundations"}`
   - Filter by tags: `{"tags": ["robotics", "kinematics"]}`
   - Filter by chapter: `{"chapter": "chapter-3"}`
   - Combine filters with AND/OR logic

7. **Re-Ranking** (optional)
   - `none` - Use raw similarity scores (default)
   - `cross-encoder` - Use cross-encoder model for re-ranking
   - `mmr` - Maximal Marginal Relevance (diversity)
   - `rrf` - Reciprocal Rank Fusion

8. **Include Metadata** (optional)
   - `true` - Return full metadata (default)
   - `false` - Return only content and score
   - Metadata useful for attribution

9. **Qdrant URL** (optional)
   - Qdrant server URL (default: `http://localhost:6333`)
   - Override environment default

## Outputs

The skill produces:

1. **Ranked Results**
   - Array of matching chunks ordered by relevance
   - Each result contains:
     - **Content**: The actual text chunk
     - **Score**: Similarity score (0.0-1.0)
     - **Rank**: Position in results (1, 2, 3...)
     - **Source Reference**: `file_path:line_numbers`
     - **Metadata**: Module, chapter, headings, tags

2. **Search Summary**
   - Total matches found
   - Results returned (after filtering)
   - Search mode used
   - Query processing time
   - Filters applied

3. **Source Attribution**
   - List of unique source files
   - File path with clickable references
   - Line numbers or section references
   - Module/chapter context

4. **Confidence Indicators**
   - Average similarity score
   - Score distribution (min, max, std dev)
   - Coverage assessment (broad vs narrow match)
   - Result quality rating (high/medium/low)

## Behaviour Rules

### Core Principles

1. **Never Fabricate Content**
   - Only return chunks that exist in indexed collection
   - Do not generate synthetic results
   - Do not modify chunk text in results
   - Report exactly what was retrieved

2. **Deterministic Output**
   - Same query + same collection → same results
   - Consistent ranking with same parameters
   - Reproducible retrieval
   - Stable scoring

3. **Explicit Failure**
   - Fail loudly if collection doesn't exist
   - Report zero results clearly (not error)
   - Log all retrieval errors
   - Never return empty results silently

4. **Source Attribution**
   - Every result MUST include source reference
   - Format: `file_path:start_line-end_line`
   - Enable verification of retrieved content
   - Support traceability

### Retrieval Strategies

**Semantic Search (Vector Similarity):**

1. **Generate Query Embedding**
   - Use same model as indexing
   - Handle query preprocessing
   - Normalize embedding vector

2. **Vector Similarity Search**
   - Cosine similarity (default)
   - Return top_k nearest neighbors
   - Apply similarity threshold filter
   - Sort by descending score

**Hybrid Search (Vector + Keyword):**

1. **Parallel Retrieval**
   - Semantic search → top_k results
   - BM25 keyword search → top_k results
   - Run in parallel for speed

2. **Score Fusion**
   - Normalize scores to [0, 1]
   - Weighted combination (0.7 semantic + 0.3 keyword)
   - Re-rank by combined score
   - Return top_k from merged results

3. **Query Expansion** (optional)
   - Identify key terms in query
   - Generate synonyms/related terms
   - Expand query for better recall
   - Weight original terms higher

**Keyword Search (BM25):**

1. **Tokenization**
   - Extract keywords from query
   - Remove stop words
   - Apply stemming (optional)

2. **BM25 Scoring**
   - Term frequency scoring
   - Inverse document frequency weighting
   - Length normalization
   - Return top matches

### Re-Ranking Methods

**Cross-Encoder Re-Ranking:**

1. **Initial Retrieval**
   - Get top_k × 2 results (over-fetch)
   - Use fast semantic search

2. **Re-Ranking**
   - Pass (query, chunk) pairs to cross-encoder
   - Get precise relevance scores
   - Re-sort by new scores
   - Return top_k final results

**Maximal Marginal Relevance (MMR):**

1. **Diversity Optimization**
   - Select most relevant result first
   - Iteratively add results that are:
     - Relevant to query
     - Diverse from already selected
   - Balance relevance vs diversity
   - Lambda parameter (0.7 = 70% relevance, 30% diversity)

**Reciprocal Rank Fusion (RRF):**

1. **Multi-Strategy Retrieval**
   - Run semantic, keyword, and other searches
   - Get ranked lists from each
   - Combine using RRF formula
   - Boost documents ranked high in multiple lists

### Metadata Filtering

**Filter Types:**

1. **Equality Filters**
   - `module = "module-0-foundations"`
   - `chapter = "chapter-3"`
   - Exact match required

2. **Array Filters**
   - `tags contains "robotics"`
   - `tags contains any ["AI", "ML"]`
   - Partial match in array fields

3. **Range Filters**
   - `chunk_token_count >= 200`
   - `sidebar_position <= 5`
   - Numeric comparisons

4. **Combined Filters**
   - AND: all conditions must match
   - OR: any condition must match
   - NOT: exclude matching results

## When to Use This Skill

Use the **rag-retriever** skill when you need to:

1. **Search Textbook Content**
   - Find relevant sections on a topic
   - Retrieve examples of a concept
   - Locate definitions or explanations
   - Gather context for a question

2. **Build RAG Pipelines**
   - Retrieve context for question answering
   - Find supporting evidence for claims
   - Gather multi-source information
   - Prepare context for LLM prompts

3. **Exploratory Research**
   - Investigate coverage of a topic
   - Find related concepts
   - Discover cross-module connections
   - Identify knowledge gaps

4. **Quality Assurance**
   - Test retrieval quality
   - Validate indexing accuracy
   - Benchmark search relevance
   - Debug RAG system performance

## When NOT to Use This Skill

**DO NOT** use this skill when:

1. **Indexing Tasks**
   - Creating/updating index → Use rag-indexer skill
   - Managing collections → Use rag-manager skill
   - Uploading documents → Use rag-indexer skill

2. **Question Answering**
   - Generating answers → Use rag-answerer skill
   - Synthesizing information → Use rag-answerer skill
   - Creating summaries → Use rag-answerer skill
   - Direct answer required → Use rag-answerer skill

3. **Collection Management**
   - Creating collections → Use rag-manager skill
   - Deleting collections → Use rag-manager skill
   - Viewing stats → Use rag-manager skill
   - Health checks → Use rag-manager skill

4. **Exact Match Searches**
   - Finding specific code → Use grep/search tools
   - Locating file by name → Use glob/find tools
   - Pattern matching → Use regex search

**Rule of Thumb:** Use rag-retriever ONLY for semantic similarity search and retrieval. For answers, use rag-answerer. For indexing, use rag-indexer. For management, use rag-manager.

## Step-by-Step Process

### Phase 1: Query Preparation

1. **Validate Inputs**
   - Check query is non-empty and reasonable length
   - Verify collection name format
   - Validate top_k and threshold ranges
   - Check metadata filter syntax

2. **Verify Collection Exists**
   - Query Qdrant for collection info
   - Error if not found with helpful message
   - Retrieve collection schema (vector size, distance metric)
   - Validate compatibility

3. **Preprocess Query**
   - Clean query text (trim whitespace)
   - Detect language if multilingual
   - Expand abbreviations if configured
   - Prepare for embedding

### Phase 2: Embedding Generation

1. **Generate Query Embedding**
   - Use same embedding model as indexing
   - Call embedding API with query text
   - Validate embedding dimensions match collection
   - Handle API errors gracefully

2. **Cache Embedding** (optional)
   - Hash query text
   - Check cache for repeated queries
   - Store embedding for reuse
   - Expire after TTL

### Phase 3: Retrieval

1. **Execute Search**
   - Based on search_mode, run:
     - Semantic: vector similarity search
     - Hybrid: parallel vector + BM25
     - Keyword: BM25 only
   - Apply metadata filters
   - Set limit to top_k (or top_k × 2 for re-ranking)

2. **Apply Similarity Threshold**
   - Filter results below threshold
   - Log number of results filtered out
   - Warn if all results filtered (threshold too high)

3. **Retrieve Chunk Metadata**
   - Fetch full metadata for each result
   - Extract source references
   - Parse heading hierarchy
   - Collect tags and module info

### Phase 4: Re-Ranking (if enabled)

1. **Select Re-Ranking Method**
   - Cross-encoder: precise relevance scoring
   - MMR: diversity-based selection
   - RRF: multi-strategy fusion
   - None: skip re-ranking

2. **Apply Re-Ranking**
   - Process results according to method
   - Generate new scores or rankings
   - Re-sort results
   - Trim to final top_k

3. **Validate Rankings**
   - Ensure top results are most relevant
   - Check for duplicate chunks
   - Verify score ordering

### Phase 5: Result Preparation

1. **Format Results**
   - Structure each result with:
     - rank (1-based position)
     - content (chunk text)
     - score (similarity or re-ranking score)
     - source (file_path:lines)
     - metadata (module, chapter, headings, tags)

2. **Generate Source References**
   - Convert file paths to clickable references
   - Add line number ranges
   - Create breadcrumb trail (module > chapter > section)
   - Format as: `docs/module-0/chapter-1.md:45-67`

3. **Calculate Confidence Indicators**
   - Average score across top results
   - Score standard deviation
   - Coverage (number of unique sources)
   - Quality rating based on thresholds

### Phase 6: Output Delivery

1. **Present Results**
   - Numbered list of ranked results
   - Content preview (first 200 chars if long)
   - Scores and source references
   - Metadata tags

2. **Provide Search Summary**
   - Total matches found
   - Filters applied
   - Search mode and re-ranking used
   - Processing time

3. **Suggest Follow-Ups** (optional)
   - Related queries
   - Broader/narrower searches
   - Filter adjustments
   - Different search modes

## Example Usage

### Example 1: Basic Semantic Search

**User Request:**
> Search for content about "forward kinematics" in the textbook

**Skill Execution:**
```
Query: "forward kinematics"
Collection: physical-ai-textbook
Mode: semantic
Top K: 5
Threshold: 0.7

Generating query embedding... ✓
Searching collection... ✓

RESULTS (5 matches):

[1] Score: 0.92 | docs/module-1-ros2/chapter-3-kinematics.md:78-95
"Forward kinematics is the process of computing the position and
orientation of the robot's end-effector from given joint angles..."
📍 Module 1 > Chapter 3 > Forward Kinematics > Definition
🏷️ Tags: kinematics, robotics, mathematics

[2] Score: 0.88 | docs/module-1-ros2/chapter-3-kinematics.md:142-168
"To calculate forward kinematics, we use the Denavit-Hartenberg
convention. This method provides a systematic way to describe..."
📍 Module 1 > Chapter 3 > Forward Kinematics > DH Convention
🏷️ Tags: kinematics, DH-parameters, transformation

[3] Score: 0.84 | docs/module-0-foundations/chapter-2-math.md:234-256
"Transformation matrices are essential for forward kinematics
calculations. A transformation matrix combines rotation and..."
📍 Module 0 > Chapter 2 > Transformation Matrices
🏷️ Tags: mathematics, linear-algebra, transformations

[4] Score: 0.79 | docs/module-1-ros2/chapter-3-kinematics.md:201-223
"Example: For a 2-DOF planar robot, forward kinematics gives us
the (x, y) position of the end-effector: x = L1*cos(θ1) + L2*cos..."
📍 Module 1 > Chapter 3 > Worked Examples
🏷️ Tags: kinematics, examples, 2DOF

[5] Score: 0.76 | docs/module-2-digital-twin/chapter-7-simulation.md:89-108
"In simulation, forward kinematics is computed in real-time to
update the robot's visual representation based on joint states..."
📍 Module 2 > Chapter 7 > Simulation Physics
🏷️ Tags: simulation, digital-twin, kinematics

SEARCH SUMMARY:
├─ Total matches: 8 (3 below threshold)
├─ Returned: 5
├─ Avg score: 0.84
├─ Search time: 142ms
└─ Coverage: 3 unique files
```

### Example 2: Hybrid Search with Filters

**User Request:**
> Find content about "sensors" in Module 0 only, use hybrid search

**Skill Execution:**
```
Query: "sensors"
Collection: physical-ai-textbook
Mode: hybrid (semantic + keyword)
Filters: module = "module-0-foundations"
Top K: 3
Threshold: 0.7

Generating query embedding... ✓
Running parallel searches:
  - Semantic search... ✓ (5 candidates)
  - Keyword search (BM25)... ✓ (5 candidates)
Merging and re-ranking... ✓

RESULTS (3 matches):

[1] Score: 0.94 (0.90 semantic + 0.98 keyword)
    docs/module-0-foundations/chapter-1-intro.md:156-178
"Sensors are devices that measure physical properties of the
environment. Common robot sensors include: cameras (vision),
IMUs (orientation), force sensors (touch), and lidar (distance)..."
📍 Module 0 > Chapter 1 > Robot Components > Sensors
🏷️ Tags: sensors, components, perception

[2] Score: 0.87 (0.85 semantic + 0.89 keyword)
    docs/module-0-foundations/chapter-2-sensing.md:12-45
"Chapter 2: Sensing and Perception. This chapter covers the
fundamental types of sensors used in robotics and how to process
their data for perception tasks..."
📍 Module 0 > Chapter 2 > Introduction
🏷️ Tags: sensors, perception, overview

[3] Score: 0.81 (0.78 semantic + 0.84 keyword)
    docs/module-0-foundations/chapter-2-sensing.md:234-267
"Vision sensors (cameras) provide rich information about the
environment. RGB cameras capture color images, while depth
cameras add distance information for each pixel..."
📍 Module 0 > Chapter 2 > Vision Sensors
🏷️ Tags: sensors, cameras, computer-vision

SEARCH SUMMARY:
├─ Mode: Hybrid (70% semantic, 30% keyword)
├─ Filters: module = module-0-foundations
├─ Total candidates: 10 (7 after filtering)
├─ Returned: 3
├─ Avg combined score: 0.87
├─ Search time: 178ms
└─ Coverage: 2 files (both in Module 0)
```

### Example 3: MMR Re-Ranking for Diversity

**User Request:**
> Search for "ROS 2" content with diverse results (not all from same chapter)

**Skill Execution:**
```
Query: "ROS 2"
Collection: physical-ai-textbook
Mode: semantic
Re-ranking: MMR (lambda=0.7)
Top K: 5
Threshold: 0.65

Generating query embedding... ✓
Initial retrieval (over-fetching)... ✓ (10 candidates)
Applying MMR re-ranking for diversity... ✓

RESULTS (5 diverse matches):

[1] Score: 0.95 (relevance) | Diversity: N/A (first pick)
    docs/module-1-ros2/chapter-4-nodes.md:23-51
"ROS 2 (Robot Operating System 2) is a flexible framework for
robot software development. Unlike ROS 1, ROS 2 uses DDS for
communication, making it suitable for real-time systems..."
📍 Module 1 > Chapter 4 > Introduction to ROS 2
🏷️ Tags: ros2, architecture, middleware

[2] Score: 0.89 | Diversity: 0.82 (different chapter, similar topic)
    docs/module-1-ros2/chapter-5-topics.md:67-94
"ROS 2 topics enable publish-subscribe communication. Nodes can
publish messages to topics, and other nodes subscribe to receive
those messages asynchronously..."
📍 Module 1 > Chapter 5 > Topics and Messages
🏷️ Tags: ros2, topics, communication

[3] Score: 0.86 | Diversity: 0.91 (different module!)
    docs/module-2-digital-twin/chapter-8-integration.md:112-145
"Integrating the digital twin with ROS 2 allows bidirectional
communication between simulation and real hardware. Use ros2
bridges to connect the simulator..."
📍 Module 2 > Chapter 8 > ROS 2 Integration
🏷️ Tags: ros2, digital-twin, integration

[4] Score: 0.78 | Diversity: 0.88 (installation/setup topic)
    docs/module-1-ros2/chapter-4-nodes.md:178-203
"To install ROS 2 on Ubuntu 22.04: sudo apt update && sudo apt
install ros-humble-desktop. Source the setup: source /opt/ros/
humble/setup.bash..."
📍 Module 1 > Chapter 4 > Installation
🏷️ Tags: ros2, installation, setup

[5] Score: 0.74 | Diversity: 0.95 (advanced/different context)
    docs/module-3-isaac/chapter-11-ros2-bridge.md:45-72
"NVIDIA Isaac Sim includes a ROS 2 bridge that enables seamless
integration. The bridge publishes sensor data and subscribes to
control commands using standard ROS 2 interfaces..."
📍 Module 3 > Chapter 11 > ROS 2 Bridge
🏷️ Tags: ros2, isaac-sim, simulation

SEARCH SUMMARY:
├─ Re-ranking: MMR (λ=0.7 for 70% relevance, 30% diversity)
├─ Initial candidates: 10
├─ Diverse results: 5 (spanning 3 modules)
├─ Avg relevance: 0.84
├─ Avg diversity: 0.89
├─ Search time: 205ms
└─ Coverage: 4 unique files, 3 modules

💡 Note: MMR selected results from different chapters and modules
   to provide broader coverage of ROS 2 topics rather than
   redundant information from the same section.
```

### Example 4: Zero Results Scenario

**User Request:**
> Search for "quantum computing in robotics"

**Skill Execution:**
```
Query: "quantum computing in robotics"
Collection: physical-ai-textbook
Mode: semantic
Top K: 5
Threshold: 0.7

Generating query embedding... ✓
Searching collection... ✓

⚠️ NO RESULTS FOUND

Search Details:
├─ Total candidates retrieved: 3
├─ After threshold filter (≥0.70): 0
├─ Highest score: 0.64
└─ This topic may not be covered in the textbook

Suggestions:
1. Lower similarity threshold (try 0.5-0.6)
2. Try broader query: "computing in robotics"
3. Use hybrid search mode for better recall
4. Check if topic is covered: use rag-manager to list indexed content

Top matches below threshold (for reference):
[1] Score: 0.64 | docs/module-4-vla-humanoids/chapter-14-ai.md:89-112
    "Advanced computing techniques enable real-time processing..."
[2] Score: 0.58 | docs/module-3-isaac/chapter-10-gpu.md:45-67
    "GPU acceleration is crucial for physics simulation..."
```

## Configuration Requirements

### Environment Variables

```bash
# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key-here  # Optional for Qdrant Cloud

# Embedding API (must match indexing model)
OPENAI_API_KEY=sk-...              # For OpenAI embeddings
COHERE_API_KEY=...                  # For Cohere embeddings

# Optional: Re-ranking
COHERE_API_KEY=...                  # For Cohere re-rank API

# Defaults
RAG_DEFAULT_COLLECTION=physical-ai-textbook
RAG_DEFAULT_TOP_K=5
RAG_SIMILARITY_THRESHOLD=0.7
RAG_SEARCH_MODE=semantic           # semantic | hybrid | keyword
```

### Dependencies

**Python:**
```bash
pip install qdrant-client openai sentence-transformers rank-bm25
```

**Node.js:**
```bash
npm install @qdrant/qdrant-js openai
```

## Error Handling

### Collection Errors

**Error:** Collection not found
```
ERROR: Collection 'physical-ai-textbook' not found in Qdrant

Available collections:
  - test-collection
  - demo-index

Did you mean one of these?
Or run: /rag-indexer to create and populate the collection
```

**Error:** Collection schema mismatch
```
ERROR: Query embedding size (1536) doesn't match collection (384)

Collection 'physical-ai-textbook' uses vector size 384
Your query used model: text-embedding-3-small (1536 dimensions)

Resolution: Use the same embedding model as indexing
Check indexed_with: text-embedding-ada-002 (in collection metadata)
```

### Query Errors

**Error:** Empty query
```
ERROR: Query cannot be empty

Please provide a search query, for example:
  - "What is forward kinematics?"
  - "ROS 2 topics"
  - "humanoid robot sensors"
```

**Error:** Embedding generation failed
```
ERROR: Failed to generate query embedding
Reason: OpenAI API key not set or invalid

Check: OPENAI_API_KEY environment variable
```

### Retrieval Warnings

**Warning:** All results below threshold
```
WARNING: All 8 results were below similarity threshold (0.7)

Highest score: 0.64
Consider:
  1. Lower threshold to 0.5-0.6
  2. Rephrase query with different keywords
  3. Check if topic is covered in indexed content
```

**Warning:** Low result diversity
```
WARNING: All top 5 results are from the same file
File: docs/module-1-ros2/chapter-4-nodes.md

Consider:
  1. Use MMR re-ranking for diversity
  2. Apply metadata filters to search other modules
  3. Query might be very specific to this chapter
```

## Performance Optimization

### Best Practices

1. **Embedding Caching**
   - Cache query embeddings by hash
   - Reuse for identical queries
   - TTL: 1 hour for development, 24 hours for production

2. **Result Caching**
   - Cache search results for common queries
   - Invalidate on index updates
   - LRU eviction policy

3. **Batch Queries**
   - If searching multiple queries, batch embedding generation
   - Parallel searches when independent
   - Combine results efficiently

4. **Optimal Top K**
   - Start with top_k=5 for most queries
   - Use top_k=10-20 for broad topics
   - Over-fetch (2×) if using re-ranking

### Monitoring

Track these metrics:
- Query latency (p50, p95, p99)
- Embedding generation time
- Qdrant search time
- Re-ranking time (if enabled)
- Cache hit rate
- Average result quality (score)

## Version

**Skill Version:** 1.0.0
**Created:** 2025-12-16
**Last Updated:** 2025-12-16
**Compatibility:** Claude Code CLI
**Dependencies:** Qdrant, OpenAI/Cohere (embeddings), optional Cohere (re-ranking)
**Author:** Custom Skill for Physical AI & Humanoid Robotics Textbook Project
