# RAG Manager Skill

## Summary

The **rag-manager** skill manages Qdrant collections and RAG system operations. It handles collection lifecycle (create, delete, update), monitors collection health and statistics, performs maintenance operations (cleanup, optimization), and provides diagnostic tools for troubleshooting RAG pipelines.

**Core Function:** Manage Qdrant collections, monitor system health, perform maintenance, and provide RAG system diagnostics.

## Inputs

The skill accepts operation-specific inputs:

### Collection Operations

1. **Operation Type** (required)
   - `create` - Create new collection
   - `delete` - Delete collection
   - `list` - List all collections
   - `info` - Get collection details
   - `update` - Update collection settings
   - `clone` - Clone existing collection
   - `export` - Export collection data
   - `import` - Import collection data
   - `health` - Health check and diagnostics
   - `cleanup` - Clean up stale/orphaned data
   - `optimize` - Optimize collection performance

2. **Collection Name** (required for most operations)
   - Target collection name
   - Must exist for: delete, info, update, export, health
   - Must not exist for: create (unless force=true)

3. **Qdrant URL** (optional)
   - Qdrant server URL (default: `http://localhost:6333`)
   - Override environment default

### Create-Specific Inputs

4. **Vector Size** (required for create)
   - Dimension of embedding vectors
   - Examples: 384, 768, 1536
   - Must match embedding model

5. **Distance Metric** (optional for create)
   - `cosine` - Cosine similarity (default, recommended)
   - `euclidean` - Euclidean distance
   - `dot` - Dot product

6. **Indexing Parameters** (optional for create)
   - `hnsw_m` - Number of edges per node (default: 16)
   - `hnsw_ef_construct` - Depth of search during construction (default: 100)
   - Higher values = better accuracy, slower indexing

### Update-Specific Inputs

7. **Optimization Settings** (optional for update)
   - `indexing_threshold` - When to start indexing (default: 20000)
   - `memmap_threshold` - When to use memory mapping (default: 50000)

### Export/Import Inputs

8. **Export Path** (required for export)
   - File path for exported data
   - Format: JSON or binary snapshot

9. **Import Path** (required for import)
   - File path containing import data
   - Must be compatible format

### Cleanup Inputs

10. **Cleanup Mode** (optional for cleanup)
    - `stale` - Remove points older than threshold
    - `orphaned` - Remove points without valid metadata
    - `duplicates` - Remove duplicate chunks
    - `all` - All cleanup operations

11. **Age Threshold** (optional for stale cleanup)
    - Days since last_indexed (default: 90)
    - Points older than this are considered stale

## Outputs

The skill produces operation-specific outputs:

### List Operation Output

```
Available Collections:

1. physical-ai-textbook
   ├─ Vectors: 1,247 points
   ├─ Size: 1536 dimensions
   ├─ Distance: Cosine
   ├─ Status: ✓ Healthy
   └─ Indexed: 2025-12-15

2. test-collection
   ├─ Vectors: 42 points
   ├─ Size: 384 dimensions
   ├─ Distance: Cosine
   ├─ Status: ⚠️ Small dataset
   └─ Indexed: 2025-12-10

3. demo-index
   ├─ Vectors: 523 points
   ├─ Size: 768 dimensions
   ├─ Distance: Euclidean
   ├─ Status: ✓ Healthy
   └─ Indexed: 2025-12-14

Total: 3 collections, 1,812 points
```

### Info Operation Output

```
Collection: physical-ai-textbook

Configuration:
├─ Vector size: 1536
├─ Distance metric: Cosine
├─ Vectors count: 1,247
├─ Indexed vectors: 1,247
├─ Points count: 1,247
├─ Segments count: 3
├─ Status: Green

Storage:
├─ Disk usage: 24.3 MB
├─ RAM usage: 8.7 MB
├─ Indexing threshold: 20,000
├─ Memmap threshold: 50,000

Metadata:
├─ Created: 2025-12-15T10:23:45Z
├─ Last indexed: 2025-12-15T14:30:12Z
├─ Embedding model: text-embedding-3-small
├─ Chunk size: 512 tokens
├─ Chunk overlap: 128 tokens
├─ Total files indexed: 42
├─ Version: 1.0.0

Performance:
├─ Avg search latency: 12ms (p95: 28ms)
├─ Index build time: 3m 42s
├─ Optimization status: ✓ Optimized

Health:
├─ Status: ✓ Healthy
├─ All vectors indexed
├─ No orphaned points
└─ No detected issues
```

### Health Check Output

```
Health Check: physical-ai-textbook

Overall Status: ✓ Healthy

Checks:
├─ [✓] Collection exists and accessible
├─ [✓] All vectors are indexed
├─ [✓] No orphaned metadata
├─ [✓] Segment count is optimal (3)
├─ [✓] Disk usage is reasonable (24.3 MB)
├─ [⚠️] RAM usage high (8.7 MB, consider memmap)
└─ [✓] No detected corruption

Recommendations:
1. Consider enabling memmap for RAM optimization
2. Run optimization if search performance degrades

Search Quality Test:
├─ Sample query: "forward kinematics"
├─ Results returned: 5
├─ Top score: 0.92
└─ Status: ✓ Good quality

Last indexed: 2 hours ago
Next recommended re-index: In 22 hours (daily schedule)
```

### Cleanup Operation Output

```
Cleanup Operation: physical-ai-textbook

Mode: stale + orphaned + duplicates
Age threshold: 90 days

Scanning collection... ✓

Results:
├─ Total points scanned: 1,247
├─ Stale points (>90 days): 23 removed
├─ Orphaned points: 0 removed
├─ Duplicate chunks: 5 removed
├─ Total removed: 28 points
└─ Remaining points: 1,219

Storage reclaimed: 1.8 MB

Removed files:
- docs/old-chapter-draft.md (deleted from source, 12 chunks)
- docs/deprecated-module.md (stale, 11 chunks)
- [5 duplicate chunks across various files]

Collection is now optimized.
Run /rag-indexer in incremental mode to add back any needed content.
```

## Behaviour Rules

### Core Principles

1. **Never Fabricate Data**
   - Report actual collection states
   - Don't assume collection configurations
   - Query Qdrant for real-time information
   - Don't simulate responses

2. **Deterministic Operations**
   - Same inputs → same outputs
   - Reproducible operations
   - Consistent reporting format

3. **Explicit Failure**
   - Fail loudly on errors
   - Provide clear error messages
   - Suggest remediation steps
   - Never silently fail

4. **Safe Destructive Operations**
   - Confirm before delete operations
   - Provide dry-run mode for cleanup
   - Backup recommendations before destructive ops
   - Allow undo/restore where possible

### Collection Lifecycle Management

**Creating Collections:**

1. **Validation**
   - Check collection name is valid (alphanumeric, hyphens, underscores)
   - Verify collection doesn't exist (unless force=true)
   - Validate vector_size > 0
   - Confirm distance metric is supported

2. **Creation**
   - Create collection with specified parameters
   - Set up optimal indexing parameters
   - Initialize metadata fields
   - Verify creation success

3. **Post-Creation**
   - Report collection details
   - Provide indexing command for next step
   - Set up monitoring if configured

**Deleting Collections:**

1. **Pre-Delete Checks**
   - Verify collection exists
   - Check if collection has data
   - Warn about data loss

2. **Confirmation** (unless force=true)
   - Require explicit confirmation
   - Show collection info (size, points count)
   - Suggest export before delete

3. **Deletion**
   - Delete collection from Qdrant
   - Clean up associated metadata
   - Verify deletion success
   - Report final status

**Updating Collections:**

1. **Non-Destructive Updates**
   - Update optimization parameters
   - Modify indexing thresholds
   - Adjust performance settings
   - Apply without data loss

2. **Destructive Updates**
   - Changing vector_size requires recreation
   - Changing distance metric requires recreation
   - Provide migration path
   - Backup and restore workflow

### Health Monitoring

**Health Check Components:**

1. **Connectivity**
   - Qdrant server reachable
   - Collection accessible
   - API responding normally

2. **Data Integrity**
   - All vectors indexed
   - No orphaned points
   - No corruption detected
   - Metadata completeness

3. **Performance**
   - Search latency within bounds
   - Index optimization status
   - Memory usage reasonable
   - Segment count optimal

4. **Freshness**
   - Time since last indexing
   - Stale content detection
   - Version tracking

**Diagnostics:**

- Run sample queries
- Check result quality
- Verify similarity scores
- Test metadata filtering

### Maintenance Operations

**Cleanup:**

1. **Stale Data Removal**
   - Identify points older than threshold
   - Check if source files still exist
   - Remove points from deleted files
   - Update statistics

2. **Orphaned Data Removal**
   - Find points without metadata
   - Find points with invalid file_paths
   - Remove or fix orphaned points
   - Report findings

3. **Duplicate Detection**
   - Hash chunk content
   - Identify duplicate chunks
   - Keep highest quality version
   - Remove duplicates

**Optimization:**

1. **Index Optimization**
   - Rebuild HNSW index
   - Merge segments
   - Compact storage
   - Update statistics

2. **Performance Tuning**
   - Adjust indexing parameters
   - Enable memmap if needed
   - Optimize segment count
   - Clear caches

## When to Use This Skill

Use the **rag-manager** skill when you need to:

1. **Collection Lifecycle**
   - Create new collections for different modules/versions
   - Delete old or test collections
   - List all available collections
   - Clone collections for testing

2. **Monitoring & Diagnostics**
   - Check collection health
   - View collection statistics
   - Debug retrieval issues
   - Verify indexing success

3. **Maintenance**
   - Clean up stale indexed content
   - Remove orphaned data
   - Optimize collection performance
   - Prepare for re-indexing

4. **Data Management**
   - Export collections for backup
   - Import collections from backup
   - Migrate between environments
   - Version control for collections

5. **Troubleshooting**
   - Diagnose poor search quality
   - Investigate slow queries
   - Debug missing results
   - Verify configuration

## When NOT to Use This Skill

**DO NOT** use this skill when:

1. **Indexing Content**
   - Adding documents → Use rag-indexer skill
   - Updating content → Use rag-indexer skill
   - Re-indexing → Use rag-indexer skill

2. **Searching**
   - Finding content → Use rag-retriever skill
   - Answering questions → Use rag-answerer skill
   - Testing retrieval → Use rag-retriever skill

3. **Qdrant Server Management**
   - Starting Qdrant → Use docker/systemd
   - Configuring Qdrant → Edit config files
   - Upgrading Qdrant → Use package manager

**Rule of Thumb:** Use rag-manager for OPERATIONS ON COLLECTIONS (create, delete, monitor, maintain). For content operations, use rag-indexer. For queries, use rag-retriever or rag-answerer.

## Step-by-Step Process

### Create Collection

1. **Validate Inputs**
   - Check collection name format
   - Verify vector_size is positive integer
   - Validate distance metric
   - Check Qdrant connectivity

2. **Check Existence**
   - Query Qdrant for collection
   - If exists and force=false → error
   - If exists and force=true → warn and delete first

3. **Create Collection**
   - Call Qdrant API with parameters
   - Set HNSW indexing configuration
   - Initialize metadata schema
   - Verify creation

4. **Report Success**
   - Show collection details
   - Suggest next step: `/rag-indexer to populate`
   - Provide example query command

### Delete Collection

1. **Verify Existence**
   - Check collection exists
   - Error if not found

2. **Gather Info**
   - Get collection stats (points count, size)
   - Prepare summary for confirmation

3. **Request Confirmation** (unless force=true)
   - Show collection info
   - Warn about data loss
   - Suggest export before delete
   - Wait for user confirmation

4. **Delete**
   - Call Qdrant delete API
   - Verify deletion
   - Report success

### List Collections

1. **Query Qdrant**
   - Get all collections
   - Fetch stats for each

2. **Format Output**
   - Numbered list
   - Key stats per collection
   - Health indicators
   - Total summary

3. **Provide Actions**
   - Suggest `info` for details
   - Suggest `health` for diagnostics

### Info/Stats

1. **Fetch Collection Info**
   - Configuration parameters
   - Vector and points count
   - Storage usage
   - Metadata

2. **Calculate Metrics**
   - Disk and RAM usage
   - Indexing percentage
   - Health indicators

3. **Run Sample Query** (optional)
   - Test search functionality
   - Measure latency
   - Check result quality

4. **Format Report**
   - Structured output
   - Clear sections
   - Visual indicators
   - Actionable recommendations

### Health Check

1. **Connectivity Test**
   - Ping Qdrant server
   - Access collection
   - Verify API response

2. **Data Integrity Checks**
   - Count indexed vs total vectors
   - Check for orphaned points
   - Verify metadata completeness
   - Detect corruption

3. **Performance Tests**
   - Run sample queries
   - Measure latency
   - Check result quality
   - Verify scoring

4. **Generate Report**
   - Overall health status
   - Individual check results
   - Recommendations
   - Next actions

### Cleanup

1. **Dry Run** (recommended)
   - Scan for stale/orphaned/duplicate data
   - Report what would be removed
   - Get user confirmation

2. **Stale Data Cleanup**
   - Find points older than threshold
   - Verify source files don't exist
   - Remove stale points
   - Track removed count

3. **Orphaned Data Cleanup**
   - Find points without metadata
   - Find points with invalid file_paths
   - Remove or fix
   - Report findings

4. **Duplicate Cleanup**
   - Hash all chunk content
   - Identify duplicates
   - Keep best version (highest score)
   - Remove others

5. **Report Results**
   - Total scanned
   - Removed by category
   - Storage reclaimed
   - Recommendations

### Optimize

1. **Analyze Collection**
   - Check segment count
   - Measure search performance
   - Identify optimization opportunities

2. **Run Optimizations**
   - Rebuild index if needed
   - Merge segments
   - Compact storage
   - Update metadata

3. **Verify Improvements**
   - Re-measure performance
   - Confirm optimization success
   - Report metrics before/after

## Example Usage

### Example 1: Create New Collection

**User Request:**
> Create a new Qdrant collection for the textbook with OpenAI embeddings

**Skill Execution:**
```
Operation: create
Collection name: physical-ai-textbook
Vector size: 1536 (text-embedding-3-small)
Distance metric: cosine

Validating inputs... ✓
Checking Qdrant connectivity... ✓
Checking if collection exists... ✓ (doesn't exist)

Creating collection with configuration:
├─ Name: physical-ai-textbook
├─ Vector size: 1536
├─ Distance: Cosine
├─ HNSW parameters:
│  ├─ m: 16
│  ├─ ef_construct: 100
│  └─ full_scan_threshold: 10000

Creating... ✓

COLLECTION CREATED SUCCESSFULLY

Collection: physical-ai-textbook
├─ Vector size: 1536 dimensions
├─ Distance metric: Cosine similarity
├─ Status: ✓ Ready (empty)
├─ Capacity: Unlimited
└─ Created: 2025-12-16T15:30:00Z

Next steps:
1. Index content: /rag-indexer --collection physical-ai-textbook
2. Verify: /rag-manager info --collection physical-ai-textbook
3. Test search: /rag-retriever "test query"
```

### Example 2: List All Collections

**User Request:**
> Show me all Qdrant collections

**Skill Execution:**
```
Operation: list

Fetching collections from Qdrant... ✓

AVAILABLE COLLECTIONS:

1. physical-ai-textbook
   ├─ Vectors: 1,247 points (1536-dim)
   ├─ Distance: Cosine
   ├─ Storage: 24.3 MB (disk), 8.7 MB (RAM)
   ├─ Status: ✓ Healthy
   ├─ Last indexed: 2 hours ago
   └─ Files: 42 documents

2. test-embeddings
   ├─ Vectors: 42 points (384-dim)
   ├─ Distance: Cosine
   ├─ Storage: 856 KB (disk), 312 KB (RAM)
   ├─ Status: ⚠️ Small dataset
   ├─ Last indexed: 6 days ago
   └─ Files: 3 documents

3. archived-v1-0
   ├─ Vectors: 1,103 points (768-dim)
   ├─ Distance: Euclidean
   ├─ Storage: 18.2 MB (disk), 6.1 MB (RAM)
   ├─ Status: ⚠️ Stale (indexed 45 days ago)
   ├─ Last indexed: 45 days ago
   └─ Files: 38 documents

SUMMARY:
├─ Total collections: 3
├─ Total vectors: 2,392 points
├─ Total storage: 43.4 MB (disk), 15.1 MB (RAM)
├─ Healthy: 1
├─ Warnings: 2 (small dataset, stale)
└─ Errors: 0

Actions:
- View details: /rag-manager info --collection <name>
- Health check: /rag-manager health --collection <name>
- Cleanup stale: /rag-manager cleanup --collection archived-v1-0
```

### Example 3: Health Check with Issues

**User Request:**
> Check the health of my RAG collection

**Skill Execution:**
```
Operation: health
Collection: physical-ai-textbook

Running health checks... ✓

HEALTH CHECK REPORT

Overall Status: ⚠️ Warnings Detected

Connectivity:
├─ [✓] Qdrant server reachable (12ms latency)
├─ [✓] Collection accessible
└─ [✓] API responding normally

Data Integrity:
├─ [✓] All vectors indexed (1,247/1,247)
├─ [⚠️] Found 12 orphaned points (missing file_path metadata)
├─ [✓] No corruption detected
└─ [⚠️] 5 duplicate chunks detected

Performance:
├─ [✓] Search latency: 15ms avg (p95: 32ms) - Good
├─ [⚠️] RAM usage high: 12.4 MB (consider memmap)
├─ [✓] Segment count optimal: 3
└─ [✓] Index optimization: Up to date

Freshness:
├─ [✓] Last indexed: 3 hours ago
├─ [⚠️] Found 8 stale points (>90 days old)
└─ [✓] Version tracking: 1.0.0

Search Quality Test:
├─ Query: "forward kinematics"
├─ Results: 5 returned
├─ Top score: 0.92
├─ Avg score: 0.84
└─ Status: ✓ Excellent quality

RECOMMENDATIONS:

1. [High Priority] Run cleanup to fix orphaned points:
   /rag-manager cleanup --collection physical-ai-textbook --mode orphaned

2. [Medium] Remove stale content (8 points >90 days):
   /rag-manager cleanup --collection physical-ai-textbook --mode stale

3. [Medium] Remove 5 duplicate chunks:
   /rag-manager cleanup --collection physical-ai-textbook --mode duplicates

4. [Low] Consider enabling memmap for RAM optimization:
   /rag-manager update --collection physical-ai-textbook --memmap-threshold 10000

5. [Info] Re-index recommended in: 21 hours (daily schedule)

Estimated cleanup impact:
├─ Points to remove: 25 (12 orphaned + 8 stale + 5 duplicates)
├─ Storage to reclaim: ~2.1 MB
└─ Collection will remain healthy (1,222 points remaining)
```

### Example 4: Cleanup Operation

**User Request:**
> Clean up stale and duplicate data from my collection

**Skill Execution:**
```
Operation: cleanup
Collection: physical-ai-textbook
Mode: stale + duplicates
Age threshold: 90 days
Dry run: false (will actually remove data)

⚠️  WARNING: This will permanently remove data from the collection
Files affected: 5 files
Points to remove: ~13 (estimated)

Proceed? [y/N]: y

Starting cleanup operation...

Phase 1: Scanning collection
├─ Total points: 1,247
├─ Scanning for stale data (>90 days)... ✓
│  └─ Found: 8 stale points
├─ Scanning for duplicates... ✓
│  └─ Found: 5 duplicate chunks
└─ Total to remove: 13 points

Phase 2: Stale data removal
├─ Checking source files...
│  ├─ docs/old-draft-chapter.md → ✗ File deleted from source (5 chunks)
│  ├─ docs/deprecated-intro.md → ✗ Marked as deprecated (3 chunks)
│  └─ Total stale: 8 chunks
├─ Removing stale points... ✓
└─ Removed: 8 points

Phase 3: Duplicate removal
├─ Analyzing chunk content hashes...
│  ├─ Hash: a7b3c9... → 2 copies (keeping highest score 0.89)
│  ├─ Hash: e4f2d1... → 2 copies (keeping highest score 0.91)
│  └─ Hash: c9a1b8... → 2 copies (keeping highest score 0.88)
├─ Removing duplicates... ✓
└─ Removed: 5 points

Phase 4: Finalizing
├─ Optimizing collection... ✓
├─ Updating statistics... ✓
└─ Verifying integrity... ✓

CLEANUP COMPLETE

Summary:
├─ Points scanned: 1,247
├─ Stale points removed: 8
├─ Duplicate points removed: 5
├─ Total removed: 13 points
├─ Remaining points: 1,234
├─ Storage reclaimed: 1.9 MB
└─ Duration: 4.2 seconds

Removed content details:
📄 docs/old-draft-chapter.md (5 chunks, deleted from source)
📄 docs/deprecated-intro.md (3 chunks, >90 days old)
📋 5 duplicate chunks across various files

Collection status: ✓ Healthy and optimized

Recommendations:
1. Run /rag-indexer --incremental to add any new content
2. Consider scheduling regular cleanup (weekly/monthly)
3. Next health check in: 24 hours
```

### Example 5: Export Collection for Backup

**User Request:**
> Export my collection for backup before making changes

**Skill Execution:**
```
Operation: export
Collection: physical-ai-textbook
Export path: ./backups/physical-ai-textbook-2025-12-16.snapshot

Preparing export...

Collection info:
├─ Name: physical-ai-textbook
├─ Points: 1,247
├─ Size: 1536 dimensions
└─ Estimated export size: ~25 MB

Exporting...
├─ Exporting vectors... [=========>] 100% (1,247/1,247)
├─ Exporting metadata... ✓
├─ Exporting configuration... ✓
└─ Compressing... ✓

EXPORT COMPLETE

Export details:
├─ File: ./backups/physical-ai-textbook-2025-12-16.snapshot
├─ Size: 18.3 MB (compressed from ~25 MB)
├─ Points: 1,247
├─ Format: Qdrant snapshot v1.0
├─ Checksum: sha256:a7b3c9d2e1f4...
└─ Created: 2025-12-16T16:45:23Z

To restore this backup:
/rag-manager import --collection physical-ai-textbook-restored \\
  --path ./backups/physical-ai-textbook-2025-12-16.snapshot

Backup verified: ✓ All data intact
```

## Configuration Requirements

### Environment Variables

```bash
# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key-here  # Optional for Qdrant Cloud

# Defaults
RAG_DEFAULT_VECTOR_SIZE=1536       # OpenAI text-embedding-3-small
RAG_DEFAULT_DISTANCE=cosine        # cosine | euclidean | dot
RAG_CLEANUP_AGE_THRESHOLD=90       # Days for stale data
RAG_BACKUP_DIR=./backups           # Default backup location
```

### Dependencies

**Python:**
```bash
pip install qdrant-client
```

**Node.js:**
```bash
npm install @qdrant/qdrant-js
```

**System:**
- Qdrant running (local or cloud)
- Sufficient disk space for exports

## Error Handling

### Collection Errors

**Error:** Collection not found
```
ERROR: Collection 'physical-ai-textbook' not found

Available collections:
  - test-collection
  - demo-index

Create collection:
  /rag-manager create --collection physical-ai-textbook --vector-size 1536
```

**Error:** Collection already exists
```
ERROR: Collection 'physical-ai-textbook' already exists

Options:
1. Use a different name
2. Delete existing: /rag-manager delete --collection physical-ai-textbook
3. Use --force to overwrite (will delete existing data!)
```

### Connectivity Errors

**Error:** Cannot connect to Qdrant
```
ERROR: Failed to connect to Qdrant at http://localhost:6333

Troubleshooting:
1. Check Qdrant is running: docker ps | grep qdrant
2. Start Qdrant: docker run -p 6333:6333 qdrant/qdrant
3. Verify URL is correct in environment
4. Check firewall settings
```

### Operation Errors

**Error:** Cleanup failed partially
```
WARNING: Cleanup completed with errors

Successfully removed: 10/13 points
Failed removals: 3 points
Reason: Permission denied (read-only mode?)

Partial success:
├─ Stale: 8/8 removed ✓
├─ Duplicates: 2/5 removed ⚠️
└─ Storage reclaimed: 1.2 MB

Recommendations:
1. Check Qdrant permissions
2. Retry failed operations
3. Collection remains functional
```

## Performance Optimization

### Best Practices

1. **Regular Maintenance**
   - Run health checks weekly
   - Cleanup monthly
   - Optimize quarterly
   - Export backups before major changes

2. **Collection Sizing**
   - Use memmap for collections >50k points
   - Adjust HNSW parameters for collection size
   - Monitor RAM usage trends

3. **Backup Strategy**
   - Export before destructive operations
   - Keep weekly backups
   - Test restore procedures
   - Automate backup schedules

4. **Monitoring**
   - Track collection growth
   - Monitor search latency trends
   - Alert on health check failures
   - Log all operations

### Monitoring Metrics

Track these metrics:
- Collection point count over time
- Storage usage (disk and RAM)
- Search latency (p50, p95, p99)
- Cleanup efficiency (points removed)
- Health check pass rate
- Indexing freshness

## Version

**Skill Version:** 1.0.0
**Created:** 2025-12-16
**Last Updated:** 2025-12-16
**Compatibility:** Claude Code CLI
**Dependencies:** Qdrant
**Author:** Custom Skill for Physical AI & Humanoid Robotics Textbook Project
