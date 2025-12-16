# RAG Suite Usage Examples & Integration Guide

Comprehensive examples and integration patterns for the RAG Suite skills.

## Table of Contents

1. [Basic Workflows](#basic-workflows)
2. [Advanced Use Cases](#advanced-use-cases)
3. [Integration Patterns](#integration-patterns)
4. [API Examples](#api-examples)
5. [Automation Scripts](#automation-scripts)
6. [Best Practices](#best-practices)

---

## Basic Workflows

### Workflow 1: First-Time Setup

Complete workflow from scratch to answering questions:

```bash
# Step 1: Start Qdrant (if not running)
docker start qdrant
# or
docker run -d -p 6333:6333 --name qdrant qdrant/qdrant

# Step 2: Verify environment
python3 verify_setup.py

# Step 3: Create collection using rag-manager skill
# In Claude Code:
/rag-manager create \
  --collection physical-ai-textbook \
  --vector-size 1536 \
  --distance cosine

# Step 4: Index textbook content using rag-indexer skill
/rag-indexer \
  --collection physical-ai-textbook \
  --docs-path ./docs \
  --chunk-size 512 \
  --chunk-overlap 128 \
  --embedding-model text-embedding-3-small

# Expected: ~3-5 minutes for 21 chapters (34 files total)

# Step 5: Verify indexing
/rag-manager info --collection physical-ai-textbook

# Step 6: Test retrieval
/rag-retriever "What is forward kinematics?" \
  --collection physical-ai-textbook \
  --top-k 5 \
  --search-mode semantic

# Step 7: Ask your first question
/rag-answerer "How do I create a ROS 2 publisher in Python?" \
  --collection physical-ai-textbook \
  --answer-length detailed \
  --include-examples true

# Success! Your RAG system is ready.
```

### Workflow 2: Daily Content Updates

Update index when textbook content changes:

```bash
# Step 1: Pull latest changes
git pull origin main

# Step 2: Incremental re-index (only new/modified files)
/rag-indexer \
  --collection physical-ai-textbook \
  --incremental true \
  --docs-path ./docs

# Step 3: Verify update
/rag-manager info --collection physical-ai-textbook
# Check "Last indexed" timestamp

# Step 4: Test updated content
/rag-answerer "What's new in the latest chapter?"
```

### Workflow 3: Collection Maintenance

Weekly maintenance routine:

```bash
# Step 1: Health check
/rag-manager health --collection physical-ai-textbook

# Step 2: Clean up stale content (if any issues found)
/rag-manager cleanup \
  --collection physical-ai-textbook \
  --mode stale \
  --age-threshold 90

# Step 3: Remove duplicates
/rag-manager cleanup \
  --collection physical-ai-textbook \
  --mode duplicates

# Step 4: Optimize for performance
/rag-manager optimize --collection physical-ai-textbook

# Step 5: Backup
/rag-manager export \
  --collection physical-ai-textbook \
  --path ./backups/weekly-backup-$(date +%Y-%m-%d).snapshot
```

---

## Advanced Use Cases

### Use Case 1: Multi-Version Documentation

Maintain separate collections for different textbook versions:

```bash
# Create collections for different versions
/rag-manager create --collection textbook-v1-0 --vector-size 1536
/rag-manager create --collection textbook-v2-0 --vector-size 1536
/rag-manager create --collection textbook-dev --vector-size 1536

# Index each version
/rag-indexer --collection textbook-v1-0 --docs-path ./versions/v1.0/docs
/rag-indexer --collection textbook-v2-0 --docs-path ./versions/v2.0/docs
/rag-indexer --collection textbook-dev --docs-path ./docs

# Query specific version
/rag-answerer "How do I use ROS 2?" \
  --collection textbook-v2-0 \
  --answer-length brief

# Compare across versions
/rag-retriever "ROS 2 setup" --collection textbook-v1-0
/rag-retriever "ROS 2 setup" --collection textbook-v2-0
```

### Use Case 2: Module-Specific Collections

Create specialized collections per module:

```bash
# Create module-specific collections
/rag-manager create --collection module-0-foundations --vector-size 1536
/rag-manager create --collection module-1-ros2 --vector-size 1536
/rag-manager create --collection module-2-digital-twin --vector-size 1536
/rag-manager create --collection module-3-isaac --vector-size 1536
/rag-manager create --collection module-4-vla-humanoids --vector-size 1536

# Index each module separately
/rag-indexer --collection module-0-foundations \
  --docs-path ./docs/module-0-foundations

/rag-indexer --collection module-1-ros2 \
  --docs-path ./docs/module-1-ros2

# ... repeat for other modules

# Query specific module
/rag-answerer "What are the prerequisites?" \
  --collection module-0-foundations

# Or use metadata filters on main collection (more efficient)
/rag-retriever "ROS 2 topics" \
  --collection physical-ai-textbook \
  --filters '{"module": "module-1-ros2"}'
```

### Use Case 3: Multilingual Support

Support English and Urdu content:

```bash
# Create language-specific collections
/rag-manager create --collection textbook-en --vector-size 1536
/rag-manager create --collection textbook-ur --vector-size 1536

# Index English content
/rag-indexer --collection textbook-en --docs-path ./docs/en

# Index Urdu content (requires Urdu embedding model)
/rag-indexer --collection textbook-ur \
  --docs-path ./docs/ur \
  --embedding-model multilingual-e5-base

# Query in specific language
/rag-answerer "What is forward kinematics?" \
  --collection textbook-en \
  --answer-style beginner-friendly

# Urdu query
/rag-answerer "Forward kinematics کیا ہے؟" \
  --collection textbook-ur \
  --answer-style beginner-friendly
```

### Use Case 4: Intelligent Tutoring System

Build an adaptive learning system:

```python
# tutoring_system.py
from qdrant_client import QdrantClient
from openai import OpenAI
import json

class IntelligentTutor:
    def __init__(self, collection="physical-ai-textbook"):
        self.qdrant = QdrantClient(url="http://localhost:6333")
        self.openai = OpenAI()
        self.collection = collection
        self.conversation_history = []

    def ask_question(self, question, student_level="beginner"):
        """Answer with adaptive difficulty"""

        # 1. Retrieve context
        context_chunks = self.retrieve_context(question, top_k=5)

        # 2. Assess student level from history
        if len(self.conversation_history) > 3:
            student_level = self.assess_level(self.conversation_history)

        # 3. Generate answer adapted to level
        answer = self.generate_adaptive_answer(
            question, context_chunks, student_level
        )

        # 4. Generate follow-up questions
        follow_ups = self.generate_follow_ups(question, answer, student_level)

        # 5. Update conversation history
        self.conversation_history.append({
            "question": question,
            "answer": answer,
            "level": student_level
        })

        return {
            "answer": answer,
            "sources": [c['source'] for c in context_chunks],
            "follow_ups": follow_ups,
            "level": student_level
        }

    def retrieve_context(self, query, top_k=5):
        """Use rag-retriever logic"""
        # Generate embedding
        embedding = self.openai.embeddings.create(
            input=query,
            model="text-embedding-3-small"
        ).data[0].embedding

        # Search Qdrant
        results = self.qdrant.search(
            collection_name=self.collection,
            query_vector=embedding,
            limit=top_k
        )

        return [
            {
                "content": hit.payload['content'],
                "source": f"{hit.payload['file_path']}:{hit.payload.get('line_start', 0)}",
                "score": hit.score
            }
            for hit in results
        ]

    def generate_adaptive_answer(self, question, context, level):
        """Generate answer adapted to student level"""

        # Different prompts for different levels
        level_prompts = {
            "beginner": "Explain in simple terms, avoid jargon, use analogies",
            "intermediate": "Use technical terms but explain them, include examples",
            "advanced": "Assume technical knowledge, focus on depth and nuances"
        }

        context_text = "\n\n".join([
            f"[{i+1}] {chunk['content']}"
            for i, chunk in enumerate(context)
        ])

        prompt = f"""You are an intelligent tutor for a Physical AI & Humanoid Robotics textbook.

Student Level: {level}
Instruction: {level_prompts.get(level, level_prompts['intermediate'])}

Use ONLY the following context to answer. Cite sources with [N].

Context:
{context_text}

Question: {question}

Answer:"""

        response = self.openai.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )

        return response.choices[0].message.content

    def assess_level(self, history):
        """Assess student level from conversation history"""
        # Simple heuristic: check question complexity and answer understanding
        recent = history[-3:]
        avg_question_length = sum(len(h['question'].split()) for h in recent) / len(recent)

        if avg_question_length > 15:
            return "advanced"
        elif avg_question_length > 8:
            return "intermediate"
        else:
            return "beginner"

    def generate_follow_ups(self, question, answer, level):
        """Generate progressive follow-up questions"""
        # Use LLM to generate relevant follow-ups
        prompt = f"""Based on this Q&A, suggest 3 progressive follow-up questions
for a {level}-level student:

Question: {question}
Answer: {answer[:500]}...

Follow-up questions (numbered list):"""

        response = self.openai.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.7,
            max_tokens=200
        )

        return response.choices[0].message.content

# Usage
tutor = IntelligentTutor()

result = tutor.ask_question(
    "What is forward kinematics?",
    student_level="beginner"
)

print(result['answer'])
print("\nSources:", result['sources'])
print("\nFollow-up questions:", result['follow_ups'])
```

### Use Case 5: Homework Helper with Citation Validation

Ensure students cite sources properly:

```python
# homework_helper.py

class HomeworkHelper:
    def __init__(self):
        self.tutor = IntelligentTutor()

    def help_with_assignment(self, assignment_question):
        """Help student with assignment, require citations"""

        print(f"Assignment: {assignment_question}\n")

        # Get answer with sources
        result = self.tutor.ask_question(
            assignment_question,
            student_level="intermediate"
        )

        # Format as homework-ready answer
        answer = f"""
Question: {assignment_question}

Answer:
{result['answer']}

References:
{chr(10).join(f"{i+1}. {src}" for i, src in enumerate(result['sources']))}

---
Note: This answer is grounded in the textbook. Please rephrase in your own words
and add your understanding. Always cite the textbook sections you used.
"""

        return answer

    def validate_citation(self, student_answer, claimed_source):
        """Verify student actually used the cited source"""

        # Retrieve the claimed source
        context = self.tutor.retrieve_context(
            claimed_source,
            top_k=1
        )

        if not context:
            return {
                "valid": False,
                "message": "Source not found in textbook"
            }

        # Check if student answer aligns with source
        source_content = context[0]['content']

        # Use LLM to check alignment
        prompt = f"""Does this student answer appropriately use information from the source?

Source:
{source_content}

Student Answer:
{student_answer}

Respond with:
- "VALID" if the answer correctly uses the source
- "INVALID" if the answer misrepresents or doesn't use the source
- "PARTIAL" if the answer uses some but not all relevant information

Explanation (one sentence):"""

        response = self.tutor.openai.chat.completions.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2
        )

        result = response.choices[0].message.content
        is_valid = result.startswith("VALID")

        return {
            "valid": is_valid,
            "message": result
        }

# Usage
helper = HomeworkHelper()

# Help with assignment
answer = helper.help_with_assignment(
    "Explain the difference between forward and inverse kinematics"
)
print(answer)

# Validate student's citation
validation = helper.validate_citation(
    student_answer="Forward kinematics computes end-effector position from joint angles.",
    claimed_source="docs/module-1-ros2/chapter-3-kinematics.md"
)
print(f"\nCitation valid: {validation['valid']}")
print(f"Reason: {validation['message']}")
```

---

## Integration Patterns

### Pattern 1: Chatbot Integration

Integrate RAG into a Discord/Slack bot:

```python
# discord_bot.py
import discord
from discord.ext import commands
from intelligent_tutor import IntelligentTutor

bot = commands.Bot(command_prefix='!')
tutor = IntelligentTutor()

@bot.command(name='ask')
async def ask_question(ctx, *, question):
    """Ask a question about the textbook"""

    # Show typing indicator
    async with ctx.typing():
        # Get answer from RAG
        result = tutor.ask_question(question)

        # Format for Discord
        embed = discord.Embed(
            title="📚 Textbook Answer",
            description=result['answer'],
            color=discord.Color.blue()
        )

        # Add sources
        sources_text = "\n".join(result['sources'][:3])  # Top 3
        embed.add_field(
            name="📖 Sources",
            value=sources_text,
            inline=False
        )

        # Add follow-ups
        embed.add_field(
            name="💡 Follow-up Questions",
            value=result['follow_ups'],
            inline=False
        )

        # Add confidence
        embed.set_footer(text=f"Confidence: {result.get('confidence', 'N/A')}")

        await ctx.send(embed=embed)

@bot.command(name='search')
async def search_textbook(ctx, *, query):
    """Search the textbook for relevant sections"""

    async with ctx.typing():
        # Use rag-retriever
        results = tutor.retrieve_context(query, top_k=5)

        embed = discord.Embed(
            title=f"🔍 Search Results for: {query}",
            color=discord.Color.green()
        )

        for i, result in enumerate(results, 1):
            embed.add_field(
                name=f"{i}. {result['source']} (Score: {result['score']:.2f})",
                value=result['content'][:200] + "...",
                inline=False
            )

        await ctx.send(embed=embed)

bot.run('YOUR_DISCORD_TOKEN')
```

### Pattern 2: Web API Service

Expose RAG as a REST API:

```python
# api_server.py
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from intelligent_tutor import IntelligentTutor
import uvicorn

app = FastAPI(title="Physical AI Textbook RAG API")
tutor = IntelligentTutor()

class QuestionRequest(BaseModel):
    question: str
    collection: str = "physical-ai-textbook"
    answer_length: str = "medium"
    student_level: str = "intermediate"
    include_follow_ups: bool = True

class SearchRequest(BaseModel):
    query: str
    collection: str = "physical-ai-textbook"
    top_k: int = 5
    filters: dict = None

@app.post("/api/v1/ask")
async def ask_question(request: QuestionRequest):
    """Answer a question about the textbook"""
    try:
        result = tutor.ask_question(
            request.question,
            student_level=request.student_level
        )
        return {
            "success": True,
            "data": result
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/v1/search")
async def search(request: SearchRequest):
    """Search the textbook"""
    try:
        results = tutor.retrieve_context(
            request.query,
            top_k=request.top_k
        )
        return {
            "success": True,
            "data": {
                "results": results,
                "count": len(results)
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/health")
async def health_check():
    """API health check"""
    return {"status": "healthy", "version": "1.0.0"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

Test the API:
```bash
# Start server
python api_server.py

# Test ask endpoint
curl -X POST http://localhost:8000/api/v1/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is forward kinematics?",
    "answer_length": "brief",
    "student_level": "beginner"
  }'

# Test search endpoint
curl -X POST http://localhost:8000/api/v1/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "ROS 2 topics",
    "top_k": 5
  }'
```

### Pattern 3: Docusaurus Plugin Integration

Add RAG search to Docusaurus site:

```javascript
// src/theme/SearchBar/index.js
import React, { useState } from 'react';
import styles from './styles.module.css';

export default function SearchBar() {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState([]);
  const [loading, setLoading] = useState(false);

  const handleSearch = async (e) => {
    e.preventDefault();
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/v1/search', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: query,
          top_k: 5
        })
      });

      const data = await response.json();
      setResults(data.data.results);
    } catch (error) {
      console.error('Search error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.searchContainer}>
      <form onSubmit={handleSearch}>
        <input
          type="text"
          placeholder="Search textbook with AI..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className={styles.searchInput}
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Searching...' : 'Search'}
        </button>
      </form>

      {results.length > 0 && (
        <div className={styles.results}>
          <h3>Results:</h3>
          {results.map((result, idx) => (
            <div key={idx} className={styles.resultItem}>
              <a href={`/${result.source.split(':')[0]}`}>
                <strong>{result.source}</strong>
                <span className={styles.score}>Score: {result.score.toFixed(2)}</span>
              </a>
              <p>{result.content.substring(0, 200)}...</p>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
```

---

## Automation Scripts

### Script 1: Automated Daily Re-Index

```bash
#!/bin/bash
# daily_reindex.sh

set -e

LOG_FILE="./logs/reindex-$(date +%Y-%m-%d).log"
COLLECTION="physical-ai-textbook"

echo "Starting daily re-index: $(date)" | tee -a $LOG_FILE

# Pull latest content
echo "Pulling latest content..." | tee -a $LOG_FILE
git pull origin main >> $LOG_FILE 2>&1

# Incremental re-index
echo "Running incremental re-index..." | tee -a $LOG_FILE
python3 scripts/rag_indexer.py \
  --collection $COLLECTION \
  --incremental \
  --docs-path ./docs \
  >> $LOG_FILE 2>&1

# Health check
echo "Running health check..." | tee -a $LOG_FILE
python3 scripts/rag_manager.py \
  health \
  --collection $COLLECTION \
  >> $LOG_FILE 2>&1

# If health check fails, alert
if [ $? -ne 0 ]; then
  echo "Health check failed! Sending alert..." | tee -a $LOG_FILE
  # Send email/Slack notification
  curl -X POST https://hooks.slack.com/services/YOUR/WEBHOOK/URL \
    -d "{\"text\": \"RAG health check failed for $COLLECTION\"}"
fi

echo "Daily re-index complete: $(date)" | tee -a $LOG_FILE
```

Schedule with cron:
```bash
# Run daily at 2 AM
crontab -e

# Add:
0 2 * * * /path/to/daily_reindex.sh
```

### Script 2: Weekly Backup and Cleanup

```bash
#!/bin/bash
# weekly_maintenance.sh

set -e

COLLECTION="physical-ai-textbook"
BACKUP_DIR="./backups"
DATE=$(date +%Y-%m-%d)

echo "Starting weekly maintenance: $(date)"

# Create backup
echo "Creating backup..."
mkdir -p $BACKUP_DIR
python3 scripts/rag_manager.py export \
  --collection $COLLECTION \
  --path $BACKUP_DIR/backup-$DATE.snapshot

# Cleanup stale data
echo "Cleaning up stale data..."
python3 scripts/rag_manager.py cleanup \
  --collection $COLLECTION \
  --mode stale \
  --age-threshold 90

# Remove duplicates
echo "Removing duplicates..."
python3 scripts/rag_manager.py cleanup \
  --collection $COLLECTION \
  --mode duplicates

# Optimize collection
echo "Optimizing collection..."
python3 scripts/rag_manager.py optimize \
  --collection $COLLECTION

# Remove old backups (keep last 4 weeks)
echo "Cleaning old backups..."
find $BACKUP_DIR -name "backup-*.snapshot" -mtime +28 -delete

echo "Weekly maintenance complete: $(date)"
```

---

## Best Practices

### 1. Chunking Strategy

**DO:**
- Use semantic-aware chunking (by headings/paragraphs)
- Maintain 10-25% overlap between chunks
- Keep code blocks intact
- Preserve context with heading hierarchy

**DON'T:**
- Split in middle of sentences
- Use fixed character counts without considering structure
- Remove metadata during chunking
- Create chunks < 50 tokens or > 2048 tokens

### 2. Embedding Selection

**DO:**
- Use same model for indexing and retrieval
- Choose model based on your language (multilingual if needed)
- Test multiple models on sample queries
- Consider cost vs quality tradeoffs

**DON'T:**
- Mix embedding models in same collection
- Use outdated or deprecated models
- Ignore dimension mismatches
- Forget to track which model was used

### 3. Retrieval Optimization

**DO:**
- Start with semantic search
- Add hybrid search for technical queries
- Use re-ranking for critical applications
- Apply metadata filters when possible

**DON'T:**
- Always fetch max results (over-retrieve)
- Ignore similarity thresholds
- Skip result validation
- Forget to cite sources

### 4. Answer Generation

**DO:**
- Inject context verbatim (no paraphrasing)
- Require citations for all claims
- Return "I don't know" for insufficient context
- Validate answers against sources

**DON'T:**
- Fabricate information beyond context
- Skip citation mapping
- Ignore confidence scores
- Generate without retrieved context

### 5. Maintenance

**DO:**
- Run health checks regularly
- Clean up stale content monthly
- Backup before destructive operations
- Monitor costs and performance

**DON'T:**
- Skip incremental re-indexing
- Ignore warnings from health checks
- Let collections grow without optimization
- Forget to track collection versions

---

## Testing & Validation

### Test Suite

```python
# test_rag_suite.py
import pytest
from intelligent_tutor import IntelligentTutor

@pytest.fixture
def tutor():
    return IntelligentTutor(collection="test-collection")

def test_retrieval_quality(tutor):
    """Test that retrieval returns relevant results"""
    results = tutor.retrieve_context("forward kinematics", top_k=5)

    assert len(results) == 5
    assert all(r['score'] > 0.7 for r in results)
    assert all('kinematics' in r['content'].lower() for r in results[:3])

def test_answer_has_citations(tutor):
    """Test that answers include citations"""
    result = tutor.ask_question("What is forward kinematics?")

    assert '[1]' in result['answer']
    assert len(result['sources']) > 0
    assert all(source.endswith('.md') for source in result['sources'])

def test_idk_response(tutor):
    """Test I don't know for out-of-scope questions"""
    result = tutor.ask_question("What is quantum computing?")

    assert any(phrase in result['answer'].lower() for phrase in [
        "don't know",
        "don't have",
        "insufficient",
        "not found"
    ])

def test_multilingual_retrieval(tutor):
    """Test retrieval works for different languages"""
    # English
    en_results = tutor.retrieve_context("robot sensors")
    assert len(en_results) > 0

    # Urdu (if multilingual collection)
    ur_results = tutor.retrieve_context("روبوٹ سینسر")
    assert len(ur_results) > 0

# Run tests
pytest.main([__file__, '-v'])
```

---

## Performance Benchmarks

Expected performance metrics:

| Operation | Latency (p50) | Latency (p95) | Notes |
|-----------|---------------|---------------|-------|
| Embedding generation | 50ms | 150ms | OpenAI API |
| Vector search (1k points) | 5ms | 15ms | Local Qdrant |
| Vector search (100k points) | 20ms | 50ms | Local Qdrant |
| Answer generation | 2s | 5s | GPT-4 |
| Full indexing (21 chapters, 34 files) | 3min | 5min | First-time |
| Incremental update | 10s | 30s | 5 modified files |

---

## Conclusion

This RAG Suite provides a complete solution for building intelligent Q&A systems on top of your Docusaurus textbook. Explore the examples above, adapt them to your needs, and build amazing educational experiences!

For support:
- Review individual SKILL.md files
- Check SETUP.md for configuration
- Open issues on GitHub
- Join community Discord

Happy building! 🚀
