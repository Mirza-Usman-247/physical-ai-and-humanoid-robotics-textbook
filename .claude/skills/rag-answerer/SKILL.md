# RAG Answerer Skill

## Summary

The **rag-answerer** skill answers questions about the Physical AI & Humanoid Robotics textbook using retrieval-augmented generation. It retrieves relevant context using the rag-retriever skill, constructs prompts with retrieved chunks, generates accurate answers with inline citations, and handles multi-hop reasoning when needed.

**Core Function:** Answer user questions about textbook content with cited, grounded responses using retrieved context from Qdrant.

## Inputs

The skill accepts the following inputs:

1. **Question** (required)
   - User's natural language question
   - Can be simple or complex
   - Examples:
     - "What is forward kinematics?"
     - "How do I set up a ROS 2 publisher?"
     - "What are the differences between ROS 1 and ROS 2?"
   - Length: 5-500 words

2. **Collection Name** (required)
   - Qdrant collection to search (e.g., `physical-ai-textbook`)
   - Must exist in Qdrant
   - Error if collection not found

3. **Answer Length** (optional)
   - `brief` - 1-2 sentences (default)
   - `medium` - 1-2 paragraphs
   - `detailed` - 3-5 paragraphs with examples
   - `comprehensive` - Full explanation with code/math

4. **Answer Style** (optional)
   - `technical` - Formal academic tone (default)
   - `beginner-friendly` - Simplified explanations
   - `tutorial` - Step-by-step instructions
   - `reference` - Concise factual statements

5. **Context Chunks** (optional)
   - Number of chunks to retrieve (default: 5)
   - Range: 3-20
   - More chunks = more context but may add noise

6. **Include Examples** (optional)
   - `true` - Include code/math examples from context
   - `false` - Text explanation only (default)
   - Automatically enabled for "how to" questions

7. **Multi-Hop Reasoning** (optional)
   - `true` - Enable iterative retrieval for complex questions
   - `false` - Single retrieval pass (default)
   - Useful for questions requiring multiple concepts

8. **Confidence Threshold** (optional)
   - Minimum confidence to provide answer (default: 0.7)
   - Range: 0.0-1.0
   - Below threshold → "I don't know" response

9. **Citation Style** (optional)
   - `inline` - Inline [1], [2] references (default)
   - `footnote` - Footnote-style citations
   - `source-file` - File path only
   - `none` - No citations (not recommended)

## Outputs

The skill produces:

1. **Answer**
   - Natural language response to question
   - Grounded in retrieved context
   - Appropriate length and style
   - Accurate technical content
   - Never fabricated information

2. **Citations**
   - Inline citations: [1], [2], etc.
   - Mapped to specific source chunks
   - Enable verification of claims
   - Format: `[1] docs/module-1/chapter-3.md:78-95`

3. **Source List**
   - Complete list of cited sources
   - File paths with line ranges
   - Section/chapter context
   - Relevance scores

4. **Confidence Score**
   - Overall confidence in answer (0.0-1.0)
   - Based on:
     - Retrieval quality (avg similarity)
     - Context sufficiency
     - Question-context alignment
   - Interpretation:
     - 0.9-1.0: High confidence
     - 0.7-0.9: Medium confidence
     - <0.7: Low confidence → may return "I don't know"

5. **Context Coverage** (optional)
   - Which parts of question were addressed
   - Missing information indicators
   - Suggestions for follow-up questions

## Behaviour Rules

### Core Principles

1. **Never Fabricate Content**
   - ONLY use information from retrieved context
   - Do not use general knowledge outside indexed content
   - If context insufficient → say "I don't know"
   - Do not invent examples, formulas, or code
   - Explicitly state when information is not available

2. **Deterministic Output**
   - Same question + same context → same answer
   - Reproducible responses
   - Consistent citation mapping
   - Stable confidence scores

3. **Explicit Failure**
   - Fail loudly if collection doesn't exist
   - Report insufficient context clearly
   - Log retrieval errors
   - Never guess or hallucinate

4. **Mandatory Attribution**
   - Every claim MUST have citation
   - Map citations to specific chunks
   - Include source file and line numbers
   - Enable verification

### Answer Construction

**Step-by-Step Process:**

1. **Retrieve Context**
   - Use rag-retriever skill to get relevant chunks
   - Apply filters if question mentions specific module/chapter
   - Ensure sufficient context (min 3 chunks above threshold)
   - Log retrieval quality metrics

2. **Assess Context Sufficiency**
   - Check if retrieved chunks address the question
   - Verify relevance scores are acceptable
   - Identify information gaps
   - Decide: answer, partial answer, or "I don't know"

3. **Construct Answer Prompt**
   - Inject retrieved chunks verbatim (no paraphrasing)
   - Number chunks for citation mapping
   - Provide clear instructions:
     - Use ONLY provided context
     - Cite every claim with [N]
     - Say "I don't know" if context insufficient
     - Match requested length and style

4. **Generate Answer**
   - Use LLM to generate response from prompt
   - Enforce citation requirements
   - Match length and style preferences
   - Ensure technical accuracy

5. **Validate Answer**
   - Verify all citations map to actual chunks
   - Check no fabricated content
   - Ensure answer addresses question
   - Validate technical correctness

6. **Format Output**
   - Present answer with inline citations
   - List sources below answer
   - Provide confidence score
   - Add follow-up suggestions if applicable

### Multi-Hop Reasoning

**For Complex Questions:**

Some questions require multiple retrieval steps:

**Example:** "How do I integrate ROS 2 with Isaac Sim for a humanoid robot?"

**Multi-Hop Process:**

1. **First Retrieval**
   - Query: "ROS 2 Isaac Sim integration"
   - Retrieve integration context
   - Identify sub-topics: ROS 2 bridge, Isaac setup

2. **Second Retrieval** (if needed)
   - Query: "Isaac Sim ROS 2 bridge setup"
   - Retrieve specific setup instructions
   - Combine with previous context

3. **Third Retrieval** (if needed)
   - Query: "humanoid robot in Isaac Sim"
   - Retrieve humanoid-specific info
   - Integrate all contexts

4. **Synthesize Answer**
   - Combine information from all retrievals
   - Maintain citations across retrievals
   - Structure answer logically
   - Cover all aspects of question

**When to Use Multi-Hop:**
- Question has multiple sub-questions
- Question requires connecting concepts from different modules
- Initial retrieval doesn't fully address question
- Question includes "and", "also", "differences between"

### "I Don't Know" Policy

**Trigger "I don't know" response when:**

1. **Insufficient Context**
   - Retrieved chunks don't address question
   - All similarity scores below threshold
   - Context is tangentially related but not sufficient

2. **Out of Scope**
   - Question about topics not in textbook
   - Question requires external knowledge
   - Question about future/speculative content

3. **Ambiguous Context**
   - Retrieved chunks contradict each other
   - Context is unclear or incomplete
   - Can't confidently synthesize answer

**"I Don't Know" Response Format:**
```
I don't have sufficient information in the indexed textbook content
to answer this question confidently.

Retrieved context summary:
- Found 3 chunks related to [partial topic]
- Highest relevance score: 0.62 (below threshold 0.70)
- Missing information about: [specific gaps]

Suggestions:
1. Try rephrasing: [suggested rephrase]
2. This topic may be covered in: [suggested modules/chapters]
3. Check table of contents for relevant sections
```

### Citation Mapping

**Inline Citation Format:**

Answer text with citations: "Forward kinematics is the process of computing the end-effector position from joint angles [1]. It uses transformation matrices [2] and the Denavit-Hartenberg convention [1, 3]."

**Source List:**
```
Sources:
[1] docs/module-1-ros2/chapter-3-kinematics.md:78-95
    Module 1 > Chapter 3 > Forward Kinematics > Definition
    Score: 0.92

[2] docs/module-0-foundations/chapter-2-math.md:234-256
    Module 0 > Chapter 2 > Transformation Matrices
    Score: 0.84

[3] docs/module-1-ros2/chapter-3-kinematics.md:142-168
    Module 1 > Chapter 3 > Forward Kinematics > DH Convention
    Score: 0.88
```

**Citation Requirements:**
- Every factual claim has citation
- Citations are accurate (map to correct chunks)
- Citations are minimal (don't over-cite)
- Citations are in order ([1] before [2])

## When to Use This Skill

Use the **rag-answerer** skill when you need to:

1. **Answer User Questions**
   - Direct questions about textbook content
   - Conceptual explanations
   - How-to instructions
   - Comparisons and differences

2. **Educational Support**
   - Student homework help (grounded in textbook)
   - Concept clarification
   - Quick reference lookups
   - Study guide generation

3. **Content Verification**
   - Check what textbook says about a topic
   - Verify claims with citations
   - Find authoritative definitions
   - Validate understanding

4. **Interactive Learning**
   - Q&A chatbot for textbook
   - Intelligent tutoring system
   - Context-aware help system
   - Adaptive learning paths

## When NOT to Use This Skill

**DO NOT** use this skill when:

1. **Indexing Tasks**
   - Creating/updating index → Use rag-indexer skill
   - Managing collections → Use rag-manager skill

2. **Just Retrieval Needed**
   - Need raw chunks without answer → Use rag-retriever skill
   - Building your own prompt → Use rag-retriever skill
   - Testing retrieval quality → Use rag-retriever skill

3. **General Questions**
   - Questions beyond textbook scope → Use general knowledge
   - Real-time information → Use web search
   - Opinions or subjective answers → Not appropriate

4. **Code Execution**
   - Running code → Use appropriate execution tools
   - Testing examples → Use sandbox/REPL
   - Debugging → Use debug tools

**Rule of Thumb:** Use rag-answerer when you need a GENERATED ANSWER with CITATIONS from indexed content. For retrieval only, use rag-retriever. For indexing, use rag-indexer.

## Step-by-Step Process

### Phase 1: Question Analysis

1. **Parse Question**
   - Identify question type (what, how, why, compare)
   - Extract key entities (ROS 2, kinematics, sensors)
   - Detect if multi-part question
   - Classify complexity (simple, moderate, complex)

2. **Determine Strategy**
   - Simple question → single retrieval
   - Complex question → multi-hop retrieval
   - How-to question → include examples
   - Comparison question → retrieve both concepts

3. **Validate Collection**
   - Check collection exists
   - Verify collection is accessible
   - Confirm collection has indexed content

### Phase 2: Context Retrieval

1. **Invoke rag-retriever Skill**
   - Pass question as query
   - Set top_k based on question complexity
   - Apply metadata filters if question is module-specific
   - Use hybrid search for better recall

2. **Evaluate Retrieved Context**
   - Check number of chunks retrieved
   - Assess average relevance score
   - Verify chunks address the question
   - Determine if sufficient for answering

3. **Multi-Hop Retrieval** (if enabled and needed)
   - Identify information gaps
   - Formulate follow-up queries
   - Retrieve additional context
   - Merge contexts intelligently

### Phase 3: Answer Generation

1. **Construct Prompt**
   - Template structure:
     ```
     You are answering a question about the Physical AI & Humanoid Robotics textbook.
     Use ONLY the provided context chunks. Cite every claim with [N].
     If context is insufficient, say "I don't know".

     Question: {question}

     Context:
     [1] {chunk_1_text}
     [2] {chunk_2_text}
     ...
     [N] {chunk_n_text}

     Answer length: {brief|medium|detailed|comprehensive}
     Answer style: {technical|beginner-friendly|tutorial|reference}
     Include examples: {yes|no}

     Answer:
     ```

2. **Generate Answer**
   - Call LLM with constructed prompt
   - Use appropriate model (GPT-4, Claude, etc.)
   - Set temperature low (0.1-0.3) for factual accuracy
   - Enforce citation requirements

3. **Validate Answer**
   - Check all citations exist in context
   - Verify no fabricated information
   - Ensure answer addresses question
   - Validate length matches request

### Phase 4: Citation Processing

1. **Map Citations to Sources**
   - Extract citation numbers from answer
   - Map to chunk IDs from retrieval
   - Get file paths and line numbers
   - Build source reference list

2. **Format Source List**
   - Number sources to match citations
   - Include file paths with line ranges
   - Add section/chapter breadcrumbs
   - Include relevance scores

3. **Validate Citation Accuracy**
   - Ensure citation numbers are sequential
   - Check no broken references
   - Verify source mappings are correct

### Phase 5: Confidence Assessment

1. **Calculate Confidence Score**
   - Base score from avg retrieval similarity
   - Adjust for context sufficiency
   - Adjust for question complexity
   - Adjust for citation density

2. **Determine Answer Quality**
   - High (0.9-1.0): Excellent context, clear answer
   - Medium (0.7-0.9): Good context, reasonable answer
   - Low (<0.7): Poor context, should return "I don't know"

3. **Apply Threshold**
   - If score < threshold → "I don't know" response
   - Otherwise → return answer with confidence

### Phase 6: Output Delivery

1. **Format Final Response**
   - Answer with inline citations
   - Source list below answer
   - Confidence score
   - Optional follow-up suggestions

2. **Add Context Metadata** (optional)
   - Which modules/chapters were used
   - Coverage assessment
   - Related topics for further reading

3. **Suggest Follow-Ups** (optional)
   - Related questions user might ask
   - Deeper dive suggestions
   - Prerequisite concepts if needed

## Example Usage

### Example 1: Simple Definitional Question

**User Question:**
> What is forward kinematics?

**Skill Execution:**
```
Question: "What is forward kinematics?"
Collection: physical-ai-textbook
Answer length: brief
Answer style: technical
Context chunks: 5

Step 1: Analyzing question... ✓
  - Type: Definitional (what is)
  - Complexity: Simple
  - Strategy: Single retrieval

Step 2: Retrieving context via rag-retriever... ✓
  - Retrieved: 5 chunks
  - Avg score: 0.89
  - Top source: module-1-ros2/chapter-3-kinematics.md

Step 3: Generating answer... ✓

Step 4: Validating citations... ✓
  - All citations mapped
  - No fabricated content

Step 5: Confidence assessment... ✓
  - Score: 0.91 (High confidence)

───────────────────────────────────────────────────────

ANSWER:

Forward kinematics is the process of computing the position and
orientation of a robot's end-effector from given joint angles [1].
It uses transformation matrices to calculate the final pose based
on the robot's kinematic structure [2].

Confidence: 0.91 (High)

Sources:
[1] docs/module-1-ros2/chapter-3-kinematics.md:78-95
    Module 1 > Chapter 3 > Forward Kinematics > Definition
    Score: 0.92

[2] docs/module-0-foundations/chapter-2-math.md:234-256
    Module 0 > Chapter 2 > Transformation Matrices
    Score: 0.84
```

### Example 2: How-To Question with Examples

**User Question:**
> How do I create a ROS 2 publisher in Python?

**Skill Execution:**
```
Question: "How do I create a ROS 2 publisher in Python?"
Collection: physical-ai-textbook
Answer length: detailed
Answer style: tutorial
Include examples: true (auto-detected "how to")
Context chunks: 8

Step 1: Analyzing question... ✓
  - Type: How-to/Tutorial
  - Complexity: Moderate
  - Strategy: Single retrieval with examples

Step 2: Retrieving context via rag-retriever... ✓
  - Retrieved: 8 chunks (including code examples)
  - Avg score: 0.87
  - Top sources: module-1-ros2 chapters

Step 3: Generating answer with code examples... ✓

───────────────────────────────────────────────────────

ANSWER:

To create a ROS 2 publisher in Python, follow these steps [1]:

1. **Import required modules** [1]:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String
   ```

2. **Create a node class** [1, 2]:
   ```python
   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
   ```
   The `create_publisher()` method takes three arguments: message type,
   topic name, and queue size [2].

3. **Publish messages** [1, 3]:
   Use a timer to publish periodically:
   ```python
   timer_period = 0.5  # seconds
   self.timer = self.create_timer(timer_period, self.timer_callback)

   def timer_callback(self):
       msg = String()
       msg.data = 'Hello World'
       self.publisher_.publish(msg)
   ```

4. **Initialize and spin** [1, 4]:
   ```python
   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()
   ```

The publisher will now continuously publish String messages to the
specified topic at 2 Hz [3].

Confidence: 0.88 (High)

Sources:
[1] docs/module-1-ros2/chapter-5-topics.md:145-203
    Module 1 > Chapter 5 > Creating Publishers > Python Example
    Score: 0.94

[2] docs/module-1-ros2/chapter-5-topics.md:67-94
    Module 1 > Chapter 5 > Publisher API
    Score: 0.87

[3] docs/module-1-ros2/chapter-5-topics.md:215-234
    Module 1 > Chapter 5 > Timer Callbacks
    Score: 0.83

[4] docs/module-1-ros2/chapter-4-nodes.md:123-145
    Module 1 > Chapter 4 > Node Lifecycle
    Score: 0.79
```

### Example 3: Complex Multi-Hop Question

**User Question:**
> What are the main differences between ROS 1 and ROS 2 for humanoid robot development?

**Skill Execution:**
```
Question: "What are the main differences between ROS 1 and ROS 2
          for humanoid robot development?"
Collection: physical-ai-textbook
Answer length: comprehensive
Answer style: technical
Multi-hop reasoning: true (auto-detected comparison + specific domain)
Context chunks: 15

Step 1: Analyzing question... ✓
  - Type: Comparison + Domain-specific
  - Complexity: Complex (multi-hop needed)
  - Strategy: Multiple retrievals

Step 2: Multi-hop retrieval... ✓

  Retrieval 1: "ROS 1 vs ROS 2 differences"
    - Retrieved: 5 chunks (score: 0.91 avg)

  Retrieval 2: "ROS 2 humanoid robot development"
    - Retrieved: 5 chunks (score: 0.84 avg)

  Retrieval 3: "ROS 1 humanoid robots limitations"
    - Retrieved: 5 chunks (score: 0.78 avg)

  Total context: 15 unique chunks merged

Step 3: Generating comprehensive answer... ✓

───────────────────────────────────────────────────────

ANSWER:

ROS 1 and ROS 2 differ significantly in ways that impact humanoid
robot development:

**Architecture & Communication** [1, 2]:
ROS 2 uses DDS (Data Distribution Service) middleware instead of
ROS 1's custom TCP/IP-based communication [1]. This provides
real-time capabilities crucial for humanoid balance control and
fast reflexes [2]. Unlike ROS 1's centralized roscore, ROS 2 is
fully distributed, improving reliability for multi-process humanoid
systems [1, 3].

**Real-Time Performance** [2, 4]:
ROS 2 supports real-time execution with RT-safe code paths, essential
for humanoid joint control running at 1kHz+ [4]. ROS 1 lacks
deterministic timing guarantees, making precise motor control
challenging [2]. For humanoids, this means ROS 2 can handle
time-critical tasks like balance recovery and force control [4, 5].

**Multi-Robot Support** [3, 6]:
ROS 2's native multi-robot capabilities enable better coordination
for humanoid teams [6]. The DDS discovery mechanism allows humanoids
to find each other automatically, unlike ROS 1's manual configuration
[3]. This is particularly relevant for multi-humanoid scenarios and
human-robot collaboration [6, 7].

**Security** [8]:
ROS 2 includes authentication and encryption, important for humanoids
in public spaces or handling sensitive tasks [8]. ROS 1 has no
built-in security, requiring custom solutions [8].

**Humanoid-Specific Advantages** [5, 7, 9]:
- **Whole-body control**: ROS 2's real-time support enables complex
  controllers managing 20+ DOF simultaneously [5, 9]
- **Sensor fusion**: Better timing control for fusing IMU, vision,
  and force sensors [7]
- **Safety**: Reliable emergency stop and fault tolerance [9]

**Migration Consideration** [10]:
While ROS 1 is still widely used in existing humanoid platforms, new
development should target ROS 2 for its superior real-time and
reliability features [10]. The ROS 1 bridge allows gradual migration
for legacy systems [10, 11].

Confidence: 0.86 (High)

Sources:
[1] docs/module-1-ros2/chapter-4-nodes.md:23-51
    Module 1 > Chapter 4 > ROS 2 Architecture
    Score: 0.95

[2] docs/module-1-ros2/chapter-4-nodes.md:189-212
    Module 1 > Chapter 4 > Real-Time Capabilities
    Score: 0.91

[3] docs/module-1-ros2/chapter-6-distributed.md:45-78
    Module 1 > Chapter 6 > Distributed Systems
    Score: 0.89

... [8 more sources]

Follow-up suggestions:
- How do I set up real-time ROS 2 for a humanoid?
- What are the best practices for whole-body control in ROS 2?
- How can I migrate a ROS 1 humanoid project to ROS 2?
```

### Example 4: "I Don't Know" Response

**User Question:**
> How do I integrate quantum sensors with my humanoid robot?

**Skill Execution:**
```
Question: "How do I integrate quantum sensors with my humanoid robot?"
Collection: physical-ai-textbook
Answer length: medium
Context chunks: 5

Step 1: Analyzing question... ✓
  - Type: How-to
  - Complexity: Moderate
  - Strategy: Single retrieval

Step 2: Retrieving context via rag-retriever... ✓
  - Retrieved: 3 chunks
  - Avg score: 0.52 (below threshold 0.70)
  - Top score: 0.58

Step 3: Assessing context sufficiency... ⚠️
  - Context does not adequately address question
  - Retrieved chunks are tangentially related but insufficient

───────────────────────────────────────────────────────

ANSWER:

I don't have sufficient information in the indexed textbook content
to answer this question confidently.

Retrieved context summary:
- Found 3 chunks related to "sensors" and "integration"
- Highest relevance score: 0.58 (below threshold 0.70)
- Missing information about: quantum sensors specifically

The textbook covers conventional robot sensors (cameras, IMUs,
force sensors, lidar) but does not appear to include content on
quantum sensors.

Suggestions:
1. Try asking about standard sensors: "What sensors are used in
   humanoid robots?"
2. Check if this advanced topic is in later chapters (not yet indexed)
3. This may be beyond the scope of this textbook

Confidence: 0.52 (Low - insufficient context)

Partial context retrieved:
[1] docs/module-0-foundations/chapter-2-sensing.md:234-267
    "Vision sensors (cameras) provide rich information..." (Score: 0.58)

[2] docs/module-1-ros2/chapter-7-sensors.md:89-112
    "Integrating sensors with ROS 2 requires..." (Score: 0.51)
```

## Configuration Requirements

### Environment Variables

```bash
# Qdrant (for rag-retriever dependency)
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key-here

# Embedding API (for rag-retriever dependency)
OPENAI_API_KEY=sk-...
COHERE_API_KEY=...

# LLM for Answer Generation
OPENAI_API_KEY=sk-...              # For GPT-4, GPT-3.5
ANTHROPIC_API_KEY=...               # For Claude models
COHERE_API_KEY=...                  # For Cohere Command

# Defaults
RAG_DEFAULT_COLLECTION=physical-ai-textbook
RAG_ANSWER_LENGTH=brief            # brief | medium | detailed | comprehensive
RAG_ANSWER_STYLE=technical         # technical | beginner-friendly | tutorial | reference
RAG_CONFIDENCE_THRESHOLD=0.7
RAG_CONTEXT_CHUNKS=5
RAG_LLM_TEMPERATURE=0.2            # Low temperature for factual accuracy
```

### Dependencies

**Required Skills:**
- rag-retriever skill (must be available)

**Python:**
```bash
pip install qdrant-client openai anthropic tiktoken
```

**Node.js:**
```bash
npm install @qdrant/qdrant-js openai @anthropic-ai/sdk
```

## Error Handling

### Retrieval Errors

**Error:** Collection not found
```
ERROR: Cannot answer question - collection 'physical-ai-textbook' not found

Run: /rag-indexer to create and populate the collection first
```

**Error:** Retrieval failed
```
ERROR: Failed to retrieve context for question

Details: rag-retriever skill returned error
Reason: Qdrant server not responding

Check: Qdrant is running and accessible
```

### Answer Generation Errors

**Error:** LLM API failure
```
ERROR: Failed to generate answer

Reason: OpenAI API rate limit exceeded
Retry: Waiting 10 seconds and retrying...
```

**Error:** No context retrieved
```
WARNING: No relevant context found for question

Retrieved: 0 chunks above similarity threshold
Cannot generate answer without context

Response: "I don't know" (insufficient context)
```

### Validation Errors

**Error:** Citation validation failed
```
WARNING: Generated answer contains invalid citations

Issue: Citation [5] not found in retrieved context (only 4 chunks)
Action: Re-generating answer with corrected prompt...
```

**Error:** Answer appears fabricated
```
WARNING: Answer validation detected potential fabrication

Issue: Answer includes information not present in retrieved chunks
Action: Returning "I don't know" instead of potentially false answer
```

## Performance Optimization

### Best Practices

1. **Cache Answers**
   - Hash question text
   - Cache answer + citations + confidence
   - Invalidate on index updates
   - TTL: 24 hours

2. **Optimize Context Size**
   - Start with 5 chunks for simple questions
   - Use 8-15 for complex questions
   - Don't over-fetch (diminishing returns)

3. **Smart Multi-Hop**
   - Only enable for complex questions
   - Limit to 2-3 hops max
   - Cache intermediate retrievals

4. **LLM Selection**
   - Use GPT-3.5-turbo for simple questions (faster, cheaper)
   - Use GPT-4 for complex multi-hop questions (better reasoning)
   - Use Claude for very long context (>10 chunks)

### Monitoring

Track these metrics:
- End-to-end latency
- Retrieval time vs generation time
- Confidence score distribution
- "I don't know" rate (should be <10% for good index)
- Citation accuracy (manual review)
- User satisfaction (if collected)

## Version

**Skill Version:** 1.0.0
**Created:** 2025-12-16
**Last Updated:** 2025-12-16
**Compatibility:** Claude Code CLI
**Dependencies:** rag-retriever skill, Qdrant, OpenAI/Anthropic/Cohere
**Author:** Custom Skill for Physical AI & Humanoid Robotics Textbook Project
