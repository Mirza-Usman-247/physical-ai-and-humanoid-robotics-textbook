"""
Answer generation using LLM with inline citations.
Generates grounded answers from retrieved chunks with source attribution.
"""

import logging
from typing import List, Dict, Any, Optional
import openai
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

logger = logging.getLogger(__name__)


class RAGAnswerer:
    """
    Generate answers with inline citations using LLM.

    Takes retrieved chunks and generates a comprehensive answer
    with inline source citations in [N] format.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "deepseek/deepseek-chat",
        max_tokens: int = 2000,
        temperature: float = 0.7
    ):
        """
        Initialize RAG Answerer.

        Args:
            api_key: OpenRouter API key
            model: Model ID for generation (default: deepseek/deepseek-chat)
            max_tokens: Maximum tokens in response (default: 2000)
            temperature: Sampling temperature (default: 0.7, range: 0-2)
        """
        self.api_key = api_key
        self.model = model
        self.max_tokens = max_tokens
        self.temperature = temperature

        # Configure OpenAI client for OpenRouter
        self.client = openai.OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=api_key
        )

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10),
        retry=retry_if_exception_type((openai.RateLimitError, openai.APITimeoutError))
    )
    def _generate_with_retry(self, messages: List[Dict[str, str]]) -> str:
        """
        Generate completion with automatic retry logic.

        Args:
            messages: List of message dicts with role and content

        Returns:
            str: Generated response text

        Raises:
            openai.OpenAIError: If API call fails after retries
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            answer = response.choices[0].message.content
            logger.debug(f"Generated answer ({len(answer)} chars)")
            return answer

        except openai.RateLimitError as e:
            logger.warning(f"Rate limit hit, retrying: {e}")
            raise
        except openai.APITimeoutError as e:
            logger.warning(f"API timeout, retrying: {e}")
            raise
        except openai.APIError as e:
            logger.error(f"OpenRouter API error: {e}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error generating answer: {e}")
            raise

    def generate_answer(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]],
        answer_length: str = "medium",
        answer_style: str = "technical",
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Generate an answer with inline citations from retrieved chunks.

        Args:
            query: User's question
            retrieved_chunks: List of retrieved chunk dicts with content and metadata
            answer_length: "brief" (1-2 sentences), "medium" (1 paragraph), "detailed" (2-3 paragraphs)
            answer_style: "technical" (precise terminology) or "beginner-friendly" (simplified explanations)
            conversation_history: Optional list of previous messages for context

        Returns:
            Dict[str, Any]: Answer with citations and metadata

        Example:
            >>> answerer = RAGAnswerer(api_key="sk-or-v1-...")
            >>> chunks = retriever.search("What is forward kinematics?", top_k=5)
            >>> result = answerer.generate_answer(
            ...     query="What is forward kinematics?",
            ...     retrieved_chunks=chunks
            ... )
            >>> print(result["answer"])
            "Forward kinematics is the process of computing... [1] This is done using... [2]"
        """
        logger.info(f"Generating answer for query: {query[:100]}...")

        if not retrieved_chunks:
            logger.warning("No retrieved chunks provided")
            return {
                "answer": "I couldn't find relevant information in the textbook to answer this question.",
                "sources": [],
                "confidence": 0.0,
                "metadata": {
                    "chunks_used": 0,
                    "answer_length": answer_length,
                    "answer_style": answer_style
                }
            }

        # Build context from retrieved chunks
        context = self._build_context(retrieved_chunks)

        # Build system prompt
        system_prompt = self._build_system_prompt(answer_length, answer_style)

        # Build user prompt
        user_prompt = self._build_user_prompt(query, context)

        # Prepare messages
        messages = [{"role": "system", "content": system_prompt}]

        # Add conversation history if provided
        if conversation_history:
            messages.extend(conversation_history[-4:])  # Last 2 turns (4 messages)

        messages.append({"role": "user", "content": user_prompt})

        # Generate answer
        try:
            answer_text = self._generate_with_retry(messages)

            # Extract citations and validate
            citations = self._extract_citations(answer_text, retrieved_chunks)

            # Calculate confidence
            confidence = self._calculate_confidence(retrieved_chunks, answer_text)

            # Build source list
            sources = self._build_sources(retrieved_chunks)

            # Generate follow-up suggestions
            follow_ups = self._suggest_follow_ups(query, retrieved_chunks, answer_length)

            result = {
                "answer": answer_text,
                "sources": sources,
                "confidence": confidence,
                "citations": citations,
                "follow_up_suggestions": follow_ups,
                "metadata": {
                    "chunks_used": len(retrieved_chunks),
                    "answer_length": answer_length,
                    "answer_style": answer_style,
                    "model": self.model,
                    "avg_chunk_score": sum(c["score"] for c in retrieved_chunks) / len(retrieved_chunks)
                }
            }

            logger.info(f"Successfully generated answer (confidence: {confidence:.2f})")
            return result

        except Exception as e:
            logger.error(f"Failed to generate answer: {e}")
            return {
                "answer": "I encountered an error while generating the answer. Please try again.",
                "sources": [],
                "confidence": 0.0,
                "error": str(e),
                "metadata": {
                    "chunks_used": len(retrieved_chunks),
                    "answer_length": answer_length,
                    "answer_style": answer_style
                }
            }

    def _build_context(self, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Build context string from retrieved chunks with source numbering.

        Args:
            retrieved_chunks: List of chunk dicts

        Returns:
            str: Formatted context with numbered sources
        """
        context_parts = []
        for i, chunk in enumerate(retrieved_chunks, start=1):
            section = " > ".join(filter(None, [
                chunk.get("section_h1", ""),
                chunk.get("section_h2", ""),
                chunk.get("section_h3", "")
            ]))

            source_ref = f"{chunk.get('file_path', 'unknown')}:{chunk.get('line_start', 0)}"

            context_parts.append(
                f"[Source {i}] ({section})\n"
                f"Location: {source_ref}\n"
                f"Content: {chunk['content']}\n"
            )

        return "\n".join(context_parts)

    def _build_system_prompt(self, answer_length: str, answer_style: str) -> str:
        """
        Build system prompt based on answer preferences.

        Args:
            answer_length: brief | medium | detailed
            answer_style: technical | beginner-friendly

        Returns:
            str: System prompt
        """
        length_instructions = {
            "brief": "Provide a concise answer in 1-2 sentences.",
            "medium": "Provide a comprehensive answer in 1 paragraph (4-6 sentences).",
            "detailed": "Provide a detailed explanation in 2-3 paragraphs."
        }

        style_instructions = {
            "technical": "Use precise technical terminology and assume familiarity with robotics concepts.",
            "beginner-friendly": "Explain concepts in simple terms, avoiding jargon where possible. Define technical terms when used."
        }

        return f"""You are a helpful AI assistant for a Physical AI and Humanoid Robotics textbook.
Your task is to answer questions based ONLY on the provided textbook content.

Guidelines:
1. Answer ONLY using information from the provided sources
2. Add inline citations using [N] format (e.g., [1], [2]) after each claim
3. {length_instructions.get(answer_length, length_instructions['medium'])}
4. {style_instructions.get(answer_style, style_instructions['technical'])}
5. If the sources don't contain enough information, say so explicitly
6. Do not make up information or draw from outside knowledge
7. If multiple sources say similar things, cite all relevant sources
8. Maintain a clear, educational tone

Citation Format:
- Use [1], [2], etc. to reference sources
- Place citations immediately after the relevant statement
- Example: "Forward kinematics computes the end-effector pose from joint angles [1]."
"""

    def _build_user_prompt(self, query: str, context: str) -> str:
        """
        Build user prompt with query and context.

        Args:
            query: User's question
            context: Formatted context from retrieved chunks

        Returns:
            str: User prompt
        """
        return f"""Question: {query}

Retrieved Content:
{context}

Please answer the question using the content above. Remember to include inline citations [N] for each fact or claim."""

    def _extract_citations(
        self,
        answer_text: str,
        retrieved_chunks: List[Dict[str, Any]]
    ) -> List[int]:
        """
        Extract citation numbers from answer text.

        Args:
            answer_text: Generated answer with [N] citations
            retrieved_chunks: List of retrieved chunks

        Returns:
            List[int]: List of cited source numbers
        """
        import re
        citation_pattern = r'\[(\d+)\]'
        citations = re.findall(citation_pattern, answer_text)
        cited_numbers = [int(n) for n in citations if int(n) <= len(retrieved_chunks)]

        logger.debug(f"Found {len(cited_numbers)} citations in answer")
        return sorted(set(cited_numbers))

    def _calculate_confidence(
        self,
        retrieved_chunks: List[Dict[str, Any]],
        answer_text: str
    ) -> float:
        """
        Calculate confidence score for the answer.

        Based on:
        - Average retrieval scores
        - Number of chunks used
        - Presence of citations

        Args:
            retrieved_chunks: List of retrieved chunks
            answer_text: Generated answer

        Returns:
            float: Confidence score (0-1)
        """
        if not retrieved_chunks:
            return 0.0

        # Average retrieval score
        avg_score = sum(c["score"] for c in retrieved_chunks) / len(retrieved_chunks)

        # Citation coverage (did the model cite the sources?)
        citations = self._extract_citations(answer_text, retrieved_chunks)
        citation_ratio = len(citations) / len(retrieved_chunks) if retrieved_chunks else 0

        # Combine factors
        confidence = (avg_score * 0.7) + (citation_ratio * 0.3)

        return round(confidence, 2)

    def _build_sources(self, retrieved_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Build source reference list from retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved chunks

        Returns:
            List[Dict[str, Any]]: List of source dicts
        """
        sources = []
        for i, chunk in enumerate(retrieved_chunks, start=1):
            section = " > ".join(filter(None, [
                chunk.get("section_h1", ""),
                chunk.get("section_h2", ""),
                chunk.get("section_h3", "")
            ]))

            sources.append({
                "id": i,
                "file_path": chunk.get("file_path", ""),
                "line_range": f"{chunk.get('line_start', 0)}-{chunk.get('line_end', 0)}",
                "section": section,
                "score": chunk.get("score", 0.0)
            })

        return sources

    def _suggest_follow_ups(
        self,
        query: str,
        retrieved_chunks: List[Dict[str, Any]],
        answer_length: str
    ) -> List[str]:
        """
        Suggest follow-up questions based on the context.

        Args:
            query: Original user query
            retrieved_chunks: Retrieved chunks
            answer_length: Answer length preference

        Returns:
            List[str]: List of suggested follow-up questions
        """
        # Simple heuristic-based suggestions
        suggestions = []

        # Extract topics from chunks
        topics = set()
        for chunk in retrieved_chunks[:3]:  # Top 3 chunks
            tags = chunk.get("tags", [])
            topics.update(tags)

        # Generate suggestions based on topics
        if "kinematics" in topics:
            suggestions.append("How do I implement forward kinematics in code?")
            suggestions.append("What is the difference between forward and inverse kinematics?")
        if "ros" in topics or "ros2" in topics:
            suggestions.append("How do I create a ROS2 node?")
            suggestions.append("What are ROS2 topics and services?")
        if "deep learning" in topics or "neural network" in topics:
            suggestions.append("What neural network architectures are used in robotics?")
            suggestions.append("How do I train a robot learning model?")

        # Generic suggestions if no specific topics
        if not suggestions:
            suggestions.append("Can you explain this in more detail?")
            suggestions.append("What are some examples of this concept?")
            suggestions.append("How is this implemented in practice?")

        return suggestions[:3]  # Return top 3


if __name__ == "__main__":
    import argparse
    import json
    import sys
    from pathlib import Path

    # Add parent directories to path for imports
    sys.path.append(str(Path(__file__).parent.parent / "rag-indexer"))
    sys.path.append(str(Path(__file__).parent.parent / "rag-retriever"))

    from embeddings import EmbeddingGenerator
    from retriever import RAGRetriever

    parser = argparse.ArgumentParser(description="RAG Answerer for question answering")
    parser.add_argument("--qdrant-url", required=True, help="Qdrant Cloud URL")
    parser.add_argument("--qdrant-api-key", required=True, help="Qdrant API key")
    parser.add_argument("--collection", default="physical-ai-textbook", help="Collection name")
    parser.add_argument("--openrouter-api-key", required=True, help="OpenRouter API key")
    parser.add_argument("--query", required=True, help="User question")
    parser.add_argument("--model", default="meta-llama/llama-3.1-8b-instruct:free", help="LLM model")
    parser.add_argument("--length", default="medium", choices=["brief", "medium", "detailed"], help="Answer length")
    parser.add_argument("--style", default="technical", choices=["technical", "beginner-friendly"], help="Answer style")
    parser.add_argument("--top-k", type=int, default=5, help="Number of chunks to retrieve")

    args = parser.parse_args()

    # Initialize components
    embedding_gen = EmbeddingGenerator(
        api_key=args.openrouter_api_key,
        model="text-embedding-3-small"
    )

    retriever = RAGRetriever(
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        collection_name=args.collection,
        embedding_generator=embedding_gen
    )

    answerer = RAGAnswerer(
        api_key=args.openrouter_api_key,
        model=args.model
    )

    # Retrieve chunks
    print(f"Retrieving relevant chunks for: {args.query}", file=sys.stderr)
    chunks = retriever.search(args.query, top_k=args.top_k)

    if not chunks:
        print(json.dumps({
            "error": "No relevant chunks found",
            "query": args.query
        }, indent=2))
        sys.exit(1)

    print(f"Found {len(chunks)} relevant chunks", file=sys.stderr)

    # Generate answer
    print(f"Generating answer...", file=sys.stderr)
    result = answerer.generate_answer(
        query=args.query,
        retrieved_chunks=chunks,
        answer_length=args.length,
        answer_style=args.style
    )

    # Output result
    output = {
        "query": args.query,
        "answer": result["answer"],
        "confidence": result["confidence"],
        "sources": result["sources"],
        "follow_up_suggestions": result.get("follow_up_suggestions", []),
        "metadata": result["metadata"]
    }

    print(json.dumps(output, indent=2))
