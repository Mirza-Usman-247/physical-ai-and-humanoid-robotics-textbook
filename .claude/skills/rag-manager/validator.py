"""
Validation utilities for RAG pipeline quality assurance.
Validates retrieval quality, citation correctness, and answer faithfulness.
"""

import logging
from typing import List, Dict, Any, Tuple
import re

logger = logging.getLogger(__name__)


class RAGValidator:
    """
    Validate RAG pipeline outputs for quality and correctness.

    Checks:
    - Retrieval quality (scores, coverage, diversity)
    - Citation correctness and completeness
    - Answer faithfulness to sources
    - Source coverage and relevance
    """

    def __init__(
        self,
        min_retrieval_score: float = 0.7,
        min_citation_coverage: float = 0.5,
        max_duplicate_sources: int = 3
    ):
        """
        Initialize RAG Validator.

        Args:
            min_retrieval_score: Minimum acceptable retrieval score
            min_citation_coverage: Minimum ratio of sources that should be cited
            max_duplicate_sources: Maximum chunks from same file
        """
        self.min_retrieval_score = min_retrieval_score
        self.min_citation_coverage = min_citation_coverage
        self.max_duplicate_sources = max_duplicate_sources

    def validate_retrieval_quality(
        self,
        retrieved_chunks: List[Dict[str, Any]],
        query: str
    ) -> Dict[str, Any]:
        """
        Validate quality of retrieved chunks.

        Args:
            retrieved_chunks: List of retrieved chunk dicts
            query: User query

        Returns:
            Dict[str, Any]: Validation report with status and warnings

        Checks:
        - Minimum score threshold
        - Source diversity
        - Coverage breadth
        """
        warnings = []
        errors = []

        if not retrieved_chunks:
            errors.append("No chunks retrieved")
            return {
                "valid": False,
                "errors": errors,
                "warnings": [],
                "metrics": {}
            }

        # Check minimum scores
        low_scores = [
            f"Chunk {i+1}: {chunk['score']:.2f}"
            for i, chunk in enumerate(retrieved_chunks)
            if chunk.get("score", 0) < self.min_retrieval_score
        ]

        if low_scores:
            warnings.append(f"Chunks below threshold ({self.min_retrieval_score}): {', '.join(low_scores)}")

        # Check source diversity
        file_paths = [chunk.get("file_path") for chunk in retrieved_chunks]
        unique_files = len(set(file_paths))

        if unique_files == 1 and len(retrieved_chunks) > 2:
            warnings.append(f"All chunks from single file: {file_paths[0]}")

        # Check for too many chunks from same file
        from collections import Counter
        file_counts = Counter(file_paths)
        excessive_files = [
            f"{file}: {count} chunks"
            for file, count in file_counts.items()
            if count > self.max_duplicate_sources
        ]

        if excessive_files:
            warnings.append(f"Too many chunks from same files: {', '.join(excessive_files)}")

        # Calculate metrics
        avg_score = sum(c.get("score", 0) for c in retrieved_chunks) / len(retrieved_chunks)
        score_variance = sum(
            (c.get("score", 0) - avg_score) ** 2
            for c in retrieved_chunks
        ) / len(retrieved_chunks)

        metrics = {
            "chunks_count": len(retrieved_chunks),
            "unique_files": unique_files,
            "avg_score": round(avg_score, 3),
            "score_variance": round(score_variance, 3),
            "min_score": round(min(c.get("score", 0) for c in retrieved_chunks), 3),
            "max_score": round(max(c.get("score", 0) for c in retrieved_chunks), 3)
        }

        is_valid = len(errors) == 0

        return {
            "valid": is_valid,
            "errors": errors,
            "warnings": warnings,
            "metrics": metrics
        }

    def validate_citations(
        self,
        answer: str,
        sources: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate citation correctness in answer.

        Args:
            answer: Generated answer text with [N] citations
            sources: List of source dicts

        Returns:
            Dict[str, Any]: Validation report

        Checks:
        - All citations are valid (within range)
        - Sufficient citation coverage
        - No missing citations
        """
        errors = []
        warnings = []

        # Extract citations
        citation_pattern = re.compile(r'\[(\d+)\]')
        citations = citation_pattern.findall(answer)
        citation_numbers = [int(n) for n in citations]

        if not sources:
            if citation_numbers:
                errors.append("Citations found but no sources provided")
            return {
                "valid": len(errors) == 0,
                "errors": errors,
                "warnings": warnings,
                "metrics": {}
            }

        # Check citation validity
        invalid_citations = [
            n for n in citation_numbers
            if n < 1 or n > len(sources)
        ]

        if invalid_citations:
            errors.append(f"Invalid citation numbers: {invalid_citations}")

        # Check citation coverage
        unique_citations = set(citation_numbers)
        citation_coverage = len(unique_citations) / len(sources) if sources else 0

        if citation_coverage < self.min_citation_coverage:
            warnings.append(
                f"Low citation coverage: {citation_coverage:.1%} "
                f"(only {len(unique_citations)}/{len(sources)} sources cited)"
            )

        # Check for uncited high-score sources
        cited_ids = {src.get("id") for src in sources if src.get("id") in unique_citations}
        uncited_sources = [
            f"Source {src.get('id')}: {src.get('score', 0):.2f}"
            for src in sources
            if src.get("id") not in cited_ids and src.get("score", 0) > 0.8
        ]

        if uncited_sources:
            warnings.append(f"High-score sources not cited: {', '.join(uncited_sources)}")

        # Calculate metrics
        metrics = {
            "total_citations": len(citation_numbers),
            "unique_citations": len(unique_citations),
            "citation_coverage": round(citation_coverage, 2),
            "sources_available": len(sources),
            "sources_cited": len(unique_citations)
        }

        is_valid = len(errors) == 0

        return {
            "valid": is_valid,
            "errors": errors,
            "warnings": warnings,
            "metrics": metrics
        }

    def validate_answer_faithfulness(
        self,
        answer: str,
        sources: List[Dict[str, Any]],
        query: str
    ) -> Dict[str, Any]:
        """
        Validate that answer is faithful to source material.

        Args:
            answer: Generated answer text
            sources: List of source dicts
            query: Original user query

        Returns:
            Dict[str, Any]: Validation report

        Note: This is a heuristic-based check. For production,
        consider using specialized models like NLI or fact-checking.
        """
        warnings = []
        errors = []

        # Check answer length appropriateness
        if len(answer) < 20:
            warnings.append("Answer is very short (< 20 chars)")
        elif len(answer) > 2000:
            warnings.append("Answer is very long (> 2000 chars)")

        # Check if answer contains query terms
        query_words = set(query.lower().split())
        answer_words = set(answer.lower().split())
        query_overlap = len(query_words & answer_words) / len(query_words) if query_words else 0

        if query_overlap < 0.3:
            warnings.append(f"Low query term overlap in answer: {query_overlap:.1%}")

        # Check for generic/unhelpful answers
        generic_phrases = [
            "i don't know",
            "i cannot answer",
            "no information",
            "not enough context",
            "unable to answer"
        ]

        answer_lower = answer.lower()
        if any(phrase in answer_lower for phrase in generic_phrases):
            errors.append("Answer contains generic/unhelpful phrases")

        # Check for source content overlap
        if sources:
            source_texts = " ".join(src.get("content", "") for src in sources[:3]).lower()
            source_words = set(source_texts.split())

            # Calculate answer words that appear in sources
            answer_content_words = set(w for w in answer_words if len(w) > 3)  # Only meaningful words
            source_overlap_words = answer_content_words & source_words
            source_overlap = len(source_overlap_words) / len(answer_content_words) if answer_content_words else 0

            if source_overlap < 0.4:
                warnings.append(f"Low overlap with source content: {source_overlap:.1%}")

        metrics = {
            "answer_length": len(answer),
            "answer_words": len(answer_words),
            "query_overlap": round(query_overlap, 2),
            "source_overlap": round(source_overlap, 2) if sources else 0
        }

        is_valid = len(errors) == 0

        return {
            "valid": is_valid,
            "errors": errors,
            "warnings": warnings,
            "metrics": metrics
        }

    def validate_complete_response(
        self,
        response: Dict[str, Any],
        query: str,
        retrieved_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Validate a complete RAG response.

        Args:
            response: Full RAG response dict
            query: User query
            retrieved_chunks: Retrieved chunks used for generation

        Returns:
            Dict[str, Any]: Comprehensive validation report
        """
        # Validate retrieval
        retrieval_validation = self.validate_retrieval_quality(
            retrieved_chunks,
            query
        )

        # Validate citations
        citation_validation = self.validate_citations(
            response.get("answer", ""),
            response.get("sources", [])
        )

        # Validate answer faithfulness
        faithfulness_validation = self.validate_answer_faithfulness(
            response.get("answer", ""),
            response.get("sources", []),
            query
        )

        # Aggregate results
        all_errors = (
            retrieval_validation.get("errors", []) +
            citation_validation.get("errors", []) +
            faithfulness_validation.get("errors", [])
        )

        all_warnings = (
            retrieval_validation.get("warnings", []) +
            citation_validation.get("warnings", []) +
            faithfulness_validation.get("warnings", [])
        )

        is_valid = (
            retrieval_validation.get("valid", False) and
            citation_validation.get("valid", False) and
            faithfulness_validation.get("valid", False)
        )

        # Calculate overall quality score
        quality_score = self._calculate_quality_score(
            retrieval_validation,
            citation_validation,
            faithfulness_validation,
            response.get("confidence", 0)
        )

        return {
            "valid": is_valid,
            "quality_score": quality_score,
            "errors": all_errors,
            "warnings": all_warnings,
            "details": {
                "retrieval": retrieval_validation,
                "citations": citation_validation,
                "faithfulness": faithfulness_validation
            }
        }

    def _calculate_quality_score(
        self,
        retrieval_validation: Dict,
        citation_validation: Dict,
        faithfulness_validation: Dict,
        confidence: float
    ) -> float:
        """
        Calculate overall quality score from validation results.

        Args:
            retrieval_validation: Retrieval validation report
            citation_validation: Citation validation report
            faithfulness_validation: Faithfulness validation report
            confidence: Model confidence score

        Returns:
            float: Quality score (0-1)
        """
        # Base score from confidence
        quality_score = confidence * 0.4

        # Add retrieval quality (30%)
        retrieval_metrics = retrieval_validation.get("metrics", {})
        avg_score = retrieval_metrics.get("avg_score", 0)
        quality_score += avg_score * 0.3

        # Add citation quality (20%)
        citation_metrics = citation_validation.get("metrics", {})
        citation_coverage = citation_metrics.get("citation_coverage", 0)
        quality_score += citation_coverage * 0.2

        # Add faithfulness (10%)
        faithfulness_metrics = faithfulness_validation.get("metrics", {})
        source_overlap = faithfulness_metrics.get("source_overlap", 0)
        quality_score += source_overlap * 0.1

        # Penalize for errors and warnings
        error_count = (
            len(retrieval_validation.get("errors", [])) +
            len(citation_validation.get("errors", [])) +
            len(faithfulness_validation.get("errors", []))
        )

        warning_count = (
            len(retrieval_validation.get("warnings", [])) +
            len(citation_validation.get("warnings", [])) +
            len(faithfulness_validation.get("warnings", []))
        )

        penalty = (error_count * 0.2) + (warning_count * 0.05)
        quality_score = max(0, quality_score - penalty)

        return round(min(quality_score, 1.0), 2)


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="RAG Validator - Quality assurance")
    parser.add_argument("--response", required=True, help="JSON file with RAG response")
    parser.add_argument("--query", required=True, help="User query")
    parser.add_argument("--chunks", required=True, help="JSON file with retrieved chunks")

    args = parser.parse_args()

    # Load data
    with open(args.response, 'r') as f:
        response = json.load(f)

    with open(args.chunks, 'r') as f:
        chunks = json.load(f)

    # Validate
    validator = RAGValidator()
    validation_result = validator.validate_complete_response(
        response=response,
        query=args.query,
        retrieved_chunks=chunks
    )

    print(json.dumps(validation_result, indent=2))

    # Exit with appropriate code
    if not validation_result["valid"]:
        exit(1)
