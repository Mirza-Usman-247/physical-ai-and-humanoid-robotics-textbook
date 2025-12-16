"""
Semantic retrieval over Qdrant vector database.
Searches indexed textbook content using embedding similarity.
"""

import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue, MatchAny

logger = logging.getLogger(__name__)


class RAGRetriever:
    """
    Semantic retriever for RAG system.

    Searches Qdrant vector database using embedding similarity
    with optional metadata filtering.
    """

    def __init__(
        self,
        qdrant_url: str,
        qdrant_api_key: str,
        collection_name: str,
        embedding_generator
    ):
        """
        Initialize RAG Retriever.

        Args:
            qdrant_url: Qdrant Cloud cluster URL
            qdrant_api_key: Qdrant API key
            collection_name: Name of the Qdrant collection
            embedding_generator: EmbeddingGenerator instance for query encoding
        """
        self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = collection_name
        self.embedding_generator = embedding_generator

    def search(
        self,
        query: str,
        top_k: int = 5,
        score_threshold: float = 0.3,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks using semantic similarity.

        Args:
            query: User query text
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (0-1, default: 0.7)
            filters: Optional metadata filters (module, chapter, tags)

        Returns:
            List[Dict[str, Any]]: Retrieved chunks with metadata and scores

        Example:
            >>> retriever = RAGRetriever(...)
            >>> results = retriever.search(
            ...     query="What is inverse kinematics?",
            ...     top_k=5,
            ...     filters={"module": "module-1-ros2"}
            ... )
            >>> for result in results:
            ...     print(f"{result['score']:.2f}: {result['content'][:100]}")
        """
        logger.info(f"Searching for: {query[:100]}...")

        # Generate query embedding
        try:
            query_embeddings = self.embedding_generator.generate_embeddings([query])
            if not query_embeddings:
                logger.error("Failed to generate query embedding")
                return []

            query_vector = query_embeddings[0]

        except Exception as e:
            logger.error(f"Error generating query embedding: {e}")
            return []

        # Build Qdrant filter
        qdrant_filter = self._build_filter(filters) if filters else None

        # Perform semantic search
        try:
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
                query_filter=qdrant_filter
            )

            logger.info(f"Found {len(search_result)} results above threshold {score_threshold}")

            # Format results
            results = []
            for hit in search_result:
                result = {
                    "chunk_id": hit.payload.get("chunk_id"),
                    "content": hit.payload.get("content"),
                    "score": float(hit.score),
                    "file_path": hit.payload.get("file_path"),
                    "section_h1": hit.payload.get("section_h1", ""),
                    "section_h2": hit.payload.get("section_h2", ""),
                    "section_h3": hit.payload.get("section_h3", ""),
                    "line_start": hit.payload.get("line_start"),
                    "line_end": hit.payload.get("line_end"),
                    "module": hit.payload.get("module"),
                    "chapter": hit.payload.get("chapter"),
                    "tags": hit.payload.get("tags", [])
                }
                results.append(result)

            return results

        except Exception as e:
            logger.error(f"Error during Qdrant search: {e}")
            return []

    def search_with_context(
        self,
        query: str,
        selected_text: Optional[str] = None,
        top_k: int = 5,
        score_threshold: float = 0.3,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search with additional context from selected text.

        Combines query with selected text to improve relevance.

        Args:
            query: User query text
            selected_text: Optional text selected by user
            top_k: Number of results to return
            score_threshold: Minimum similarity score
            filters: Optional metadata filters

        Returns:
            List[Dict[str, Any]]: Retrieved chunks
        """
        # Combine query with selected text
        if selected_text:
            # Weight query higher than selected text
            combined_query = f"{query}\n\nContext: {selected_text[:500]}"
            logger.info(f"Searching with context (selected text: {len(selected_text)} chars)")
        else:
            combined_query = query

        return self.search(
            query=combined_query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters=filters
        )

    def hybrid_search(
        self,
        query: str,
        top_k: int = 5,
        score_threshold: float = 0.3,
        keyword_boost: float = 0.1,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Hybrid search combining semantic and keyword matching.

        Boosts results that contain exact query terms.

        Args:
            query: User query text
            top_k: Number of results to return
            score_threshold: Minimum similarity score
            keyword_boost: Score boost for keyword matches (0-1, default: 0.1)
            filters: Optional metadata filters

        Returns:
            List[Dict[str, Any]]: Retrieved chunks with boosted scores
        """
        # Perform semantic search
        results = self.search(
            query=query,
            top_k=top_k * 2,  # Retrieve more candidates
            score_threshold=score_threshold * 0.9,  # Lower threshold
            filters=filters
        )

        # Extract query keywords (simple tokenization)
        query_keywords = set(query.lower().split())

        # Boost scores for keyword matches
        for result in results:
            content_lower = result["content"].lower()
            keyword_matches = sum(1 for kw in query_keywords if kw in content_lower)

            if keyword_matches > 0:
                boost = min(keyword_boost * keyword_matches, 0.3)  # Cap at 0.3
                result["score"] = min(result["score"] + boost, 1.0)
                result["keyword_matches"] = keyword_matches
                logger.debug(f"Boosted score by {boost:.2f} for {keyword_matches} keyword matches")

        # Re-sort by boosted scores and apply top_k
        results.sort(key=lambda x: x["score"], reverse=True)
        results = results[:top_k]

        # Apply final threshold
        results = [r for r in results if r["score"] >= score_threshold]

        logger.info(f"Hybrid search returned {len(results)} results")
        return results

    def search_by_module(
        self,
        query: str,
        module: str,
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search within a specific module.

        Args:
            query: User query text
            module: Module name (e.g., "module-1-ros2")
            top_k: Number of results to return
            score_threshold: Minimum similarity score

        Returns:
            List[Dict[str, Any]]: Retrieved chunks from specified module
        """
        return self.search(
            query=query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters={"module": module}
        )

    def search_by_chapter(
        self,
        query: str,
        chapter: str,
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search within a specific chapter.

        Args:
            query: User query text
            chapter: Chapter name (e.g., "chapter-3")
            top_k: Number of results to return
            score_threshold: Minimum similarity score

        Returns:
            List[Dict[str, Any]]: Retrieved chunks from specified chapter
        """
        return self.search(
            query=query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters={"chapter": chapter}
        )

    def search_by_tags(
        self,
        query: str,
        tags: List[str],
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Search for chunks with specific tags.

        Args:
            query: User query text
            tags: List of tags to filter by (e.g., ["kinematics", "ros"])
            top_k: Number of results to return
            score_threshold: Minimum similarity score

        Returns:
            List[Dict[str, Any]]: Retrieved chunks with specified tags
        """
        return self.search(
            query=query,
            top_k=top_k,
            score_threshold=score_threshold,
            filters={"tags": tags}
        )

    def _build_filter(self, filters: Dict[str, Any]) -> Filter:
        """
        Build Qdrant filter from metadata dict.

        Args:
            filters: Dictionary of filter conditions

        Returns:
            Filter: Qdrant filter object

        Supported filters:
            - module: str
            - chapter: str
            - tags: List[str] (any match)
            - file_path: str
        """
        conditions = []

        # Module filter
        if "module" in filters:
            conditions.append(
                FieldCondition(
                    key="module",
                    match=MatchValue(value=filters["module"])
                )
            )

        # Chapter filter
        if "chapter" in filters:
            conditions.append(
                FieldCondition(
                    key="chapter",
                    match=MatchValue(value=filters["chapter"])
                )
            )

        # Tags filter (match any)
        if "tags" in filters:
            tags = filters["tags"] if isinstance(filters["tags"], list) else [filters["tags"]]
            conditions.append(
                FieldCondition(
                    key="tags",
                    match=MatchAny(any=tags)
                )
            )

        # File path filter
        if "file_path" in filters:
            conditions.append(
                FieldCondition(
                    key="file_path",
                    match=MatchValue(value=filters["file_path"])
                )
            )

        return Filter(must=conditions) if conditions else None

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dict[str, Any]: Collection statistics

        Example:
            >>> retriever.get_collection_info()
            {
                "name": "physical-ai-textbook",
                "vectors_count": 1200,
                "indexed_points": 1200,
                "status": "green"
            }
        """
        try:
            collection_info = self.client.get_collection(collection_name=self.collection_name)

            return {
                "name": self.collection_name,
                "vectors_count": collection_info.vectors_count,
                "indexed_points": collection_info.points_count,
                "status": collection_info.status
            }

        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {
                "name": self.collection_name,
                "error": str(e)
            }


if __name__ == "__main__":
    import argparse
    import json
    import sys

    # Add parent directory to path for imports
    from pathlib import Path
    sys.path.append(str(Path(__file__).parent.parent / "rag-indexer"))
    from embeddings import EmbeddingGenerator

    parser = argparse.ArgumentParser(description="RAG Retriever for semantic search")
    parser.add_argument("--qdrant-url", required=True, help="Qdrant Cloud URL")
    parser.add_argument("--qdrant-api-key", required=True, help="Qdrant API key")
    parser.add_argument("--collection", default="physical-ai-textbook", help="Collection name")
    parser.add_argument("--openrouter-api-key", required=True, help="OpenRouter API key")
    parser.add_argument("--query", required=True, help="Search query")
    parser.add_argument("--top-k", type=int, default=5, help="Number of results")
    parser.add_argument("--threshold", type=float, default=0.7, help="Score threshold")
    parser.add_argument("--module", help="Filter by module")
    parser.add_argument("--chapter", help="Filter by chapter")
    parser.add_argument("--tags", nargs="+", help="Filter by tags")
    parser.add_argument("--hybrid", action="store_true", help="Use hybrid search")

    args = parser.parse_args()

    # Initialize embedding generator
    embedding_gen = EmbeddingGenerator(
        api_key=args.openrouter_api_key,
        model="text-embedding-3-small"
    )

    # Initialize retriever
    retriever = RAGRetriever(
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        collection_name=args.collection,
        embedding_generator=embedding_gen
    )

    # Build filters
    filters = {}
    if args.module:
        filters["module"] = args.module
    if args.chapter:
        filters["chapter"] = args.chapter
    if args.tags:
        filters["tags"] = args.tags

    # Perform search
    if args.hybrid:
        results = retriever.hybrid_search(
            query=args.query,
            top_k=args.top_k,
            score_threshold=args.threshold,
            filters=filters if filters else None
        )
    else:
        results = retriever.search(
            query=args.query,
            top_k=args.top_k,
            score_threshold=args.threshold,
            filters=filters if filters else None
        )

    # Output results
    output = {
        "query": args.query,
        "results_count": len(results),
        "results": [
            {
                "score": r["score"],
                "file_path": r["file_path"],
                "section": f"{r['section_h1']} > {r['section_h2']} > {r['section_h3']}".strip(" > "),
                "line_range": f"{r['line_start']}-{r['line_end']}",
                "content_preview": r["content"][:200] + "..." if len(r["content"]) > 200 else r["content"]
            }
            for r in results
        ]
    }

    print(json.dumps(output, indent=2))
