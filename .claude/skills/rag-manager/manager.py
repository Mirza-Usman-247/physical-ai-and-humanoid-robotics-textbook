"""
RAG pipeline orchestration and management.
Coordinates indexing, retrieval, and answer generation with validation.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
import sys
from pathlib import Path

# Add parent directories to path for imports
sys.path.append(str(Path(__file__).parent.parent / "rag-indexer"))
sys.path.append(str(Path(__file__).parent.parent / "rag-retriever"))
sys.path.append(str(Path(__file__).parent.parent / "rag-answerer"))

from indexer import RAGIndexer
from embeddings import EmbeddingGenerator
from retriever import RAGRetriever
from answerer import RAGAnswerer
from citation_mapper import CitationMapper

logger = logging.getLogger(__name__)


class RAGManager:
    """
    Orchestrate the complete RAG pipeline.

    Manages:
    - Indexing operations
    - Retrieval with filtering
    - Answer generation with citations
    - Validation and error handling
    - Conversation history
    """

    def __init__(
        self,
        qdrant_url: str,
        qdrant_api_key: str,
        openrouter_api_key: str,
        collection_name: str = "physical-ai-textbook",
        embedding_model: str = "text-embedding-3-small",
        llm_model: str = "deepseek/deepseek-chat"
    ):
        """
        Initialize RAG Manager.

        Args:
            qdrant_url: Qdrant Cloud cluster URL
            qdrant_api_key: Qdrant API key
            openrouter_api_key: OpenRouter API key
            collection_name: Name of the Qdrant collection
            embedding_model: Model for embeddings (default: text-embedding-3-small)
            llm_model: Model for answer generation (default: llama-3.1-8b)
        """
        self.collection_name = collection_name

        # Initialize embedding generator
        self.embedding_gen = EmbeddingGenerator(
            api_key=openrouter_api_key,
            model=embedding_model
        )

        # Initialize indexer
        self.indexer = RAGIndexer(
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            collection_name=collection_name,
            embedding_generator=self.embedding_gen
        )

        # Initialize retriever
        self.retriever = RAGRetriever(
            qdrant_url=qdrant_url,
            qdrant_api_key=qdrant_api_key,
            collection_name=collection_name,
            embedding_generator=self.embedding_gen
        )

        # Initialize answerer
        self.answerer = RAGAnswerer(
            api_key=openrouter_api_key,
            model=llm_model
        )

        # Initialize citation mapper
        self.citation_mapper = CitationMapper()

        logger.info(f"RAG Manager initialized with collection: {collection_name}")

    def query(
        self,
        question: str,
        top_k: int = 5,
        score_threshold: float = 0.3,
        answer_length: str = "medium",
        answer_style: str = "technical",
        filters: Optional[Dict[str, Any]] = None,
        conversation_history: Optional[List[Dict[str, str]]] = None,
        selected_text: Optional[str] = None,
        validate_citations: bool = True
    ) -> Dict[str, Any]:
        """
        Process a complete RAG query.

        Workflow:
        1. Retrieve relevant chunks
        2. Generate answer with citations
        3. Validate citations (optional)
        4. Return result with metadata

        Args:
            question: User's question
            top_k: Number of chunks to retrieve
            score_threshold: Minimum similarity score
            answer_length: "brief" | "medium" | "detailed"
            answer_style: "technical" | "beginner-friendly"
            filters: Optional metadata filters (module, chapter, tags)
            conversation_history: Optional list of previous messages
            selected_text: Optional user-selected text for context
            validate_citations: Whether to validate citation correctness

        Returns:
            Dict[str, Any]: Complete RAG response with answer, sources, and metadata

        Example:
            >>> manager = RAGManager(...)
            >>> result = manager.query(
            ...     question="What is forward kinematics?",
            ...     answer_length="medium",
            ...     filters={"module": "module-1-ros2"}
            ... )
            >>> print(result["answer"])
            >>> print(result["confidence"])
        """
        logger.info(f"Processing query: {question[:100]}...")
        start_time = datetime.utcnow()

        try:
            # Step 1: Retrieve relevant chunks
            logger.debug(f"Retrieving chunks (top_k={top_k}, threshold={score_threshold})")

            if selected_text:
                chunks = self.retriever.search_with_context(
                    query=question,
                    selected_text=selected_text,
                    top_k=top_k,
                    score_threshold=score_threshold,
                    filters=filters
                )
            else:
                chunks = self.retriever.search(
                    query=question,
                    top_k=top_k,
                    score_threshold=score_threshold,
                    filters=filters
                )

            logger.info(f"Retrieved {len(chunks)} chunks")

            # Step 2: Generate answer
            logger.debug("Generating answer with citations")
            answer_result = self.answerer.generate_answer(
                query=question,
                retrieved_chunks=chunks,
                answer_length=answer_length,
                answer_style=answer_style,
                conversation_history=conversation_history
            )

            # Step 3: Validate citations (optional)
            citation_validation = {"valid": True, "errors": []}
            if validate_citations and chunks:
                is_valid, errors = self.citation_mapper.validate_citations(
                    answer_result["answer"],
                    len(chunks)
                )
                citation_validation = {"valid": is_valid, "errors": errors}

                if not is_valid:
                    logger.warning(f"Citation validation errors: {errors}")

            # Step 4: Calculate processing time
            end_time = datetime.utcnow()
            response_time_ms = int((end_time - start_time).total_seconds() * 1000)

            # Step 5: Build complete response
            result = {
                "answer": answer_result["answer"],
                "sources": answer_result["sources"],
                "confidence": answer_result["confidence"],
                "citations": answer_result.get("citations", []),
                "follow_up_suggestions": answer_result.get("follow_up_suggestions", []),
                "metadata": {
                    "chunks_retrieved": len(chunks),
                    "chunks_used": answer_result.get("metadata", {}).get("chunks_used", 0),
                    "answer_length": answer_length,
                    "answer_style": answer_style,
                    "response_time_ms": response_time_ms,
                    "model": answer_result.get("metadata", {}).get("model", "unknown"),
                    "collection": self.collection_name,
                    "citation_validation": citation_validation,
                    "timestamp": datetime.utcnow().isoformat()
                }
            }

            logger.info(f"Query completed successfully (confidence: {result['confidence']:.2f}, time: {response_time_ms}ms)")
            return result

        except Exception as e:
            logger.error(f"Error processing query: {e}", exc_info=True)
            return {
                "answer": "I encountered an error while processing your question. Please try again.",
                "sources": [],
                "confidence": 0.0,
                "error": str(e),
                "metadata": {
                    "chunks_retrieved": 0,
                    "chunks_used": 0,
                    "response_time_ms": 0,
                    "timestamp": datetime.utcnow().isoformat()
                }
            }

    def query_with_history(
        self,
        question: str,
        conversation_id: str,
        conversation_history: List[Dict[str, str]],
        **kwargs
    ) -> Dict[str, Any]:
        """
        Process a query with conversation history.

        Args:
            question: User's question
            conversation_id: Unique conversation identifier
            conversation_history: List of previous messages
            **kwargs: Additional arguments passed to query()

        Returns:
            Dict[str, Any]: RAG response
        """
        logger.info(f"Processing query with history (conversation_id: {conversation_id}, history length: {len(conversation_history)})")

        result = self.query(
            question=question,
            conversation_history=conversation_history,
            **kwargs
        )

        result["metadata"]["conversation_id"] = conversation_id
        result["metadata"]["history_length"] = len(conversation_history)

        return result

    def index_directory(
        self,
        directory_path: str,
        file_pattern: str = "**/*.md",
        commit_hash: Optional[str] = None,
        force_recreate: bool = False
    ) -> Dict[str, Any]:
        """
        Index all markdown files in a directory.

        Args:
            directory_path: Path to directory containing markdown files
            file_pattern: Glob pattern for matching files (default: **/*.md)
            commit_hash: Git commit hash for versioning
            force_recreate: If True, delete and recreate collection

        Returns:
            Dict[str, Any]: Indexing results

        Example:
            >>> manager.index_directory(
            ...     directory_path="docs",
            ...     commit_hash="abc123"
            ... )
        """
        logger.info(f"Indexing directory: {directory_path}")

        try:
            # Create collection if needed
            if force_recreate:
                self.indexer.create_collection(force_recreate=True)
            else:
                self.indexer.create_collection()

            # Index directory
            result = self.indexer.index_directory(
                directory_path=directory_path,
                file_pattern=file_pattern,
                commit_hash=commit_hash
            )

            total_chunks = sum(len(chunks) for chunks in result.values())

            return {
                "status": "success",
                "files_indexed": len(result),
                "total_chunks": total_chunks,
                "collection": self.collection_name,
                "commit_hash": commit_hash
            }

        except Exception as e:
            logger.error(f"Error indexing directory: {e}", exc_info=True)
            return {
                "status": "error",
                "error": str(e)
            }

    def reindex_file(
        self,
        file_path: str,
        commit_hash: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Re-index a single file (delete old chunks, index new).

        Args:
            file_path: Path to file to re-index
            commit_hash: Git commit hash

        Returns:
            Dict[str, Any]: Re-indexing results
        """
        logger.info(f"Re-indexing file: {file_path}")

        try:
            # Delete old chunks
            deleted_count = self.indexer.delete_file_chunks(file_path)

            # Index new version
            chunk_ids = self.indexer.index_file(
                file_path=file_path,
                commit_hash=commit_hash
            )

            return {
                "status": "success",
                "file_path": file_path,
                "deleted_chunks": deleted_count,
                "new_chunks": len(chunk_ids),
                "commit_hash": commit_hash
            }

        except Exception as e:
            logger.error(f"Error re-indexing file: {e}", exc_info=True)
            return {
                "status": "error",
                "error": str(e)
            }

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the Qdrant collection.

        Returns:
            Dict[str, Any]: Collection statistics
        """
        try:
            info = self.retriever.get_collection_info()
            return {
                "status": "success",
                **info
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {
                "status": "error",
                "error": str(e)
            }

    def health_check(self) -> Dict[str, Any]:
        """
        Check health of all RAG components.

        Returns:
            Dict[str, Any]: Health status of each component
        """
        health = {
            "timestamp": datetime.utcnow().isoformat(),
            "collection": self.collection_name,
            "components": {}
        }

        # Check Qdrant
        try:
            collection_info = self.retriever.get_collection_info()
            health["components"]["qdrant"] = {
                "status": "healthy",
                "vectors_count": collection_info.get("vectors_count", 0)
            }
        except Exception as e:
            health["components"]["qdrant"] = {
                "status": "unhealthy",
                "error": str(e)
            }

        # Check embeddings
        try:
            test_embedding = self.embedding_gen.generate_embeddings(["test"])
            health["components"]["embeddings"] = {
                "status": "healthy",
                "dimension": len(test_embedding[0]) if test_embedding else 0
            }
        except Exception as e:
            health["components"]["embeddings"] = {
                "status": "unhealthy",
                "error": str(e)
            }

        # Overall status
        all_healthy = all(
            comp["status"] == "healthy"
            for comp in health["components"].values()
        )
        health["overall_status"] = "healthy" if all_healthy else "unhealthy"

        return health


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="RAG Manager - Pipeline orchestration")
    parser.add_argument("--qdrant-url", required=True, help="Qdrant Cloud URL")
    parser.add_argument("--qdrant-api-key", required=True, help="Qdrant API key")
    parser.add_argument("--openrouter-api-key", required=True, help="OpenRouter API key")
    parser.add_argument("--collection", default="physical-ai-textbook", help="Collection name")

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Query command
    query_parser = subparsers.add_parser("query", help="Query the RAG system")
    query_parser.add_argument("--question", required=True, help="User question")
    query_parser.add_argument("--top-k", type=int, default=5, help="Number of chunks")
    query_parser.add_argument("--length", default="medium", choices=["brief", "medium", "detailed"], help="Answer length")
    query_parser.add_argument("--style", default="technical", choices=["technical", "beginner-friendly"], help="Answer style")

    # Index command
    index_parser = subparsers.add_parser("index", help="Index a directory")
    index_parser.add_argument("--directory", required=True, help="Directory to index")
    index_parser.add_argument("--commit-hash", help="Git commit hash")
    index_parser.add_argument("--force-recreate", action="store_true", help="Force recreate collection")

    # Health command
    subparsers.add_parser("health", help="Check system health")

    # Info command
    subparsers.add_parser("info", help="Get collection info")

    args = parser.parse_args()

    # Initialize manager
    manager = RAGManager(
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        openrouter_api_key=args.openrouter_api_key,
        collection_name=args.collection
    )

    # Execute command
    if args.command == "query":
        result = manager.query(
            question=args.question,
            top_k=args.top_k,
            answer_length=args.length,
            answer_style=args.style
        )
        print(json.dumps(result, indent=2))

    elif args.command == "index":
        result = manager.index_directory(
            directory_path=args.directory,
            commit_hash=args.commit_hash,
            force_recreate=args.force_recreate
        )
        print(json.dumps(result, indent=2))

    elif args.command == "health":
        result = manager.health_check()
        print(json.dumps(result, indent=2))

    elif args.command == "info":
        result = manager.get_collection_info()
        print(json.dumps(result, indent=2))

    else:
        parser.print_help()
