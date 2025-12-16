"""
RAG Indexer: Ingests textbook content and stores embeddings in Qdrant.
Main indexing logic that orchestrates chunking, embedding, and storage.
"""

import os
import sys
from pathlib import Path
from typing import List, Dict, Any, Optional
import logging
from datetime import datetime
import json

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent.parent))

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

from chunker import MarkdownChunker
from embeddings import EmbeddingGenerator

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RAGIndexer:
    """
    RAG Indexer for textbook content.

    Chunks markdown files, generates embeddings, and stores in Qdrant.
    Supports incremental re-indexing based on git commit hashes.
    """

    def __init__(
        self,
        qdrant_url: str,
        qdrant_api_key: str,
        collection_name: str,
        embedding_generator: EmbeddingGenerator
    ):
        """
        Initialize RAG Indexer.

        Args:
            qdrant_url: Qdrant Cloud cluster URL
            qdrant_api_key: Qdrant API key
            collection_name: Name of the Qdrant collection
            embedding_generator: EmbeddingGenerator instance
        """
        self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = collection_name
        self.embedding_generator = embedding_generator
        self.chunker = MarkdownChunker(chunk_size=512, chunk_overlap=128)

    def create_collection(self, vector_size: int = 1536, force_recreate: bool = False):
        """
        Create Qdrant collection if it doesn't exist.

        Args:
            vector_size: Dimension of embedding vectors (default: 1536 for Qwen)
            force_recreate: If True, delete and recreate collection
        """
        try:
            if force_recreate:
                self.client.delete_collection(collection_name=self.collection_name)
                logger.info(f"Deleted existing collection: {self.collection_name}")

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
            )
            logger.info(f"Created collection: {self.collection_name}")

        except Exception as e:
            if "already exists" in str(e):
                logger.info(f"Collection {self.collection_name} already exists")
            else:
                raise

    def index_file(
        self,
        file_path: str,
        commit_hash: Optional[str] = None,
        module: Optional[str] = None,
        chapter: Optional[str] = None
    ) -> List[str]:
        """
        Index a single markdown file.

        Args:
            file_path: Path to markdown file
            commit_hash: Git commit hash for versioning
            module: Module name (e.g., "module-1-ros2")
            chapter: Chapter name (e.g., "chapter-3")

        Returns:
            List[str]: List of chunk IDs that were indexed
        """
        logger.info(f"Indexing file: {file_path}")

        # Read file content
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()

        # Chunk the content
        chunks = self.chunker.chunk_markdown(content, file_path)
        logger.info(f"Generated {len(chunks)} chunks from {file_path}")

        if not chunks:
            logger.warning(f"No chunks generated from {file_path}")
            return []

        # Generate embeddings
        chunk_texts = [chunk["content"] for chunk in chunks]
        embeddings = self.embedding_generator.generate_embeddings(chunk_texts)

        if len(embeddings) != len(chunks):
            raise ValueError(f"Embedding count mismatch: {len(embeddings)} != {len(chunks)}")

        # Prepare points for Qdrant
        points = []
        chunk_ids = []

        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            chunk_id = f"{file_path}:{chunk['line_start']}"
            chunk_ids.append(chunk_id)

            # Build metadata
            metadata = {
                "file_path": file_path,
                "section_h1": chunk.get("h1", ""),
                "section_h2": chunk.get("h2", ""),
                "section_h3": chunk.get("h3", ""),
                "line_start": chunk["line_start"],
                "line_end": chunk["line_end"],
                "last_indexed": datetime.utcnow().isoformat(),
                "commit_hash": commit_hash or "unknown",
                "version_tag": "textbook-v1-0",
                "token_count": chunk["token_count"],
                "module": module or self._extract_module(file_path),
                "chapter": chapter or self._extract_chapter(file_path),
                "tags": chunk.get("tags", [])
            }

            point = PointStruct(
                id=hash(chunk_id) & 0x7FFFFFFF,  # Positive 32-bit integer
                vector=embedding,
                payload={
                    "chunk_id": chunk_id,
                    "content": chunk["content"],
                    **metadata
                }
            )
            points.append(point)

        # Upload to Qdrant in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            logger.info(f"Uploaded batch {i // batch_size + 1}/{(len(points) + batch_size - 1) // batch_size}")

        logger.info(f"Successfully indexed {len(chunk_ids)} chunks from {file_path}")
        return chunk_ids

    def index_directory(
        self,
        directory_path: str,
        file_pattern: str = "**/*.md",
        commit_hash: Optional[str] = None
    ) -> Dict[str, List[str]]:
        """
        Index all markdown files in a directory.

        Args:
            directory_path: Path to directory containing markdown files
            file_pattern: Glob pattern for matching files (default: **/*.md)
                         Can also pass multiple patterns as comma-separated: "**/*.md,**/*.mdx"
            commit_hash: Git commit hash for versioning

        Returns:
            Dict[str, List[str]]: Mapping of file paths to chunk IDs
        """
        directory = Path(directory_path)

        # Support multiple file patterns (comma-separated)
        markdown_files = []
        patterns = [p.strip() for p in file_pattern.split(',')]
        for pattern in patterns:
            markdown_files.extend(directory.glob(pattern))

        logger.info(f"Found {len(markdown_files)} markdown files in {directory_path}")

        indexed_files = {}
        for file_path in markdown_files:
            try:
                chunk_ids = self.index_file(
                    str(file_path),
                    commit_hash=commit_hash
                )
                indexed_files[str(file_path)] = chunk_ids
            except Exception as e:
                logger.error(f"Failed to index {file_path}: {e}")

        logger.info(f"Successfully indexed {len(indexed_files)}/{len(markdown_files)} files")
        return indexed_files

    def delete_file_chunks(self, file_path: str) -> int:
        """
        Delete all chunks from a specific file.

        Args:
            file_path: Path to file whose chunks should be deleted

        Returns:
            int: Number of chunks deleted
        """
        # Search for all points with this file_path
        search_result = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter={
                "must": [
                    {"key": "file_path", "match": {"value": file_path}}
                ]
            },
            limit=10000
        )

        points = search_result[0]
        if not points:
            logger.info(f"No chunks found for {file_path}")
            return 0

        # Delete points
        point_ids = [point.id for point in points]
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=point_ids
        )

        logger.info(f"Deleted {len(point_ids)} chunks from {file_path}")
        return len(point_ids)

    def _extract_module(self, file_path: str) -> str:
        """Extract module name from file path."""
        # Example: docs/module-1-ros2/chapter-3.md -> module-1-ros2
        parts = Path(file_path).parts
        for part in parts:
            if part.startswith("module-"):
                return part
        return "unknown"

    def _extract_chapter(self, file_path: str) -> str:
        """Extract chapter name from file path."""
        # Example: docs/module-1-ros2/chapter-3.md -> chapter-3
        parts = Path(file_path).parts
        for part in parts:
            if part.startswith("chapter-") or part.startswith("ch-"):
                return part
        return Path(file_path).stem


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RAG Indexer for textbook content")
    parser.add_argument("--qdrant-url", required=True, help="Qdrant Cloud URL")
    parser.add_argument("--qdrant-api-key", required=True, help="Qdrant API key")
    parser.add_argument("--collection", default="physical-ai-textbook", help="Collection name")
    parser.add_argument("--openrouter-api-key", required=True, help="OpenRouter API key")
    parser.add_argument("--directory", required=True, help="Directory to index")
    parser.add_argument("--file-pattern", default="**/*.md", help="File pattern(s) to match (comma-separated for multiple)")
    parser.add_argument("--commit-hash", help="Git commit hash")
    parser.add_argument("--create-collection", action="store_true", help="Create collection if not exists")

    args = parser.parse_args()

    # Initialize embedding generator
    embedding_gen = EmbeddingGenerator(
        api_key=args.openrouter_api_key,
        model="text-embedding-3-small"
    )

    # Initialize indexer
    indexer = RAGIndexer(
        qdrant_url=args.qdrant_url,
        qdrant_api_key=args.qdrant_api_key,
        collection_name=args.collection,
        embedding_generator=embedding_gen
    )

    # Create collection if requested
    if args.create_collection:
        indexer.create_collection()

    # Index directory
    result = indexer.index_directory(
        directory_path=args.directory,
        file_pattern=args.file_pattern,
        commit_hash=args.commit_hash
    )

    print(json.dumps({"indexed_files": len(result), "total_chunks": sum(len(v) for v in result.values())}, indent=2))
