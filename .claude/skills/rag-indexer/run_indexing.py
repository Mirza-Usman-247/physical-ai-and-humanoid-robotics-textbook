#!/usr/bin/env python3
"""
Run RAG Indexing
Index textbook content into Qdrant for RAG chatbot.
"""

import os
import sys
import argparse
import subprocess
import json
from pathlib import Path

def get_git_commit_hash():
    """Get current git commit hash."""
    try:
        result = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            capture_output=True,
            text=True,
            check=True
        )
        return result.stdout.strip()
    except:
        return "unknown"

def run_indexing(
    qdrant_url: str,
    qdrant_api_key: str,
    openrouter_api_key: str,
    directory: str,
    collection: str = "physical-ai-textbook",
    create_collection: bool = False
):
    """
    Run the indexing pipeline.

    Args:
        qdrant_url: Qdrant Cloud URL
        qdrant_api_key: Qdrant API key
        openrouter_api_key: OpenRouter API key
        directory: Directory to index
        collection: Collection name
        create_collection: Whether to create collection
    """
    # Get current directory
    script_dir = Path(__file__).parent
    indexer_script = script_dir / "indexer.py"

    if not indexer_script.exists():
        print(f"Error: indexer.py not found at {indexer_script}")
        sys.exit(1)

    # Get git commit hash
    commit_hash = get_git_commit_hash()
    print(f"Git commit: {commit_hash}")

    # Build command
    cmd = [
        sys.executable,
        str(indexer_script),
        "--qdrant-url", qdrant_url,
        "--qdrant-api-key", qdrant_api_key,
        "--openrouter-api-key", openrouter_api_key,
        "--collection", collection,
        "--directory", directory,
        "--commit-hash", commit_hash
    ]

    if create_collection:
        cmd.append("--create-collection")

    print(f"\nRunning indexer...")
    print(f"Directory: {directory}")
    print(f"Collection: {collection}")
    print(f"Create collection: {create_collection}\n")

    # Run indexer
    try:
        result = subprocess.run(cmd, check=True, capture_output=False, text=True)
        print("\n✅ Indexing completed successfully!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Indexing failed with exit code {e.returncode}")
        return False

def verify_indexing(qdrant_url: str, qdrant_api_key: str, collection: str):
    """
    Verify indexing results.

    Args:
        qdrant_url: Qdrant Cloud URL
        qdrant_api_key: Qdrant API key
        collection: Collection name
    """
    print(f"\nVerifying collection: {collection}...")

    try:
        from qdrant_client import QdrantClient

        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        collection_info = client.get_collection(collection_name=collection)

        print(f"\n✅ Collection verified!")
        print(f"   - Vectors count: {collection_info.vectors_count}")
        print(f"   - Points count: {collection_info.points_count}")
        print(f"   - Status: {collection_info.status}")

        # Estimate chunks
        print(f"\n   Expected: ~700-900 chunks for full textbook (34 files: 21 chapters + 5 module indexes + 6 docs)")
        print(f"   Current: {collection_info.points_count} chunks")

        if collection_info.points_count > 0:
            print(f"\n   ✅ Collection is populated and ready!")
        else:
            print(f"\n   ⚠️  Collection is empty. Run indexing first.")

        return True
    except Exception as e:
        print(f"\n❌ Verification failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Index textbook content for RAG chatbot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Index with environment variables
  export QDRANT_URL="https://your-cluster.qdrant.io"
  export QDRANT_API_KEY="your-key"
  export OPENROUTER_API_KEY="sk-or-v1-your-key"
  python run_indexing.py --directory docs

  # Index with explicit parameters
  python run_indexing.py \\
    --qdrant-url "https://your-cluster.qdrant.io" \\
    --qdrant-api-key "your-key" \\
    --openrouter-api-key "sk-or-v1-your-key" \\
    --directory docs \\
    --create-collection

  # Verify existing collection
  python run_indexing.py --verify-only
        """
    )

    parser.add_argument(
        "--qdrant-url",
        default=os.getenv("QDRANT_URL"),
        help="Qdrant Cloud URL (or set QDRANT_URL env var)"
    )
    parser.add_argument(
        "--qdrant-api-key",
        default=os.getenv("QDRANT_API_KEY"),
        help="Qdrant API key (or set QDRANT_API_KEY env var)"
    )
    parser.add_argument(
        "--openrouter-api-key",
        default=os.getenv("OPENROUTER_API_KEY"),
        help="OpenRouter API key (or set OPENROUTER_API_KEY env var)"
    )
    parser.add_argument(
        "--directory",
        default="docs",
        help="Directory to index (default: docs)"
    )
    parser.add_argument(
        "--collection",
        default="physical-ai-textbook",
        help="Collection name (default: physical-ai-textbook)"
    )
    parser.add_argument(
        "--create-collection",
        action="store_true",
        help="Create collection if it doesn't exist"
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Only verify collection, don't index"
    )

    args = parser.parse_args()

    # Validate required parameters
    if not args.verify_only:
        if not args.qdrant_url:
            print("Error: --qdrant-url is required (or set QDRANT_URL env var)")
            sys.exit(1)
        if not args.qdrant_api_key:
            print("Error: --qdrant-api-key is required (or set QDRANT_API_KEY env var)")
            sys.exit(1)
        if not args.openrouter_api_key:
            print("Error: --openrouter-api-key is required (or set OPENROUTER_API_KEY env var)")
            sys.exit(1)

    # Verify only mode
    if args.verify_only:
        if not args.qdrant_url or not args.qdrant_api_key:
            print("Error: --qdrant-url and --qdrant-api-key required for verification")
            sys.exit(1)

        success = verify_indexing(
            args.qdrant_url,
            args.qdrant_api_key,
            args.collection
        )
        sys.exit(0 if success else 1)

    # Run indexing
    success = run_indexing(
        args.qdrant_url,
        args.qdrant_api_key,
        args.openrouter_api_key,
        args.directory,
        args.collection,
        args.create_collection
    )

    # Verify if successful
    if success:
        verify_indexing(
            args.qdrant_url,
            args.qdrant_api_key,
            args.collection
        )

    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
