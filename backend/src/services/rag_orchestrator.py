"""
RAG Orchestrator Service
Coordinates RAG pipeline execution between FastAPI backend and Claude Code skills.
"""

import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
import subprocess
import json
import os
from pathlib import Path

from src.config import get_settings
from src.utils.token_counter import TokenCounter
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

logger = logging.getLogger(__name__)
settings = get_settings()


class RAGOrchestrator:
    """
    Orchestrate RAG pipeline using Claude Code skills.

    Bridges FastAPI backend with modular RAG skills:
    - rag-retriever: Semantic search
    - rag-answerer: Answer generation
    - rag-manager: Pipeline coordination
    """

    def __init__(self):
        """Initialize RAG Orchestrator."""
        self.token_counter = TokenCounter()
        self.skills_dir = self._find_skills_directory()

        # Verify skills exist
        self._verify_skills()

        logger.info("RAG Orchestrator initialized")

    def _find_skills_directory(self) -> Path:
        """
        Find the .claude/skills directory.

        Returns:
            Path: Path to skills directory
        """
        # Try from backend directory
        backend_dir = Path(__file__).parent.parent.parent
        project_root = backend_dir.parent
        skills_dir = project_root / ".claude" / "skills"

        if not skills_dir.exists():
            logger.warning(f"Skills directory not found at {skills_dir}")
            # Fallback: try environment variable
            if "RAG_SKILLS_DIR" in os.environ:
                skills_dir = Path(os.environ["RAG_SKILLS_DIR"])

        return skills_dir

    def _verify_skills(self):
        """Verify required skills exist."""
        required_skills = ["rag-manager", "rag-retriever", "rag-answerer"]

        for skill in required_skills:
            skill_path = self.skills_dir / skill
            if not skill_path.exists():
                logger.error(f"Required skill not found: {skill}")
            else:
                logger.debug(f"Found skill: {skill}")

    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10),
        retry=retry_if_exception_type(Exception)
    )
    def _call_rag_manager(
        self,
        command: str,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Call rag-manager skill via subprocess.

        Args:
            command: Command to execute ("query", "index", "health", "info")
            **kwargs: Command-specific arguments

        Returns:
            Dict[str, Any]: RAG manager response

        Raises:
            Exception: If skill execution fails
        """
        manager_script = self.skills_dir / "rag-manager" / "manager.py"

        if not manager_script.exists():
            raise FileNotFoundError(f"rag-manager script not found: {manager_script}")

        # Build command
        cmd = [
            "python",
            str(manager_script),
            "--qdrant-url", settings.qdrant_url,
            "--qdrant-api-key", settings.qdrant_api_key,
            "--openrouter-api-key", settings.openrouter_api_key,
            "--collection", settings.qdrant_collection,
            command
        ]

        # Add command-specific args
        if command == "query":
            cmd.extend([
                "--question", kwargs.get("question", ""),
                "--top-k", str(kwargs.get("top_k", 5)),
                "--length", kwargs.get("answer_length", "medium"),
                "--style", kwargs.get("answer_style", "technical")
            ])
        elif command == "index":
            cmd.extend([
                "--directory", kwargs.get("directory", ""),
                "--commit-hash", kwargs.get("commit_hash", "")
            ])

        logger.debug(f"Executing RAG manager: {command}")

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60,  # 60 second timeout
                check=True
            )

            # Parse JSON response
            response = json.loads(result.stdout)
            return response

        except subprocess.TimeoutExpired:
            logger.error("RAG manager call timed out")
            raise Exception("Request timed out. Please try again.")

        except subprocess.CalledProcessError as e:
            logger.error(f"RAG manager call failed: {e.stderr}")
            raise Exception(f"RAG pipeline error: {e.stderr}")

        except json.JSONDecodeError as e:
            logger.error(f"Failed to parse RAG manager response: {e}")
            raise Exception("Invalid response from RAG pipeline")

    def query(
        self,
        query: str,
        selected_text: Optional[str] = None,
        answer_length: str = "medium",
        answer_style: str = "technical",
        conversation_history: Optional[List[Dict[str, str]]] = None,
        user_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a RAG query.

        Args:
            query: User's question
            selected_text: Optional selected text for context
            answer_length: "brief" | "medium" | "detailed"
            answer_style: "technical" | "beginner-friendly"
            conversation_history: Optional conversation history
            user_id: Optional user ID for logging

        Returns:
            Dict[str, Any]: RAG response with answer, sources, and metadata
        """
        logger.info(f"Processing RAG query (user: {user_id}): {query[:100]}...")

        # Check token limits
        budget = self.token_counter.calculate_input_budget(
            query=query,
            selected_text=selected_text or "",
            conversation_history=conversation_history or []
        )

        if budget["exceeds_limit"]:
            logger.warning(f"Query exceeds token budget: {budget['total_used_tokens']}/{budget['max_input_tokens']}")
            raise ValueError(
                f"Query is too long. Please shorten your question or selected text. "
                f"(Current: {budget['total_used_tokens']} tokens, Max: {budget['max_input_tokens']})"
            )

        logger.debug(f"Token budget: {budget['remaining_budget_for_chunks']} tokens available for context")

        # Calculate top_k based on available tokens
        # Assume average chunk is ~500 tokens
        estimated_top_k = min(budget["remaining_budget_for_chunks"] // 500, 10)
        top_k = max(3, estimated_top_k)  # At least 3, max 10

        logger.debug(f"Using top_k={top_k} based on token budget")

        try:
            # Call rag-manager
            response = self._call_rag_manager(
                command="query",
                question=query,
                top_k=top_k,
                answer_length=answer_length,
                answer_style=answer_style
            )

            # Add user context to metadata
            if user_id:
                response["metadata"]["user_id"] = user_id

            logger.info(f"RAG query completed (confidence: {response.get('confidence', 0):.2f})")
            return response

        except Exception as e:
            logger.error(f"RAG query failed: {e}", exc_info=True)
            raise

    def health_check(self) -> Dict[str, Any]:
        """
        Check health of RAG pipeline.

        Returns:
            Dict[str, Any]: Health status
        """
        try:
            response = self._call_rag_manager(command="health")
            return response
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return {
                "overall_status": "unhealthy",
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat()
            }

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get Qdrant collection information.

        Returns:
            Dict[str, Any]: Collection statistics
        """
        try:
            response = self._call_rag_manager(command="info")
            return response
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            return {
                "status": "error",
                "error": str(e)
            }

    def index_directory(
        self,
        directory_path: str,
        commit_hash: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Index a directory of markdown files.

        Args:
            directory_path: Path to directory
            commit_hash: Optional git commit hash

        Returns:
            Dict[str, Any]: Indexing results
        """
        logger.info(f"Indexing directory: {directory_path}")

        try:
            response = self._call_rag_manager(
                command="index",
                directory=directory_path,
                commit_hash=commit_hash or ""
            )
            return response
        except Exception as e:
            logger.error(f"Indexing failed: {e}")
            return {
                "status": "error",
                "error": str(e)
            }


# Singleton instance
_orchestrator_instance: Optional[RAGOrchestrator] = None


def reset_orchestrator():
    """Reset the singleton orchestrator instance (for testing/reload)."""
    global _orchestrator_instance
    _orchestrator_instance = None


def get_rag_orchestrator(force_reset: bool = False) -> RAGOrchestrator:
    """
    Get singleton RAG Orchestrator instance.

    Args:
        force_reset: If True, reset and recreate the orchestrator

    Returns:
        RAGOrchestrator: Orchestrator instance
    """
    global _orchestrator_instance

    if force_reset:
        reset_orchestrator()

    if _orchestrator_instance is None:
        try:
            _orchestrator_instance = RAGOrchestrator()
        except Exception as e:
            logger.error(f"Failed to create RAG orchestrator: {e}", exc_info=True)
            raise

    return _orchestrator_instance
