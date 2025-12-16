"""
Token counting utility using tiktoken for OpenRouter API token limits.
Enforces 8,000 input tokens + 2,000 output tokens per request.
"""

import tiktoken
from typing import List, Dict, Any
import logging

from src.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()


class TokenCounter:
    """
    Token counter for managing OpenRouter API token limits.

    Attributes:
        encoding: tiktoken encoding for the model
        max_input_tokens: Maximum input tokens allowed
        max_output_tokens: Maximum output tokens allowed
    """

    def __init__(self, max_input_tokens: int = None, max_output_tokens: int = None):
        """Initialize token counter with tiktoken encoding.

        Args:
            max_input_tokens: Maximum input tokens (defaults to settings value)
            max_output_tokens: Maximum output tokens (defaults to settings value)
        """
        # Use cl100k_base encoding (used by GPT-4, Claude, and most modern models)
        self.encoding = tiktoken.get_encoding("cl100k_base")
        self.max_input_tokens = max_input_tokens if max_input_tokens is not None else settings.max_input_tokens
        self.max_output_tokens = max_output_tokens if max_output_tokens is not None else settings.max_output_tokens

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in a text string.

        Args:
            text: Input text to count tokens

        Returns:
            int: Number of tokens in the text
        """
        if not text:
            return 0
        return len(self.encoding.encode(text))

    def count_tokens_batch(self, texts: List[str]) -> List[int]:
        """
        Count tokens for multiple text strings.

        Args:
            texts: List of text strings

        Returns:
            List[int]: Token counts for each text
        """
        return [self.count_tokens(text) for text in texts]

    def truncate_to_limit(self, text: str, max_tokens: int) -> str:
        """
        Truncate text to fit within token limit.

        Args:
            text: Input text to truncate
            max_tokens: Maximum number of tokens allowed

        Returns:
            str: Truncated text that fits within token limit
        """
        if not text:
            return text

        tokens = self.encoding.encode(text)
        if len(tokens) <= max_tokens:
            return text

        # Truncate tokens and decode back to text
        truncated_tokens = tokens[:max_tokens]
        truncated_text = self.encoding.decode(truncated_tokens)

        logger.warning(f"Text truncated from {len(tokens)} to {max_tokens} tokens")
        return truncated_text

    def calculate_input_budget(
        self,
        query: str,
        selected_text: str = "",
        conversation_history: List[Dict[str, str]] = None,
        system_prompt: str = ""
    ) -> Dict[str, Any]:
        """
        Calculate token budget for RAG query input.

        Args:
            query: User query text
            selected_text: Optional selected text context
            conversation_history: Optional conversation history
            system_prompt: System prompt for the LLM

        Returns:
            Dict containing token counts and remaining budget for context chunks
        """
        conversation_history = conversation_history or []

        query_tokens = self.count_tokens(query)
        selected_tokens = self.count_tokens(selected_text)
        system_tokens = self.count_tokens(system_prompt)

        # Count conversation history tokens
        history_tokens = sum(
            self.count_tokens(msg.get("role", "")) + self.count_tokens(msg.get("content", ""))
            for msg in conversation_history
        )

        # Calculate used tokens
        used_tokens = query_tokens + selected_tokens + system_tokens + history_tokens

        # Calculate remaining budget for context chunks
        remaining_budget = self.max_input_tokens - used_tokens

        return {
            "query_tokens": query_tokens,
            "selected_text_tokens": selected_tokens,
            "system_prompt_tokens": system_tokens,
            "conversation_history_tokens": history_tokens,
            "total_used_tokens": used_tokens,
            "remaining_budget_for_chunks": max(0, remaining_budget),
            "max_input_tokens": self.max_input_tokens,
            "exceeds_limit": used_tokens > self.max_input_tokens
        }

    def truncate_chunks_to_budget(
        self,
        chunks: List[str],
        available_tokens: int
    ) -> List[str]:
        """
        Truncate context chunks to fit within available token budget.
        Prioritizes keeping most relevant chunks (assumes chunks are pre-sorted by relevance).

        Args:
            chunks: List of context chunk texts (sorted by relevance, highest first)
            available_tokens: Available token budget for chunks

        Returns:
            List[str]: Truncated list of chunks that fit within budget
        """
        if available_tokens <= 0:
            logger.warning("No token budget available for context chunks")
            return []

        selected_chunks = []
        used_tokens = 0

        for chunk in chunks:
            chunk_tokens = self.count_tokens(chunk)

            # If adding this chunk exceeds budget, try to fit partial chunk
            if used_tokens + chunk_tokens > available_tokens:
                remaining_tokens = available_tokens - used_tokens
                if remaining_tokens > 50:  # Only add if at least 50 tokens available
                    truncated_chunk = self.truncate_to_limit(chunk, remaining_tokens)
                    selected_chunks.append(truncated_chunk)
                    logger.info(f"Truncated chunk from {chunk_tokens} to {remaining_tokens} tokens")
                break

            selected_chunks.append(chunk)
            used_tokens += chunk_tokens

        logger.info(f"Selected {len(selected_chunks)}/{len(chunks)} chunks using {used_tokens}/{available_tokens} tokens")
        return selected_chunks

    def validate_output_length(self, text: str) -> bool:
        """
        Validate that output text is within token limit.

        Args:
            text: Generated output text

        Returns:
            bool: True if within limit, False otherwise
        """
        tokens = self.count_tokens(text)
        is_valid = tokens <= self.max_output_tokens

        if not is_valid:
            logger.warning(f"Output exceeds max tokens: {tokens}/{self.max_output_tokens}")

        return is_valid


# Global token counter instance
token_counter = TokenCounter()


def get_token_counter() -> TokenCounter:
    """
    Get the global token counter instance.

    Returns:
        TokenCounter: Configured token counter

    Example:
        >>> from src.utils.token_counter import get_token_counter
        >>> counter = get_token_counter()
        >>> tokens = counter.count_tokens("Hello world")
        >>> print(tokens)
    """
    return token_counter
