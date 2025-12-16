"""
Embedding generation using Qwen via OpenRouter API.
Supports batch processing with retry logic and rate limiting.
"""

import time
import logging
from typing import List
import openai
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

logger = logging.getLogger(__name__)


class EmbeddingGenerator:
    """
    Generate embeddings using Qwen model via OpenRouter API.

    Supports batch processing, automatic retries, and rate limiting.
    """

    def __init__(
        self,
        api_key: str,
        model: str = "text-embedding-3-small",
        batch_size: int = 100,
        max_retries: int = 3
    ):
        """
        Initialize embedding generator.

        Args:
            api_key: OpenRouter API key
            model: Model ID for embeddings (default: text-embedding-3-small)
            batch_size: Number of texts to embed in one API call (default: 100)
            max_retries: Maximum number of retry attempts (default: 3)
        """
        self.api_key = api_key
        self.model = model
        self.batch_size = batch_size
        self.max_retries = max_retries

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
    def _generate_embeddings_with_retry(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings with automatic retry logic.

        Args:
            texts: List of text strings to embed

        Returns:
            List[List[float]]: List of embedding vectors (1536 dimensions each)

        Raises:
            openai.OpenAIError: If API call fails after retries
        """
        try:
            response = self.client.embeddings.create(
                model=self.model,
                input=texts
            )

            # Extract embeddings from response
            embeddings = [data.embedding for data in response.data]

            logger.debug(f"Generated {len(embeddings)} embeddings")
            return embeddings

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
            logger.error(f"Unexpected error generating embeddings: {e}")
            raise

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Automatically batches requests to respect API limits and improve efficiency.

        Args:
            texts: List of text strings to embed (can be any length)

        Returns:
            List[List[float]]: List of embedding vectors (1536 dimensions each)

        Example:
            >>> generator = EmbeddingGenerator(api_key="sk-or-v1-...")
            >>> texts = ["Hello world", "How are you?"]
            >>> embeddings = generator.generate_embeddings(texts)
            >>> len(embeddings)
            2
            >>> len(embeddings[0])
            1536
        """
        if not texts:
            logger.warning("Empty text list provided")
            return []

        logger.info(f"Generating embeddings for {len(texts)} texts")

        all_embeddings = []

        # Process in batches
        for i in range(0, len(texts), self.batch_size):
            batch = texts[i:i + self.batch_size]
            batch_num = (i // self.batch_size) + 1
            total_batches = (len(texts) + self.batch_size - 1) // self.batch_size

            logger.info(f"Processing batch {batch_num}/{total_batches} ({len(batch)} texts)")

            try:
                batch_embeddings = self._generate_embeddings_with_retry(batch)
                all_embeddings.extend(batch_embeddings)

                # Rate limiting: small delay between batches to avoid hitting limits
                if i + self.batch_size < len(texts):
                    time.sleep(0.5)  # 500ms delay between batches

            except Exception as e:
                logger.error(f"Failed to generate embeddings for batch {batch_num}: {e}")
                raise

        logger.info(f"Successfully generated {len(all_embeddings)} embeddings")
        return all_embeddings

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of embeddings produced by this model.

        Returns:
            int: Embedding dimension (1536 for Qwen)
        """
        return 1536

    def validate_embeddings(self, embeddings: List[List[float]]) -> bool:
        """
        Validate that embeddings have the correct shape and values.

        Args:
            embeddings: List of embedding vectors to validate

        Returns:
            bool: True if valid, False otherwise
        """
        if not embeddings:
            logger.warning("Empty embeddings list")
            return False

        expected_dim = self.get_embedding_dimension()

        for i, embedding in enumerate(embeddings):
            if len(embedding) != expected_dim:
                logger.error(f"Embedding {i} has wrong dimension: {len(embedding)} != {expected_dim}")
                return False

            if not all(isinstance(x, (int, float)) for x in embedding):
                logger.error(f"Embedding {i} contains non-numeric values")
                return False

        logger.debug(f"Validated {len(embeddings)} embeddings")
        return True


if __name__ == "__main__":
    import argparse
    import json

    parser = argparse.ArgumentParser(description="Generate embeddings for text")
    parser.add_argument("--api-key", required=True, help="OpenRouter API key")
    parser.add_argument("--model", default="qwen/qwen-2-embedding", help="Model ID")
    parser.add_argument("--text", required=True, nargs="+", help="Text to embed")
    parser.add_argument("--batch-size", type=int, default=100, help="Batch size")

    args = parser.parse_args()

    # Initialize generator
    generator = EmbeddingGenerator(
        api_key=args.api_key,
        model=args.model,
        batch_size=args.batch_size
    )

    # Generate embeddings
    embeddings = generator.generate_embeddings(args.text)

    # Validate
    if generator.validate_embeddings(embeddings):
        print(json.dumps({
            "success": True,
            "count": len(embeddings),
            "dimension": len(embeddings[0]) if embeddings else 0,
            "sample": embeddings[0][:5] if embeddings else []  # First 5 values of first embedding
        }, indent=2))
    else:
        print(json.dumps({"success": False, "error": "Validation failed"}, indent=2))
