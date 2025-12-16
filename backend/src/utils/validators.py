"""
Input validation and sanitization utilities.
Prevents prompt injection, XSS, and other security vulnerabilities.
"""

import re
from typing import Optional, List
import logging
import html

logger = logging.getLogger(__name__)


class InputValidator:
    """
    Input validation and sanitization for user queries and text selections.
    """

    # Suspicious patterns that might indicate prompt injection
    SUSPICIOUS_PATTERNS = [
        r"ignore previous instructions",
        r"disregard all prior",
        r"forget everything",
        r"new instructions:",
        r"system:",
        r"<\|im_start\|>",
        r"<\|im_end\|>",
        r"###\s*Instruction",
        r"###\s*System",
        r"{{.*}}",  # Template injection attempts
        r"\[INST\]",
        r"\[/INST\]",
    ]

    # Maximum lengths for different input types
    MAX_QUERY_LENGTH = 2000  # ~500 words
    MAX_SELECTED_TEXT_LENGTH = 2000  # As per spec
    MAX_FEEDBACK_LENGTH = 500
    MAX_SESSION_ID_LENGTH = 255

    def __init__(self):
        """Initialize validator with compiled regex patterns."""
        self.suspicious_pattern = re.compile(
            "|".join(self.SUSPICIOUS_PATTERNS),
            re.IGNORECASE | re.MULTILINE
        )

    def sanitize_text(self, text: str) -> str:
        """
        Sanitize text by removing potentially harmful characters.

        Args:
            text: Input text to sanitize

        Returns:
            str: Sanitized text
        """
        if not text:
            return ""

        # HTML escape to prevent XSS
        sanitized = html.escape(text)

        # Remove null bytes
        sanitized = sanitized.replace("\x00", "")

        # Normalize whitespace (but preserve newlines)
        sanitized = re.sub(r"[ \t]+", " ", sanitized)

        return sanitized.strip()

    def detect_prompt_injection(self, text: str) -> List[str]:
        """
        Detect potential prompt injection attempts.

        Args:
            text: Text to check for injection patterns

        Returns:
            List[str]: List of detected suspicious patterns
        """
        if not text:
            return []

        matches = self.suspicious_pattern.findall(text.lower())
        return list(set(matches))  # Return unique matches

    def validate_query(self, query: str) -> tuple[bool, Optional[str]]:
        """
        Validate user query input.

        Args:
            query: User query text

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str])
        """
        if not query or not query.strip():
            return False, "Query cannot be empty"

        if len(query) > self.MAX_QUERY_LENGTH:
            return False, f"Query exceeds maximum length of {self.MAX_QUERY_LENGTH} characters"

        # Check for prompt injection
        suspicious_patterns = self.detect_prompt_injection(query)
        if suspicious_patterns:
            logger.warning(f"Suspicious patterns detected in query: {suspicious_patterns}")
            return False, "Query contains suspicious patterns that may be attempting prompt injection"

        return True, None

    def validate_selected_text(self, selected_text: str) -> tuple[bool, Optional[str]]:
        """
        Validate user-selected text.

        Args:
            selected_text: Selected text from textbook

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str])
        """
        if not selected_text:
            return True, None  # Selected text is optional

        if len(selected_text) > self.MAX_SELECTED_TEXT_LENGTH:
            return False, f"Selected text exceeds maximum length of {self.MAX_SELECTED_TEXT_LENGTH} characters"

        return True, None

    def validate_feedback(self, comment: str) -> tuple[bool, Optional[str]]:
        """
        Validate feedback comment.

        Args:
            comment: User feedback comment

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str])
        """
        if not comment:
            return True, None  # Comment is optional

        if len(comment) > self.MAX_FEEDBACK_LENGTH:
            return False, f"Feedback comment exceeds maximum length of {self.MAX_FEEDBACK_LENGTH} characters"

        return True, None

    def validate_session_id(self, session_id: str) -> tuple[bool, Optional[str]]:
        """
        Validate session ID format.

        Args:
            session_id: Session identifier

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str])
        """
        if not session_id or not session_id.strip():
            return False, "Session ID cannot be empty"

        if len(session_id) > self.MAX_SESSION_ID_LENGTH:
            return False, f"Session ID exceeds maximum length of {self.MAX_SESSION_ID_LENGTH} characters"

        # Session ID should be alphanumeric with optional hyphens/underscores
        if not re.match(r"^[a-zA-Z0-9_-]+$", session_id):
            return False, "Session ID contains invalid characters"

        return True, None

    def sanitize_and_validate_query(self, query: str) -> tuple[bool, Optional[str], str]:
        """
        Sanitize and validate query in one step.

        Args:
            query: Raw user query

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str], sanitized_query: str)
        """
        sanitized = self.sanitize_text(query)
        is_valid, error_message = self.validate_query(sanitized)
        return is_valid, error_message, sanitized

    def sanitize_and_validate_selected_text(
        self,
        selected_text: str
    ) -> tuple[bool, Optional[str], str]:
        """
        Sanitize and validate selected text in one step.

        Args:
            selected_text: Raw selected text

        Returns:
            tuple: (is_valid: bool, error_message: Optional[str], sanitized_text: str)
        """
        sanitized = self.sanitize_text(selected_text) if selected_text else ""
        is_valid, error_message = self.validate_selected_text(sanitized)
        return is_valid, error_message, sanitized


# Global validator instance
input_validator = InputValidator()


def get_input_validator() -> InputValidator:
    """
    Get the global input validator instance.

    Returns:
        InputValidator: Configured input validator

    Example:
        >>> from src.utils.validators import get_input_validator
        >>> validator = get_input_validator()
        >>> is_valid, error, sanitized = validator.sanitize_and_validate_query("Hello?")
        >>> print(is_valid, sanitized)
    """
    return input_validator
