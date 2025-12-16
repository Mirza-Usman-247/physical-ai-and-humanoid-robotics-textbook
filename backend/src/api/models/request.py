"""
Pydantic request models for API endpoints.
Defines input validation and serialization for all API requests.
"""

from typing import Optional, Literal
from pydantic import BaseModel, Field, validator
from uuid import UUID


class ChatQueryRequest(BaseModel):
    """
    Request model for POST /chat/query endpoint.

    Attributes:
        query: Natural language question (required, max 500 words ~2000 chars)
        selected_text: User-selected text context (optional, max 2000 chars)
        source_location: Source location of selected text (optional)
        conversation_id: Existing conversation ID for multi-turn (optional)
        answer_length: Desired answer length (optional)
        answer_style: Desired answer style (optional)
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="Natural language question from user"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=2000,
        description="User-selected text from textbook as context"
    )
    source_location: Optional[str] = Field(
        None,
        max_length=500,
        description="Source location of selected text (file path, line number)"
    )
    conversation_id: Optional[UUID] = Field(
        None,
        description="Existing conversation ID for multi-turn dialogue"
    )
    answer_length: Optional[Literal["brief", "medium", "detailed"]] = Field(
        "medium",
        description="Desired answer length"
    )
    answer_style: Optional[Literal["technical", "beginner-friendly"]] = Field(
        "technical",
        description="Desired answer style"
    )

    @validator("query")
    def validate_query_not_empty(cls, v):
        """Ensure query is not just whitespace."""
        if not v or not v.strip():
            raise ValueError("Query cannot be empty or whitespace")
        return v.strip()

    @validator("selected_text")
    def validate_selected_text(cls, v):
        """Sanitize selected text."""
        if v:
            return v.strip()
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is forward kinematics?",
                "selected_text": "Forward kinematics uses DH parameters",
                "source_location": "docs/module-1/chapter-3.md#L67",
                "conversation_id": None,
                "answer_length": "medium",
                "answer_style": "technical"
            }
        }


class FeedbackRequest(BaseModel):
    """
    Request model for POST /chat/feedback endpoint.

    Attributes:
        message_id: ID of the message being rated (required)
        rating: Thumbs up/down rating (required)
        comment: Optional free-text feedback (max 500 chars)
    """
    message_id: UUID = Field(
        ...,
        description="ID of the assistant message being rated"
    )
    rating: Literal["positive", "negative"] = Field(
        ...,
        description="User rating: positive (thumbs up) or negative (thumbs down)"
    )
    comment: Optional[str] = Field(
        None,
        max_length=500,
        description="Optional free-text feedback explaining the rating"
    )

    @validator("comment")
    def validate_comment(cls, v):
        """Sanitize comment."""
        if v:
            return v.strip()
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "message_id": "550e8400-e29b-41d4-a716-446655440000",
                "rating": "positive",
                "comment": "Very helpful explanation with clear examples"
            }
        }


class ConversationHistoryRequest(BaseModel):
    """
    Request model for GET /chat/history endpoint (query parameters).

    Attributes:
        conversation_id: ID of conversation to retrieve
        limit: Maximum number of messages to return (optional)
    """
    conversation_id: UUID = Field(
        ...,
        description="Conversation ID to retrieve history for"
    )
    limit: Optional[int] = Field(
        20,
        ge=1,
        le=100,
        description="Maximum number of messages to return"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
                "limit": 20
            }
        }
