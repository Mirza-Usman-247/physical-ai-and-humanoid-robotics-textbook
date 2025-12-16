"""
Chat API endpoints for RAG-powered Q&A.
Implements FR-001 through FR-025.
"""

from fastapi import APIRouter, Depends, HTTPException, Header, status
from sqlalchemy.orm import Session
from typing import Optional
from uuid import UUID
import logging

from src.api.models.request import ChatQueryRequest, FeedbackRequest, ConversationHistoryRequest
from src.api.models.response import ChatQueryResponse, ConversationHistoryResponse, MessageResponse, CitationSource
from src.db.connection import get_db
from src.db.models import Message, MessageRole, FeedbackRating
from src.services.rag_orchestrator import get_rag_orchestrator
from src.services.conversation_service import get_conversation_service
from src.utils.validators import InputValidator
from src.config import get_settings

logger = logging.getLogger(__name__)
settings = get_settings()

router = APIRouter()


@router.post(
    "/chat/query",
    response_model=ChatQueryResponse,
    status_code=status.HTTP_200_OK,
    summary="Ask a question about the textbook",
    description="Submit a question to the RAG chatbot and receive an answer with citations"
)
async def chat_query(
    request: ChatQueryRequest,
    x_session_id: Optional[str] = Header(None),
    db: Session = Depends(get_db)
):
    """
    Process a chat query using RAG pipeline.

    Workflow:
    1. Validate and sanitize input
    2. Get or create user from session
    3. Get or create conversation
    4. Retrieve conversation history
    5. Execute RAG pipeline
    6. Store user query and assistant response
    7. Return response with citations

    Args:
        request: ChatQueryRequest with query, selected_text, preferences
        x_session_id: Session ID from header
        db: Database session

    Returns:
        ChatQueryResponse: Answer with inline citations and sources

    Raises:
        HTTPException: If validation fails or RAG pipeline errors

    Example:
        POST /chat/query
        {
            "query": "What is forward kinematics?",
            "selected_text": null,
            "conversation_id": null,
            "answer_length": "medium",
            "answer_style": "technical"
        }
    """
    logger.info(f"Processing chat query: {request.query[:100]}...")

    # Get services
    orchestrator = get_rag_orchestrator()
    conversation_service = get_conversation_service()
    validator = InputValidator()

    # Validate and sanitize query
    is_valid, error_msg, sanitized_query = validator.sanitize_and_validate_query(request.query)
    if not is_valid:
        logger.warning(f"Invalid query: {error_msg}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=error_msg
        )

    # Validate selected text if provided
    sanitized_selected = None
    if request.selected_text:
        is_valid, error_msg, sanitized_selected = validator.sanitize_and_validate_selected_text(
            request.selected_text
        )
        if not is_valid:
            logger.warning(f"Invalid selected text: {error_msg}")
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=error_msg
            )

    try:
        # Get or create user
        session_id = x_session_id or "default_session"
        user = conversation_service.get_or_create_user(db, session_id)

        # Get or create conversation
        if request.conversation_id:
            conversation = conversation_service.get_conversation(db, request.conversation_id)
            if not conversation:
                raise HTTPException(
                    status_code=status.HTTP_404_NOT_FOUND,
                    detail=f"Conversation {request.conversation_id} not found"
                )
        else:
            # Create new conversation
            conversation = conversation_service.create_conversation(
                db=db,
                user_id=user.user_id,
                title=sanitized_query[:100]  # Use query as title
            )

        # Get conversation history for context
        conversation_history = conversation_service.get_conversation_history_for_rag(
            db=db,
            conversation_id=conversation.conversation_id,
            max_turns=2  # Last 2 turns
        )

        # Store user message
        user_message = conversation_service.add_message(
            db=db,
            conversation_id=conversation.conversation_id,
            role=MessageRole.USER,
            content_text=sanitized_query,
            selected_context={
                "text": sanitized_selected,
                "location": request.source_location
            } if sanitized_selected else None
        )

        # Execute RAG pipeline
        rag_response = orchestrator.query(
            query=sanitized_query,
            selected_text=sanitized_selected,
            answer_length=request.answer_length,
            answer_style=request.answer_style,
            conversation_history=conversation_history,
            user_id=str(user.user_id)
        )

        # Store assistant message
        assistant_message = conversation_service.add_message(
            db=db,
            conversation_id=conversation.conversation_id,
            role=MessageRole.ASSISTANT,
            content_text=rag_response["answer"],
            retrieved_chunks=rag_response.get("sources", []),
            citations=rag_response.get("citations", []),
            confidence=rag_response.get("confidence"),
            response_time_ms=rag_response["metadata"].get("response_time_ms")
        )

        # Build citation sources
        citation_sources = [
            CitationSource(
                id=src["id"],
                file_path=src["file_path"],
                line_range=src["line_range"],
                section=src["section"],
                score=src["score"]
            )
            for src in rag_response.get("sources", [])
        ]

        # Build response
        response = ChatQueryResponse(
            message_id=assistant_message.message_id,
            conversation_id=conversation.conversation_id,
            answer=rag_response["answer"],
            sources=citation_sources,
            follow_up_suggestions=rag_response.get("follow_up_suggestions", []),
            confidence=rag_response.get("confidence", 0.0),
            response_time_ms=rag_response["metadata"].get("response_time_ms", 0),
            timestamp=assistant_message.timestamp
        )

        logger.info(
            f"Chat query completed (conversation: {conversation.conversation_id}, "
            f"confidence: {response.confidence:.2f}, time: {response.response_time_ms}ms)"
        )

        return response

    except ValueError as e:
        # Token limit or validation errors
        logger.warning(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

    except Exception as e:
        logger.error(f"Chat query failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while processing your question. Please try again."
        )


@router.get(
    "/chat/history",
    response_model=ConversationHistoryResponse,
    status_code=status.HTTP_200_OK,
    summary="Get conversation history",
    description="Retrieve messages from a conversation"
)
async def get_conversation_history(
    conversation_id: UUID,
    limit: int = 50,
    offset: int = 0,
    db: Session = Depends(get_db)
):
    """
    Get conversation history.

    Args:
        conversation_id: Conversation ID
        limit: Maximum number of messages (default: 50, max: 100)
        offset: Pagination offset
        db: Database session

    Returns:
        ConversationHistoryResponse: Conversation with messages

    Raises:
        HTTPException: If conversation not found

    Example:
        GET /chat/history?conversation_id=123e4567-e89b-12d3-a456-426614174000&limit=20
    """
    logger.info(f"Getting conversation history: {conversation_id}")

    conversation_service = get_conversation_service()

    # Validate limit
    if limit > 100:
        limit = 100

    # Get conversation
    conversation = conversation_service.get_conversation(db, conversation_id)
    if not conversation:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Conversation {conversation_id} not found"
        )

    # Get messages
    messages = conversation_service.get_conversation_messages(
        db=db,
        conversation_id=conversation_id,
        limit=limit,
        offset=offset
    )

    # Build response
    message_responses = [
        MessageResponse(
            message_id=msg.message_id,
            role=msg.role.value,
            content=msg.content_text,
            timestamp=msg.timestamp,
            response_time_ms=msg.response_time_ms
        )
        for msg in messages
    ]

    response = ConversationHistoryResponse(
        conversation_id=conversation.conversation_id,
        total_messages=conversation.message_count,
        messages=message_responses,
        start_timestamp=conversation.start_timestamp
    )

    logger.info(f"Returned {len(message_responses)} messages for conversation {conversation_id}")
    return response


@router.post(
    "/chat/feedback",
    status_code=status.HTTP_201_CREATED,
    summary="Submit feedback",
    description="Submit feedback for an assistant message"
)
async def submit_feedback(
    request: FeedbackRequest,
    db: Session = Depends(get_db)
):
    """
    Submit feedback for a message.

    Args:
        request: FeedbackRequest with message_id, rating, and optional comment
        db: Database session

    Returns:
        dict: Feedback confirmation

    Raises:
        HTTPException: If message not found or validation fails

    Example:
        POST /chat/feedback
        {
            "message_id": "123e4567-e89b-12d3-a456-426614174000",
            "rating": "positive",
            "comment": "Very helpful explanation!"
        }
    """
    logger.info(f"Submitting feedback for message: {request.message_id}")

    conversation_service = get_conversation_service()
    validator = InputValidator()

    # Validate comment if provided
    if request.comment:
        is_valid, error_msg, sanitized_comment = validator.sanitize_and_validate_feedback(
            request.comment
        )
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=error_msg
            )
    else:
        sanitized_comment = None

    try:
        # Check if message exists
        message = db.query(Message).filter(Message.message_id == request.message_id).first()
        if not message:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Message {request.message_id} not found"
            )

        # Create feedback
        feedback = conversation_service.add_feedback(
            db=db,
            message_id=request.message_id,
            rating=FeedbackRating(request.rating),
            comment=sanitized_comment
        )

        logger.info(f"Feedback submitted: {feedback.feedback_id}")

        return {
            "status": "success",
            "feedback_id": str(feedback.feedback_id),
            "message": "Thank you for your feedback!"
        }

    except Exception as e:
        logger.error(f"Failed to submit feedback: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit feedback. Please try again."
        )
