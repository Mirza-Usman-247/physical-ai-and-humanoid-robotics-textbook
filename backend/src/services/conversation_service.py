"""
Conversation Service
Handles database operations for conversations, messages, and feedback.
"""

import logging
from typing import List, Dict, Any, Optional
from uuid import UUID, uuid4
from datetime import datetime
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError

from src.db.models import User, Conversation, Message, Feedback, ConversationStatus, MessageRole, FeedbackRating
from src.db.connection import get_db

logger = logging.getLogger(__name__)


class ConversationService:
    """
    Service for managing conversations, messages, and feedback.

    Handles:
    - User creation and retrieval
    - Conversation lifecycle
    - Message storage and retrieval
    - Feedback collection
    """

    def get_or_create_user(self, db: Session, session_id: str) -> User:
        """
        Get existing user by session ID or create new one.

        Args:
            db: Database session
            session_id: Unique session identifier

        Returns:
            User: User instance
        """
        user = db.query(User).filter(User.session_id == session_id).first()

        if not user:
            user = User(
                user_id=uuid4(),
                session_id=session_id
            )
            db.add(user)
            db.commit()
            db.refresh(user)
            logger.info(f"Created new user: {user.user_id}")
            logger.debug(f"Found existing user: {user.user_id}")

        return user

    def create_conversation(
        self,
        db: Session,
        user_id: UUID,
        title: Optional[str] = None
    ) -> Conversation:
        """
        Create a new conversation.

        Args:
            db: Database session
            user_id: User ID
            title: Optional conversation title

        Returns:
            Conversation: Created conversation
        """
        conversation = Conversation(
            conversation_id=uuid4(),
            user_id=user_id,
            message_count=0,
            status=ConversationStatus.ACTIVE
        )

        db.add(conversation)
        db.commit()
        db.refresh(conversation)

        logger.info(f"Created conversation: {conversation.conversation_id}")
        return conversation

    def get_conversation(
        self,
        db: Session,
        conversation_id: UUID
    ) -> Optional[Conversation]:
        """
        Get conversation by ID.

        Args:
            db: Database session
            conversation_id: Conversation ID

        Returns:
            Optional[Conversation]: Conversation if found
        """
        conversation = db.query(Conversation).filter(
            Conversation.conversation_id == conversation_id
        ).first()

        return conversation

    def get_user_conversations(
        self,
        db: Session,
        user_id: UUID,
        limit: int = 50,
        offset: int = 0
    ) -> List[Conversation]:
        """
        Get user's conversations.

        Args:
            db: Database session
            user_id: User ID
            limit: Maximum number of conversations
            offset: Pagination offset

        Returns:
            List[Conversation]: List of conversations
        """
        conversations = db.query(Conversation).filter(
            Conversation.user_id == user_id
        ).order_by(
            Conversation.updated_at.desc()
        ).limit(limit).offset(offset).all()

        return conversations

    def add_message(
        self,
        db: Session,
        conversation_id: UUID,
        role: MessageRole,
        content_text: str,
        selected_context: Optional[Dict[str, Any]] = None,
        retrieved_chunks: Optional[List[Dict[str, Any]]] = None,
        citations: Optional[List[int]] = None,
        confidence: Optional[float] = None,
        response_time_ms: Optional[int] = None
    ) -> Message:
        """
        Add a message to a conversation.

        Args:
            db: Database session
            conversation_id: Conversation ID
            role: Message role (user | assistant)
            content_text: Message content
            selected_context: Optional selected text context
            retrieved_chunks: Optional retrieved chunks (for assistant messages)
            citations: Optional citation numbers (for assistant messages)
            confidence: Optional confidence score (for assistant messages)
            response_time_ms: Optional response time (for assistant messages)

        Returns:
            Message: Created message
        """
        message = Message(
            message_id=uuid4(),
            conversation_id=conversation_id,
            role=role,
            content_text=content_text,
            selected_context=selected_context,
            retrieved_chunks=retrieved_chunks,
            citations=citations,
            response_time_ms=response_time_ms
        )

        db.add(message)

        # Update conversation
        conversation = self.get_conversation(db, conversation_id)
        if conversation:
            conversation.message_count += 1
            conversation.updated_at = datetime.utcnow()

        db.commit()
        db.refresh(message)

        logger.debug(f"Added message: {message.message_id} (role: {role})")
        return message

    def get_conversation_messages(
        self,
        db: Session,
        conversation_id: UUID,
        limit: int = 50,
        offset: int = 0
    ) -> List[Message]:
        """
        Get messages in a conversation.

        Args:
            db: Database session
            conversation_id: Conversation ID
            limit: Maximum number of messages
            offset: Pagination offset

        Returns:
            List[Message]: List of messages
        """
        messages = db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(
            Message.timestamp.asc()
        ).limit(limit).offset(offset).all()

        return messages

    def add_feedback(
        self,
        db: Session,
        message_id: UUID,
        rating: FeedbackRating,
        comment: Optional[str] = None
    ) -> Feedback:
        """
        Add feedback to a message.

        Args:
            db: Database session
            message_id: Message ID
            rating: Feedback rating (positive | negative)
            comment: Optional feedback comment

        Returns:
            Feedback: Created feedback
        """
        feedback = Feedback(
            feedback_id=uuid4(),
            message_id=message_id,
            rating=rating,
            comment=comment
        )

        db.add(feedback)
        db.commit()
        db.refresh(feedback)

        logger.info(f"Added feedback: {feedback.feedback_id} (rating: {rating})")
        return feedback

    def get_message_feedback(
        self,
        db: Session,
        message_id: UUID
    ) -> Optional[Feedback]:
        """
        Get feedback for a message.

        Args:
            db: Database session
            message_id: Message ID

        Returns:
            Optional[Feedback]: Feedback if exists
        """
        feedback = db.query(Feedback).filter(
            Feedback.message_id == message_id
        ).first()

        return feedback

    def archive_conversation(
        self,
        db: Session,
        conversation_id: UUID
    ) -> Optional[Conversation]:
        """
        Archive a conversation.

        Args:
            db: Database session
            conversation_id: Conversation ID

        Returns:
            Optional[Conversation]: Updated conversation
        """
        conversation = self.get_conversation(db, conversation_id)

        if conversation:
            conversation.status = ConversationStatus.ARCHIVED
            conversation.updated_at = datetime.utcnow()
            db.commit()
            db.refresh(conversation)
            logger.info(f"Archived conversation: {conversation_id}")

        return conversation

    def delete_conversation(
        self,
        db: Session,
        conversation_id: UUID
    ) -> bool:
        """
        Delete a conversation and all its messages.

        Args:
            db: Database session
            conversation_id: Conversation ID

        Returns:
            bool: True if deleted successfully
        """
        try:
            # Delete messages first (due to foreign key)
            db.query(Message).filter(
                Message.conversation_id == conversation_id
            ).delete()

            # Delete conversation
            db.query(Conversation).filter(
                Conversation.conversation_id == conversation_id
            ).delete()

            db.commit()
            logger.info(f"Deleted conversation: {conversation_id}")
            return True

        except SQLAlchemyError as e:
            logger.error(f"Failed to delete conversation: {e}")
            db.rollback()
            return False

    def get_conversation_history_for_rag(
        self,
        db: Session,
        conversation_id: UUID,
        max_turns: int = 2
    ) -> List[Dict[str, str]]:
        """
        Get conversation history formatted for RAG pipeline.

        Args:
            db: Database session
            conversation_id: Conversation ID
            max_turns: Maximum number of turns (user+assistant pairs)

        Returns:
            List[Dict[str, str]]: List of messages with role and content
        """
        # Get recent messages
        messages = db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(
            Message.timestamp.desc()
        ).limit(max_turns * 2).all()

        # Reverse to get chronological order
        messages.reverse()

        # Format for RAG pipeline
        history = [
            {
                "role": msg.role.value,
                "content": msg.content_text
            }
            for msg in messages
        ]

        return history


# Singleton instance
_service_instance: Optional[ConversationService] = None


def get_conversation_service() -> ConversationService:
    """
    Get singleton Conversation Service instance.

    Returns:
        ConversationService: Service instance
    """
    global _service_instance

    if _service_instance is None:
        _service_instance = ConversationService()

    return _service_instance
