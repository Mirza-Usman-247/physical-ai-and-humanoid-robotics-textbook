"""
Profile service for managing user profiles.

Handles retrieval and creation of user profiles.
"""
from typing import Optional, Dict, Any
from sqlalchemy.orm import Session
from fastapi import HTTPException, status

from src.db.models import UserProfile


def get_profile(db: Session, user_id: str) -> Dict[str, Any]:
    """
    Get user profile by user ID.

    Args:
        db: Database session
        user_id: User UUID string

    Returns:
        Dict containing user profile data:
        {
            "user_id": str,
            "email": str,  # Note: Email is from AuthUser, needs join
            "ai_level": int,
            "ml_level": int,
            "ros_level": int,
            "python_level": int,
            "linux_level": int,
            "has_gpu": bool,
            "has_jetson": bool,
            "has_robot": bool,
            "cloud_only": bool,
            "created_at": str
        }

    Raises:
        HTTPException(404): Profile not found

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     profile = get_profile(
        >>>         db=db,
        >>>         user_id="123e4567-e89b-12d3-a456-426614174000"
        >>>     )
        >>>     print(f"AI Level: {profile['ai_level']}")
    """
    profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    # Get user email (requires join with AuthUser)
    from src.db.models import AuthUser
    user = db.query(AuthUser).filter(AuthUser.id == user_id).first()

    return {
        "user_id": str(profile.user_id),
        "email": user.email if user else "unknown",
        "ai_level": profile.ai_level,
        "ml_level": profile.ml_level,
        "ros_level": profile.ros_level,
        "python_level": profile.python_level,
        "linux_level": profile.linux_level,
        "has_gpu": profile.has_gpu,
        "has_jetson": profile.has_jetson,
        "has_robot": profile.has_robot,
        "cloud_only": profile.cloud_only,
        "created_at": profile.created_at.isoformat()
    }


def create_profile(
    db: Session,
    user_id: str,
    ai_level: int,
    ml_level: int,
    ros_level: int,
    python_level: int,
    linux_level: int,
    has_gpu: bool = False,
    has_jetson: bool = False,
    has_robot: bool = False,
) -> UserProfile:
    """
    Create a new user profile.

    Note: This function is typically called by auth_service.signup_user().
    It's provided separately for flexibility and testing.

    Args:
        db: Database session
        user_id: User UUID string
        ai_level: AI knowledge level (1-5)
        ml_level: ML knowledge level (1-5)
        ros_level: ROS expertise level (1-5)
        python_level: Python proficiency (1-5)
        linux_level: Linux skill (1-5)
        has_gpu: Has GPU hardware (default: False)
        has_jetson: Has Jetson hardware (default: False)
        has_robot: Has robot hardware (default: False)

    Returns:
        UserProfile instance (SQLAlchemy model)

    Raises:
        HTTPException(400): Profile already exists for user
        HTTPException(500): Database error

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     profile = create_profile(
        >>>         db=db,
        >>>         user_id="123e4567-e89b-12d3-a456-426614174000",
        >>>         ai_level=3, ml_level=4, ros_level=2,
        >>>         python_level=5, linux_level=3,
        >>>         has_gpu=True
        >>>     )
    """
    # Check if profile already exists
    existing_profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()
    if existing_profile:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Profile already exists for this user"
        )

    try:
        # Create profile
        profile = UserProfile(
            user_id=user_id,
            ai_level=ai_level,
            ml_level=ml_level,
            ros_level=ros_level,
            python_level=python_level,
            linux_level=linux_level,
            has_gpu=has_gpu,
            has_jetson=has_jetson,
            has_robot=has_robot
        )
        db.add(profile)
        db.commit()
        db.refresh(profile)
        return profile

    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create profile: {str(e)}"
        )


def update_profile(
    db: Session,
    user_id: str,
    ai_level: Optional[int] = None,
    ml_level: Optional[int] = None,
    ros_level: Optional[int] = None,
    python_level: Optional[int] = None,
    linux_level: Optional[int] = None,
    has_gpu: Optional[bool] = None,
    has_jetson: Optional[bool] = None,
    has_robot: Optional[bool] = None,
) -> Dict[str, Any]:
    """
    Update user profile (partial update supported).

    Args:
        db: Database session
        user_id: User UUID string
        ai_level: AI knowledge level (1-5) - optional
        ml_level: ML knowledge level (1-5) - optional
        ros_level: ROS expertise level (1-5) - optional
        python_level: Python proficiency (1-5) - optional
        linux_level: Linux skill (1-5) - optional
        has_gpu: Has GPU hardware - optional
        has_jetson: Has Jetson hardware - optional
        has_robot: Has robot hardware - optional

    Returns:
        Dict containing updated user profile data (same structure as get_profile)

    Raises:
        HTTPException(404): Profile not found
        HTTPException(500): Database error

    Example:
        >>> from src.db.connection import get_db_context
        >>> with get_db_context() as db:
        >>>     updated = update_profile(
        >>>         db=db,
        >>>         user_id="123e4567-e89b-12d3-a456-426614174000",
        >>>         ai_level=4,  # Update AI level only
        >>>         has_gpu=False  # Update GPU flag
        >>>     )
    """
    profile = db.query(UserProfile).filter(UserProfile.user_id == user_id).first()

    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    try:
        # Update fields if provided
        if ai_level is not None:
            profile.ai_level = ai_level
        if ml_level is not None:
            profile.ml_level = ml_level
        if ros_level is not None:
            profile.ros_level = ros_level
        if python_level is not None:
            profile.python_level = python_level
        if linux_level is not None:
            profile.linux_level = linux_level
        if has_gpu is not None:
            profile.has_gpu = has_gpu
        if has_jetson is not None:
            profile.has_jetson = has_jetson
        if has_robot is not None:
            profile.has_robot = has_robot

        db.commit()
        db.refresh(profile)

        # Return updated profile
        return get_profile(db, user_id)

    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update profile: {str(e)}"
        )
