"""
T020: Integration test for full signup flow.

Tests the end-to-end signup process including database persistence.
"""
import pytest
from fastapi.testclient import TestClient
from sqlalchemy.orm import Session
from fastapi import status

from src.db.models import AuthUser, UserProfile


def test_signup_creates_user_and_profile(test_client: TestClient, test_db_session: Session, test_settings):
    """
    Test that signup creates both AuthUser and UserProfile records in database.

    Acceptance Criteria:
    - Signup request with valid data returns 201 Created
    - AuthUser record created in database with:
      - Email matching request
      - Password hashed (not plain text)
      - is_active = True
      - created_at and updated_at timestamps set
    - UserProfile record created in database with:
      - user_id matching AuthUser.id
      - All skill levels matching request
      - All hardware flags matching request
      - cloud_only computed correctly (True if no hardware)
      - created_at and updated_at timestamps set
    - Response tokens are valid JWTs
    - Password is hashed using bcrypt (starts with $2b$)
    """
    signup_data = {
        "email": "integration@example.com",
        "password": "IntegrationTest123",
        "ai_level": 4,
        "ml_level": 3,
        "ros_level": 2,
        "python_level": 5,
        "linux_level": 4,
        "has_gpu": False,
        "has_jetson": True,
        "has_robot": False,
    }

    # Perform signup
    response = test_client.post("/api/auth/signup", json=signup_data)

    # Expected to fail until T023-T025 implemented
    assert response.status_code == status.HTTP_201_CREATED

    response_data = response.json()
    user_data = response_data["user"]
    tokens_data = response_data["tokens"]

    # Verify AuthUser created in database
    db_user = test_db_session.query(AuthUser).filter(
        AuthUser.email == signup_data["email"]
    ).first()

    assert db_user is not None, "AuthUser not found in database"
    assert db_user.email == signup_data["email"]
    assert db_user.password_hash != signup_data["password"], "Password not hashed"
    assert db_user.password_hash.startswith("$2b$"), "Password not hashed with bcrypt"
    assert db_user.is_active is True
    assert db_user.created_at is not None
    assert db_user.updated_at is not None

    # Verify UserProfile created in database
    db_profile = test_db_session.query(UserProfile).filter(
        UserProfile.user_id == db_user.id
    ).first()

    assert db_profile is not None, "UserProfile not found in database"
    assert db_profile.user_id == db_user.id
    assert db_profile.ai_level == signup_data["ai_level"]
    assert db_profile.ml_level == signup_data["ml_level"]
    assert db_profile.ros_level == signup_data["ros_level"]
    assert db_profile.python_level == signup_data["python_level"]
    assert db_profile.linux_level == signup_data["linux_level"]
    assert db_profile.has_gpu == signup_data["has_gpu"]
    assert db_profile.has_jetson == signup_data["has_jetson"]
    assert db_profile.has_robot == signup_data["has_robot"]
    assert db_profile.cloud_only is False, "cloud_only should be False (has_jetson=True)"
    assert db_profile.created_at is not None
    assert db_profile.updated_at is not None

    # Verify response matches database
    assert user_data["email"] == db_user.email
    assert user_data["user_id"] == str(db_user.id)
    assert user_data["ai_level"] == db_profile.ai_level
    assert user_data["ml_level"] == db_profile.ml_level

    # Verify tokens returned
    assert tokens_data["access_token"] is not None
    assert tokens_data["refresh_token"] is not None
    assert tokens_data["token_type"] == "bearer"


def test_signup_cloud_only_computation(test_client: TestClient, test_db_session: Session, test_settings):
    """
    Test that cloud_only is computed correctly based on hardware flags.

    Acceptance Criteria:
    - cloud_only = True when has_gpu=False, has_jetson=False, has_robot=False
    - cloud_only = False when any hardware flag is True
    """
    # Test Case 1: No hardware (cloud_only should be True)
    signup_data_cloud = {
        "email": "cloudonly@example.com",
        "password": "CloudTest123",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": False,
        "has_jetson": False,
        "has_robot": False,
    }

    response1 = test_client.post("/api/auth/signup", json=signup_data_cloud)
    # Expected to fail until T023-T025 implemented
    assert response1.status_code == status.HTTP_201_CREATED

    user1 = response1.json()["user"]
    assert user1["cloud_only"] is True, "cloud_only should be True when no hardware"

    # Verify in database
    db_profile1 = test_db_session.query(UserProfile).filter(
        UserProfile.user_id == user1["user_id"]
    ).first()
    assert db_profile1.cloud_only is True

    # Test Case 2: Has GPU (cloud_only should be False)
    signup_data_gpu = {
        "email": "gpu@example.com",
        "password": "GpuTest123",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": True,
        "has_jetson": False,
        "has_robot": False,
    }

    response2 = test_client.post("/api/auth/signup", json=signup_data_gpu)
    assert response2.status_code == status.HTTP_201_CREATED

    user2 = response2.json()["user"]
    assert user2["cloud_only"] is False, "cloud_only should be False when has_gpu=True"


def test_signup_password_verification(test_client: TestClient, test_db_session: Session, test_settings):
    """
    Test that hashed password can be verified against plain text password.

    Acceptance Criteria:
    - Signup creates user with hashed password
    - verify_password(plain_password, hashed_password) returns True
    - verify_password(wrong_password, hashed_password) returns False
    """
    from src.utils.password import verify_password

    signup_data = {
        "email": "password-verify@example.com",
        "password": "VerifyMe123",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": False,
        "has_jetson": False,
        "has_robot": False,
    }

    response = test_client.post("/api/auth/signup", json=signup_data)
    # Expected to fail until T023-T025 implemented
    assert response.status_code == status.HTTP_201_CREATED

    # Get user from database
    db_user = test_db_session.query(AuthUser).filter(
        AuthUser.email == signup_data["email"]
    ).first()

    # Verify correct password
    assert verify_password(signup_data["password"], db_user.password_hash) is True

    # Verify wrong password
    assert verify_password("WrongPassword123", db_user.password_hash) is False
