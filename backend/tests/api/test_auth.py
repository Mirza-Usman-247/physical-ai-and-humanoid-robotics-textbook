"""
T019: Contract tests for POST /api/auth/signup endpoint.

Tests the API contract for user signup including:
- Successful signup with valid data
- Duplicate email rejection
- Invalid password validation
"""
import pytest
from fastapi.testclient import TestClient
from fastapi import status


def test_signup_success(test_client: TestClient, test_settings):
    """
    Test successful user signup with valid data.

    Acceptance Criteria:
    - POST /api/auth/signup with valid email, password, and skill levels
    - Returns 201 Created status
    - Response includes user profile (user_id, email, skill levels, hardware flags, cloud_only, created_at)
    - Response includes auth tokens (access_token, refresh_token, token_type="bearer")
    - Password is NOT included in response
    - User + profile stored in database
    """
    signup_data = {
        "email": "test@example.com",
        "password": "SecurePass123",
        "ai_level": 3,
        "ml_level": 2,
        "ros_level": 4,
        "python_level": 5,
        "linux_level": 3,
        "has_gpu": True,
        "has_jetson": False,
        "has_robot": False,
    }

    response = test_client.post("/api/auth/signup", json=signup_data)

    # Expected to fail until T023-T025 implemented (auth service + signup endpoint)
    assert response.status_code == status.HTTP_201_CREATED

    data = response.json()

    # Verify response structure
    assert "user" in data
    assert "tokens" in data

    # Verify user profile data
    user = data["user"]
    assert user["email"] == "test@example.com"
    assert user["ai_level"] == 3
    assert user["ml_level"] == 2
    assert user["ros_level"] == 4
    assert user["python_level"] == 5
    assert user["linux_level"] == 3
    assert user["has_gpu"] is True
    assert user["has_jetson"] is False
    assert user["has_robot"] is False
    assert user["cloud_only"] is False  # has_gpu=True, so cloud_only=False
    assert "user_id" in user
    assert "created_at" in user
    assert "password" not in user  # Password must NOT be in response

    # Verify tokens
    tokens = data["tokens"]
    assert "access_token" in tokens
    assert "refresh_token" in tokens
    assert tokens["token_type"] == "bearer"
    assert len(tokens["access_token"]) > 0
    assert len(tokens["refresh_token"]) > 0


def test_signup_duplicate_email(test_client: TestClient, test_settings):
    """
    Test signup rejection when email already exists.

    Acceptance Criteria:
    - First signup succeeds
    - Second signup with same email returns 409 Conflict or 400 Bad Request
    - Error message indicates email already registered
    - No duplicate user created in database
    """
    signup_data = {
        "email": "duplicate@example.com",
        "password": "SecurePass123",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": False,
        "has_jetson": False,
        "has_robot": False,
    }

    # First signup - should succeed
    response1 = test_client.post("/api/auth/signup", json=signup_data)
    # Expected to fail until T023-T025 implemented
    assert response1.status_code == status.HTTP_201_CREATED

    # Second signup with same email - should fail
    response2 = test_client.post("/api/auth/signup", json=signup_data)
    assert response2.status_code in [status.HTTP_409_CONFLICT, status.HTTP_400_BAD_REQUEST]

    error_data = response2.json()
    assert "detail" in error_data
    assert "email" in error_data["detail"].lower() or "already" in error_data["detail"].lower()


def test_signup_invalid_password(test_client: TestClient, test_settings):
    """
    Test signup rejection with invalid password.

    Acceptance Criteria:
    - Password missing uppercase: returns 422 Unprocessable Entity
    - Password missing lowercase: returns 422 Unprocessable Entity
    - Password missing number: returns 422 Unprocessable Entity
    - Password too short (<8 chars): returns 422 Unprocessable Entity
    - Error message explains password requirements
    - No user created in database
    """
    base_signup_data = {
        "email": "password-test@example.com",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": False,
        "has_jetson": False,
        "has_robot": False,
    }

    # Test 1: Missing uppercase letter
    response1 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "password": "weakpass123"}  # No uppercase
    )
    assert response1.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY
    assert "uppercase" in response1.json()["detail"][0]["msg"].lower()

    # Test 2: Missing lowercase letter
    response2 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "password": "WEAKPASS123"}  # No lowercase
    )
    assert response2.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY
    assert "lowercase" in response2.json()["detail"][0]["msg"].lower()

    # Test 3: Missing number
    response3 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "password": "WeakPassword"}  # No number
    )
    assert response3.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY
    assert "number" in response3.json()["detail"][0]["msg"].lower()

    # Test 4: Too short
    response4 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "password": "Pass1"}  # Only 5 chars
    )
    assert response4.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY
    # Should fail on min_length validation from Pydantic


def test_signup_invalid_skill_levels(test_client: TestClient, test_settings):
    """
    Test signup rejection with invalid skill levels (out of 1-5 range).

    Acceptance Criteria:
    - Skill level < 1: returns 422 Unprocessable Entity
    - Skill level > 5: returns 422 Unprocessable Entity
    - Error message indicates valid range (1-5)
    """
    base_signup_data = {
        "email": "skill-test@example.com",
        "password": "SecurePass123",
        "ai_level": 3,
        "ml_level": 3,
        "ros_level": 3,
        "python_level": 3,
        "linux_level": 3,
        "has_gpu": False,
        "has_jetson": False,
        "has_robot": False,
    }

    # Test: Skill level too low (0)
    response1 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "ai_level": 0}
    )
    assert response1.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY

    # Test: Skill level too high (6)
    response2 = test_client.post(
        "/api/auth/signup",
        json={**base_signup_data, "ml_level": 6}
    )
    assert response2.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY


def test_signup_missing_required_fields(test_client: TestClient, test_settings):
    """
    Test signup rejection when required fields are missing.

    Acceptance Criteria:
    - Missing email: returns 422 Unprocessable Entity
    - Missing password: returns 422 Unprocessable Entity
    - Missing any skill level: returns 422 Unprocessable Entity
    - Error message lists missing fields
    """
    # Missing email
    response1 = test_client.post(
        "/api/auth/signup",
        json={
            "password": "SecurePass123",
            "ai_level": 3,
            "ml_level": 3,
            "ros_level": 3,
            "python_level": 3,
            "linux_level": 3,
        }
    )
    assert response1.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY

    # Missing password
    response2 = test_client.post(
        "/api/auth/signup",
        json={
            "email": "test@example.com",
            "ai_level": 3,
            "ml_level": 3,
            "ros_level": 3,
            "python_level": 3,
            "linux_level": 3,
        }
    )
    assert response2.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY

    # Missing skill level (ai_level)
    response3 = test_client.post(
        "/api/auth/signup",
        json={
            "email": "test@example.com",
            "password": "SecurePass123",
            "ml_level": 3,
            "ros_level": 3,
            "python_level": 3,
            "linux_level": 3,
        }
    )
    assert response3.status_code == status.HTTP_422_UNPROCESSABLE_ENTITY
