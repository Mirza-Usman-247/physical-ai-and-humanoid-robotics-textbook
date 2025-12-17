"""
T021: Unit tests for password hashing utilities.

Tests the password hashing and verification functions in src/utils/password.py.
"""
import pytest
from src.utils.password import hash_password, verify_password, needs_rehash


def test_hash_password():
    """
    Test that hash_password creates a bcrypt hash.

    Acceptance Criteria:
    - Returns a string (hashed password)
    - Hash starts with '$2b$' (bcrypt identifier)
    - Hash is at least 60 characters (bcrypt standard)
    - Same password produces different hashes (salt is random)
    - Hash is different from plain text password
    """
    plain_password = "TestPassword123"

    # Hash the password
    hashed = hash_password(plain_password)

    # Verify hash properties
    assert isinstance(hashed, str), "Hash should be a string"
    assert hashed.startswith("$2b$"), "Hash should use bcrypt ($2b$ prefix)"
    assert len(hashed) >= 60, "Bcrypt hash should be at least 60 characters"
    assert hashed != plain_password, "Hash must be different from plain text"

    # Verify same password produces different hashes (due to random salt)
    hashed2 = hash_password(plain_password)
    assert hashed != hashed2, "Same password should produce different hashes (random salt)"


def test_verify_password():
    """
    Test that verify_password correctly verifies passwords against hashes.

    Acceptance Criteria:
    - Returns True when plain password matches the hash
    - Returns False when plain password does not match the hash
    - Uses constant-time comparison (security requirement)
    - Works with bcrypt cost factor 12 (from password.py)
    """
    plain_password = "CorrectPassword123"
    wrong_password = "WrongPassword456"

    # Hash the correct password
    password_hash = hash_password(plain_password)

    # Test correct password
    assert verify_password(plain_password, password_hash) is True, \
        "Correct password should verify successfully"

    # Test wrong password
    assert verify_password(wrong_password, password_hash) is False, \
        "Wrong password should fail verification"

    # Test empty password
    assert verify_password("", password_hash) is False, \
        "Empty password should fail verification"

    # Test similar but wrong password (different case)
    assert verify_password("correctpassword123", password_hash) is False, \
        "Case-sensitive verification should fail for different case"


def test_verify_password_with_different_hashes():
    """
    Test password verification with multiple different passwords and hashes.

    Acceptance Criteria:
    - Each password only verifies against its own hash
    - Cross-verification always fails
    """
    passwords = [
        "Password1!",
        "SecurePass123",
        "MySecret456",
        "Testing789!"
    ]

    # Hash all passwords
    hashes = [hash_password(pwd) for pwd in passwords]

    # Test each password verifies against its own hash
    for i, password in enumerate(passwords):
        assert verify_password(password, hashes[i]) is True, \
            f"Password {i} should verify against its own hash"

        # Test password does NOT verify against other hashes
        for j, other_hash in enumerate(hashes):
            if i != j:
                assert verify_password(password, other_hash) is False, \
                    f"Password {i} should NOT verify against hash {j}"


def test_needs_rehash():
    """
    Test that needs_rehash correctly identifies when a password needs rehashing.

    Acceptance Criteria:
    - Returns False for bcrypt hashes with current cost factor (12)
    - Returns True for hashes with different cost factor (if algorithm changed)
    - Used for password hash migration scenarios
    """
    # Hash with current settings (cost factor 12)
    current_hash = hash_password("TestPassword123")

    # Should not need rehashing with current settings
    assert needs_rehash(current_hash) is False, \
        "Current bcrypt hash should not need rehashing"


def test_hash_password_special_characters():
    """
    Test password hashing with special characters.

    Acceptance Criteria:
    - Handles passwords with special characters (!@#$%^&*()_+-=[]{}|;:,.<>?)
    - Handles passwords with unicode characters
    - Handles passwords with whitespace
    """
    special_passwords = [
        "Pass!@#$%^&*()",
        "Test_Password-123+",
        "Password With Spaces",
        "Unicode\u2764Password",  # Heart emoji
        "Tab\tPassword",
        "Newline\nPassword"
    ]

    for password in special_passwords:
        hashed = hash_password(password)
        assert hashed.startswith("$2b$"), f"Should hash special password: {repr(password)}"
        assert verify_password(password, hashed) is True, \
            f"Should verify special password: {repr(password)}"
        assert verify_password(password + "X", hashed) is False, \
            f"Should fail verification with modified special password: {repr(password)}"


def test_hash_password_edge_cases():
    """
    Test password hashing with edge cases.

    Acceptance Criteria:
    - Handles minimum length passwords (8 characters per spec)
    - Handles maximum length passwords (128 characters per spec)
    - Handles single character type passwords (will fail validation at API layer, but hash should work)
    """
    # Minimum length (8 chars)
    min_password = "Pass123!"
    assert len(min_password) == 8
    min_hash = hash_password(min_password)
    assert verify_password(min_password, min_hash) is True

    # Maximum length (128 chars)
    max_password = "A" * 64 + "1" * 64  # 128 characters
    assert len(max_password) == 128
    max_hash = hash_password(max_password)
    assert verify_password(max_password, max_hash) is True

    # Very long password (beyond 128 - will be truncated at API validation)
    long_password = "A" * 200
    long_hash = hash_password(long_password)
    assert verify_password(long_password, long_hash) is True


def test_password_hash_timing_resistance():
    """
    Test that verify_password uses constant-time comparison.

    Acceptance Criteria:
    - bcrypt's verify method inherently uses constant-time comparison
    - No early return on first character mismatch
    - Protects against timing attacks

    Note: This is a functional test that bcrypt is used correctly.
    Detailed timing analysis requires specialized testing tools.
    """
    password = "TimingTest123"
    password_hash = hash_password(password)

    # These should all take similar time (bcrypt constant-time verification)
    # We can't measure timing precisely in unit tests, but we verify behavior
    assert verify_password("X" + password[1:], password_hash) is False  # First char wrong
    assert verify_password(password[:-1] + "X", password_hash) is False  # Last char wrong
    assert verify_password("X" * len(password), password_hash) is False  # All chars wrong

    # All should complete without errors and return False
    # (timing resistance is inherent in bcrypt implementation)
