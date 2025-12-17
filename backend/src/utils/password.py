"""
Password hashing and verification utilities using bcrypt.
Uses cost factor 12 (2^12 iterations) for security vs performance balance.
"""

from passlib.context import CryptContext


# bcrypt context with cost factor 12 (industry standard)
# Cost factor 12 provides ~200ms hash time, balancing security and UX
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto", bcrypt__rounds=12)


def hash_password(plain_password: str) -> str:
    """
    Hash a plain text password using bcrypt with cost factor 12.

    Args:
        plain_password: Plain text password to hash

    Returns:
        Hashed password string (60 characters, bcrypt format)

    Example:
        >>> hashed = hash_password("MySecurePassword123!")
        >>> print(hashed)
        '$2b$12$abcdefghijklmnopqrstuv...'
    """
    return pwd_context.hash(plain_password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain text password against a hashed password.

    Args:
        plain_password: Plain text password to verify
        hashed_password: Hashed password from database

    Returns:
        True if password matches, False otherwise

    Example:
        >>> hashed = hash_password("MyPassword123!")
        >>> verify_password("MyPassword123!", hashed)
        True
        >>> verify_password("WrongPassword", hashed)
        False
    """
    return pwd_context.verify(plain_password, hashed_password)


def needs_rehash(hashed_password: str) -> bool:
    """
    Check if a hashed password needs to be rehashed due to algorithm updates.

    Useful for migrating to stronger hash algorithms or cost factors over time.

    Args:
        hashed_password: Hashed password to check

    Returns:
        True if password should be rehashed on next login, False otherwise

    Example:
        >>> old_hash = hash_password_with_old_algorithm("password")
        >>> if needs_rehash(old_hash):
        >>>     new_hash = hash_password("password")
        >>>     # Update database with new_hash
    """
    return pwd_context.needs_update(hashed_password)
