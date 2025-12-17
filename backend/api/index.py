"""
Vercel serverless function entry point for FastAPI backend.
This file adapts the FastAPI app to work with Vercel's serverless architecture.
"""

import sys
from pathlib import Path

# Add the backend directory to Python path
backend_dir = Path(__file__).parent.parent
sys.path.insert(0, str(backend_dir))

from src.api.main import app

# Vercel expects an 'app' or 'handler' export
# FastAPI's app object is already ASGI compatible
handler = app
