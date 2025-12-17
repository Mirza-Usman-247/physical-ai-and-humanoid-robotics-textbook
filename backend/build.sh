#!/usr/bin/env bash
# Render build script for Python backend with Rust dependencies

set -e  # Exit on error

echo "🔧 Setting up build environment..."

# Set writable directories for Rust/Cargo
export CARGO_HOME="${CARGO_HOME:-/opt/render/project/.cargo}"
export RUSTUP_HOME="${RUSTUP_HOME:-/opt/render/project/.rustup}"

echo "📦 CARGO_HOME: $CARGO_HOME"
echo "📦 RUSTUP_HOME: $RUSTUP_HOME"

# Create directories if they don't exist
mkdir -p "$CARGO_HOME"
mkdir -p "$RUSTUP_HOME"

echo "⬆️  Upgrading pip..."
pip install --upgrade pip

echo "📚 Installing Python dependencies..."
pip install -r requirements.prod.txt

echo "✅ Build completed successfully!"
