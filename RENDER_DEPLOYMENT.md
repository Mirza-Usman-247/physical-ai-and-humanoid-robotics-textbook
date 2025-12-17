# Render Deployment Guide

## Problem: Rust Compilation Error on Render

If you encounter this error during deployment:
```
error: failed to create directory `/usr/local/cargo/registry/cache/index.crates.io-...`
Caused by: Read-only file system (os error 30)
```

This happens because the `tiktoken` package requires Rust compilation, and Render's default cargo directory is read-only.

## Solutions Implemented

### 1. ✅ Updated `render.yaml` (Primary Solution)

The `render.yaml` file has been configured with:
- **CARGO_HOME**: Set to writable directory `/opt/render/project/.cargo`
- **RUSTUP_HOME**: Set to writable directory `/opt/render/project/.rustup`
- Proper build commands that export these environment variables

### 2. ✅ Updated tiktoken Version

Upgraded `tiktoken` from `0.6.0` to `0.8.0` in both:
- `backend/requirements.txt`
- `backend/requirements.prod.txt`

This version has better support for pre-built wheels, reducing the need for compilation.

### 3. ✅ Created Build Script

Added `backend/build.sh` that can be used as an alternative build command:
```bash
chmod +x backend/build.sh && backend/build.sh
```

### 4. ✅ Updated Dockerfile

Enhanced the Dockerfile to install Rust toolchain for local/Docker builds.

## Deployment Steps

### Option A: Using render.yaml (Recommended)

1. Push the changes to your repository:
   ```bash
   git add render.yaml backend/
   git commit -m "Fix: Configure Rust environment for tiktoken compilation"
   git push
   ```

2. In Render Dashboard:
   - Go to your service
   - Trigger a manual deploy or wait for auto-deploy
   - The `render.yaml` will be automatically detected and used

### Option B: Manual Configuration

If not using `render.yaml`, configure in Render Dashboard:

1. Go to your backend service
2. Navigate to **Environment**
3. Add these environment variables:
   ```
   CARGO_HOME=/opt/render/project/.cargo
   RUSTUP_HOME=/opt/render/project/.rustup
   ```

4. Update **Build Command** to:
   ```bash
   export CARGO_HOME=/opt/render/project/.cargo && export RUSTUP_HOME=/opt/render/project/.rustup && cd backend && pip install --upgrade pip && pip install -r requirements.prod.txt
   ```

### Option C: Using the Build Script

1. Make the script executable (do this locally before pushing):
   ```bash
   chmod +x backend/build.sh
   ```

2. Set your Render **Build Command** to:
   ```bash
   cd backend && chmod +x build.sh && ./build.sh
   ```

## Verification

After deployment succeeds, check:

1. ✅ Build logs show no Rust/Cargo errors
2. ✅ `tiktoken` installs successfully
3. ✅ Service starts without crashes
4. ✅ API endpoints are accessible

## Troubleshooting

### Still Getting Rust Errors?

1. **Check Python Version**: Ensure you're using Python 3.11 (configured in render.yaml)
   - `tiktoken 0.8.0` has pre-built wheels for Python 3.8-3.12
   - Python 3.13 may require compilation

2. **Try Explicit Wheel Installation**:
   Add to your build command:
   ```bash
   pip install --only-binary :all: tiktoken
   ```

3. **Alternative: Remove tiktoken**:
   If you don't need token counting, you can remove it:
   ```bash
   # In requirements files, comment out or remove:
   # tiktoken==0.8.0
   ```

### Build Taking Too Long?

Rust compilation can be slow. The pre-built wheels (version 0.8.0) should help, but if builds are still slow:
- Use Docker deployment instead
- Enable Render's build cache
- Consider switching to a package without Rust dependencies

## Additional Resources

- [Render Python Deployment Docs](https://render.com/docs/deploy-python)
- [tiktoken GitHub](https://github.com/openai/tiktoken)
- [Rust on Render](https://render.com/docs/rust)

## Need Help?

If issues persist:
1. Check Render build logs for specific errors
2. Verify environment variables are set correctly
3. Ensure you're using the correct requirements file (`requirements.prod.txt`)
