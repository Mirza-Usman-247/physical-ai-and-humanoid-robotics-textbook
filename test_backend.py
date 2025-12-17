"""Quick test script for backend API endpoints."""
import requests
import json

BASE_URL = "http://localhost:8000"

print("=" * 60)
print("Testing Backend API Endpoints")
print("=" * 60)

# Test 1: Health Check
print("\n1. Testing Health Endpoint...")
try:
    response = requests.get(f"{BASE_URL}/health")
    print(f"   Status: {response.status_code}")
    print(f"   Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"   ERROR: {e}")

# Test 2: User Signin (user already created)
print("\n2. Testing User Signin...")
try:
    signin_data = {
        "email": "test@example.com",
        "password": "TestPass123"
    }
    response = requests.post(f"{BASE_URL}/api/auth/signin", json=signin_data)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        access_token = data["tokens"]["access_token"]
        print(f"   User ID: {data['user']['user_id']}")
        print(f"   Email: {data['user']['email']}")
        print(f"   Access Token: {access_token[:50]}...")
    else:
        print(f"   Response: {response.text}")
except Exception as e:
    print(f"   ERROR: {e}")
    access_token = None

# Test 3: Chatbot Query
print("\n3. Testing Chatbot Query...")
try:
    chat_data = {
        "query": "What is physical AI and why is it important?"
    }
    headers = {
        "X-Session-Id": "test-session-456"
    }
    response = requests.post(f"{BASE_URL}/chat/query", json=chat_data, headers=headers)
    print(f"   Status: {response.status_code}")
    if response.status_code == 200:
        data = response.json()
        print(f"   Answer: {data['answer'][:200]}...")
        print(f"   Response Time: {data['response_time_ms']}ms")
        print(f"   Citations: {len(data.get('citations', []))} sources")
    else:
        print(f"   Response: {response.text}")
except Exception as e:
    print(f"   ERROR: {e}")

# Test 4: Get User Profile (with auth)
if access_token:
    print("\n4. Testing Protected Profile Endpoint...")
    try:
        headers = {
            "Authorization": f"Bearer {access_token}"
        }
        response = requests.get(f"{BASE_URL}/api/profile", headers=headers)
        print(f"   Status: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(f"   Profile: {json.dumps(data, indent=2)}")
        else:
            print(f"   Response: {response.text}")
    except Exception as e:
        print(f"   ERROR: {e}")

print("\n" + "=" * 60)
print("Testing Complete")
print("=" * 60)
