"""Debug chatbot endpoint to see exact error."""
import requests
import json

BASE_URL = "http://localhost:8000"

print("Testing chatbot endpoint with debug info...")

chat_data = {
    "query": "What is ROS?"
}

headers = {
    "X-Session-Id": "debug-session-12345"
}

try:
    response = requests.post(f"{BASE_URL}/chat/query", json=chat_data, headers=headers)
    print(f"Status: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"Request error: {e}")
    print(f"Response text: {response.text if 'response' in locals() else 'N/A'}")
