interface QueryRequest {
  question: string;
  selected_text?: string;
}

interface QueryResponse {
  answer: string;
}

interface AuthResponse {
  access_token: string;
  token_type: string;
  id: string; // Assuming user ID is returned
  email: string; // Assuming user email is returned
}

const API_BASE_URL = 'http://localhost:8000'; // Will be configurable later

export const sendQuery = async (query: QueryRequest, token: string): Promise<QueryResponse> => {
  const response = await fetch(`${API_BASE_URL}/api/query`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`,
    },
    body: JSON.stringify(query),
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to send query');
  }

  return response.json();
};

export const loginUser = async (email: string, password: string): Promise<AuthResponse> => {
  const formBody = new URLSearchParams();
  formBody.append('username', email);
  formBody.append('password', password);

  const response = await fetch(`${API_BASE_URL}/token`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: formBody.toString(),
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to login');
  }

  return response.json();
};

export const registerUser = async (email: string, password: string): Promise<AuthResponse> => {
  const response = await fetch(`${API_BASE_URL}/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ email, hashed_password: password }), // Note: hashed_password here is a plain password for registration endpoint
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Failed to register');
  }

  return response.json();
};
