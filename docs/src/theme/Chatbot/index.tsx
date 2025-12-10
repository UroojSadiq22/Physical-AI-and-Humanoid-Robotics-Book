import React, { useState, useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import { sendQuery, loginUser, registerUser } from '../../api/chatbot';

// Placeholder components
import { MessageBubble } from './components/MessageBubble';
import { MessageInput } from './components/MessageInput';
import { LoadingIndicator } from './components/LoadingIndicator';

interface ChatMessage {
  author: 'user' | 'bot';
  content: string;
  timestamp: string;
}

export default function Chatbot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [token, setToken] = useState<string | null>(null);
  const [userEmail, setUserEmail] = useState<string | null>(null);
  const [isLoginModalOpen, setIsLoginModalOpen] = useState(false);
  const [loginEmail, setLoginEmail] = useState('');
  const [loginPassword, setLoginPassword] = useState('');
  const [isRegisterMode, setIsRegisterMode] = useState(false);
  const [authError, setAuthError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [selectionRect, setSelectionRect] = useState<DOMRect | null>(null);

  const messageListRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (messageListRef.current) {
      messageListRef.current.scrollTop = messageListRef.current.scrollHeight;
    }
  }, [messages, isLoading]);

  useEffect(() => {
    // Check for existing token in localStorage
    const storedToken = localStorage.getItem('chatbot_access_token');
    const storedEmail = localStorage.getItem('chatbot_user_email');
    if (storedToken && storedEmail) {
      setToken(storedToken);
      setUserEmail(storedEmail);
    }
  }, []);

  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    if (selection && selection.toString().length > 0) {
      const text = selection.toString();
      setSelectedText(text);
      const range = selection.getRangeAt(0);
      setSelectionRect(range.getBoundingClientRect());
    } else {
      setSelectedText(null);
      setSelectionRect(null);
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('keyup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('keyup', handleTextSelection);
    };
  }, [handleTextSelection]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && !token) {
      setIsLoginModalOpen(true);
    }
  };

  const handleLoginRegister = async (e: React.FormEvent) => {
    e.preventDefault();
    setAuthError(null);
    try {
      let authResponse;
      if (isRegisterMode) {
        authResponse = await registerUser(loginEmail, loginPassword);
        alert('Registration successful! Please log in.');
        setIsRegisterMode(false); // Switch to login after successful registration
      } else {
        authResponse = await loginUser(loginEmail, loginPassword);
        setToken(authResponse.access_token);
        setUserEmail(authResponse.email);
        localStorage.setItem('chatbot_access_token', authResponse.access_token);
        localStorage.setItem('chatbot_user_email', authResponse.email);
        setIsLoginModalOpen(false);
      }
    } catch (error: any) {
      setAuthError(error.message || 'Authentication failed');
    }
  };

  const handleLogout = () => {
    setToken(null);
    setUserEmail(null);
    localStorage.removeItem('chatbot_access_token');
    localStorage.removeItem('chatbot_user_email');
    setMessages([]);
    setIsOpen(false);
    setIsLoginModalOpen(true);
  };

  const handleSendMessage = async (message: string, context?: string) => {
    if (!message.trim() || !token) return;

    const newMessage: ChatMessage = {
      author: 'user',
      content: message,
      timestamp: new Date().toLocaleTimeString(),
    };
    setMessages((prevMessages) => [...prevMessages, newMessage]);
    setIsLoading(true);
    setSelectedText(null); // Clear selected text after sending message

    try {
      const response = await sendQuery({ question: message, selected_text: context }, token);
      const botResponse: ChatMessage = {
        author: 'bot',
        content: response.answer,
        timestamp: new Date().toLocaleTimeString(),
      };
      setMessages((prevMessages) => [...prevMessages, botResponse]);
    } catch (error: any) {
      console.error('Error sending message:', error);
      const errorMessage: ChatMessage = {
        author: 'bot',
        content: `Error: ${error.message || 'Sorry, something went wrong. Please try again.'}`,
        timestamp: new Date().toLocaleTimeString(),
      };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
      // If unauthorized, prompt for login again
      if (error.message.includes('Unauthorized')) {
        setToken(null);
        setUserEmail(null);
        localStorage.removeItem('chatbot_access_token');
        localStorage.removeItem('chatbot_user_email');
        setIsLoginModalOpen(true);
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleAskAboutSelection = () => {
    if (selectedText) {
      // Open chat and pre-populate message input or directly send a contextual query
      setIsOpen(true);
      if (!token) { // If not logged in, prompt for login first
        setIsLoginModalOpen(true);
      } else {
        // Here you might want to show the chat input with the selected text or directly send
        // For now, let's assume we open the chat and the user will then type their question.
        // The selectedText state is preserved.
        // Or, we can directly send a question about the selected text.
        handleSendMessage(`Explain this:`, selectedText); // Example: direct contextual query
      }
    }
  };

  if (isLoginModalOpen && (!token || !userEmail)) {
    return (
      <div className={styles.loginModalOverlay}>
        <div className={styles.loginModal}>
          <h3>{isRegisterMode ? 'Register' : 'Login'} to Chatbot</h3>
          <form onSubmit={handleLoginRegister}>
            <input
              type="email"
              placeholder="Email"
              value={loginEmail}
              onChange={(e) => setLoginEmail(e.target.value)}
              required
            />
            <input
              type="password"
              placeholder="Password"
              value={loginPassword}
              onChange={(e) => setLoginPassword(e.target.value)}
              required
            />
            {authError && <p className={styles.authError}>{authError}</p>}
            <button type="submit" disabled={isLoading}>
              {isRegisterMode ? 'Register' : 'Login'}
            </button>
          </form>
          <button
            onClick={() => setIsRegisterMode(!isRegisterMode)}
            className={styles.toggleAuthMode}
          >
            {isRegisterMode ? 'Already have an account? Login' : "Don't have an account? Register"}
          </button>
          <button onClick={() => setIsLoginModalOpen(false)} className={styles.closeModalButton}>Close</button>
        </div>
      </div>
    );
  }

  return (
    <>
      {selectedText && selectionRect && token && (
        <button
          className={styles.askAboutSelectionButton}
          style={{
            top: selectionRect.top + window.scrollY - 40, // Position above selection
            left: selectionRect.left + window.scrollX + (selectionRect.width / 2) - 50, // Center horizontally
          }}
          onClick={handleAskAboutSelection}
        >
          Ask about selection
        </button>
      )}

      <div className={clsx(styles.chatbotContainer, isOpen && styles.open)}>
        <button className={styles.fab} onClick={toggleChat}>
          {isOpen ? 'X' : '💬'}
        </button>

        {isOpen && token && userEmail && (
          <div className={styles.chatWindow}>
            <div className={styles.chatHeader}>
              <h3>AI Chatbot {userEmail && `(${userEmail})`}</h3>
              <button onClick={handleLogout} className={styles.logoutButton}>Logout</button>
            </div>
            <div className={styles.messageList} ref={messageListRef}>
              {messages.map((msg, index) => (
                <MessageBubble key={index} message={msg} />
              ))}
              {isLoading && <LoadingIndicator />}
            </div>
            <MessageInput onSendMessage={handleSendMessage} isLoading={isLoading} />
          </div>
        )}
      </div>
    </>
  );
}
