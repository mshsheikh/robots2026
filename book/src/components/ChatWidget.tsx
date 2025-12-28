import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

interface Message {
  text: string;
  isUser: boolean;
  timestamp: Date;
  isLoading?: boolean;
}

// Helper function to resolve API_BASE with proper priority and safety
const resolveApiBase = (): string => {
  if (typeof window !== 'undefined' && (window as any).__RAG_API_BASE__) {
    return (window as any).__RAG_API_BASE__;
  }
  if (process.env.REACT_APP_RAG_API_BASE) {
    return process.env.REACT_APP_RAG_API_BASE;
  }
  return 'https://robots2026.up.railway.app';
};

const ChatWidget: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isVisible, setIsVisible] = useState(false);
  const [theme, setTheme] = useState<'light' | 'dark'>('dark');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Initialize theme from localStorage, data-theme attribute, or system preference
  useEffect(() => {
    const prefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    const savedTheme = localStorage.getItem('theme') as 'light' | 'dark' | null;
    const dataTheme = document.body.getAttribute('data-theme') as 'light' | 'dark' | null;
    const initialTheme = savedTheme || dataTheme || (prefersDark ? 'dark' : 'light');
    setTheme(initialTheme as 'light' | 'dark');
    document.body.setAttribute('data-theme', initialTheme);
  }, []);

  // Update theme attribute when theme changes
  useEffect(() => {
    document.body.setAttribute('data-theme', theme);
  }, [theme]);

  // Listen for global theme changes by observing body's data-theme attribute
  useEffect(() => {
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
          const newTheme = document.body.getAttribute('data-theme') as 'light' | 'dark' | null;
          if (newTheme && newTheme !== theme) {
            setTheme(newTheme);
          }
        }
      });
    });

    observer.observe(document.body, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    // Cleanup observer on component unmount
    return () => {
      observer.disconnect();
    };
  }, [theme]);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const addMessage = (text: string, isUser: boolean) => {
    setMessages(prev => [
      ...prev,
      { text, isUser, timestamp: new Date() }
    ]);
  };

  const addLoadingMessage = () => {
    setMessages(prev => [
      ...prev,
      { text: 'Thinking...', isUser: false, timestamp: new Date(), isLoading: true }
    ]);
  };

  const removeLoadingMessage = () => {
    setMessages(prev => prev.filter(msg => !msg.isLoading));
  };

  const toggleTheme = () => {
    const newTheme = theme === 'dark' ? 'light' : 'dark';
    setTheme(newTheme);
    document.body.setAttribute('data-theme', newTheme);
    localStorage.setItem('theme', newTheme);
  };

  // Verify backend connection
  const verifyBackend = async () => {
    try {
      const API_BASE = resolveApiBase();
      const res = await fetch(`${API_BASE}/verify`);
      const data = await res.json();
      return data;
    } catch (err) {
      console.error('Backend verification failed:', err);
      return null;
    }
  };

  const handleSend = async () => {
    const question = inputValue.trim();
    if (!question) return;

    // Add user message
    addMessage(question, true);
    setInputValue('');
    setIsLoading(true);
    addLoadingMessage();

    // Resolve API_BASE before the try block to make it available in catch
    const API_BASE = resolveApiBase();

    try {
      const res = await fetch(`${API_BASE}/ask`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: question })
      });

      const data = await res.json();
      removeLoadingMessage();

      if (data.answer && data.answer.trim()) {
        // Use the clean answer from the backend
        addMessage(data.answer, false);
      } else if (data.results && data.results.length > 0) {
        // Fallback to the old format if answer is not available
        const responseText = data.results.map((result: any, index: number) =>
          `${index + 1}. ${result.text} (Score: ${result.score})`
        ).join('\n\n');
        addMessage(responseText, false);
      } else {
        addMessage('No results found for your query.', false);
      }
    } catch (err) {
      removeLoadingMessage();
      const errorMessage = API_BASE.includes('localhost') || API_BASE.includes('127.0.0.1')
        ? 'Backend not reachable. Make sure to run: uvicorn rag.fastapi.main:app --port 8000'
        : `RAG backend unreachable at ${API_BASE}`;
      addMessage(errorMessage, false);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const toggleVisibility = () => {
    setIsVisible(!isVisible);
  };

  return (
    <>
      {/* Floating button */}
      <button
        className="chat-widget-toggle"
        onClick={toggleVisibility}
        aria-label={isVisible ? 'Close chat' : 'Open chat'}
      >
        {isVisible ? '✕' : '🤖'}
      </button>

      {/* Chat widget */}
      {isVisible && (
        <div className={`chat-widget-container ${theme}`}>
          <div className="chat-header">
            <h2 className="chat-title">🤖 Robots 2026 — AI Assistant</h2>
            <button
              className="theme-toggle"
              onClick={toggleTheme}
              title="Toggle theme"
              aria-label="Toggle theme"
            >
              {theme === 'dark' ? '☀️' : '🌙'}
            </button>
          </div>
          <div className="messages">
            {messages.map((msg, index) => {
              if (msg.isLoading) {
                return (
                  <div key={index} className="message bot-message">
                    <div className="loading">
                      <span className="loading-spinner"></span> Thinking...
                    </div>
                  </div>
                );
              }
              return (
                <div
                  key={index}
                  className={`message ${msg.isUser ? 'user-message' : 'bot-message'}`}
                >
                  <div className="message-text">{msg.text}</div>
                  <div className="message-timestamp">
                    {msg.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              );
            })}
            <div ref={messagesEndRef} />
          </div>
          <div className="input-area">
            <textarea
              ref={textareaRef}
              id="chat-input"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask a question about the Robots 2026 book..."
              aria-label="Type your message"
            />
            <button
              className="send-button"
              onClick={handleSend}
              disabled={isLoading}
              aria-label="Send message"
            >
              Ask →
            </button>
          </div>
          <div className="footer-note">
            This demo runs locally. Start backend with: uvicorn rag.fastapi.main:app --port 8000
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;