import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

// TypeScript interfaces matching FastAPI backend models
interface ChatMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
}

interface ChatRequest {
  query: string;
  history: ChatMessage[];
  top_k?: number;
}

interface ChatResponse {
  answer: string;
  sources: string[];
}

const API_BASE_URL = 'https://physical-ai-robotic-book-hackathon-delta.vercel.app/'; // Assuming FastAPI runs on port 8000

const ChatWidget: React.FC = () => {
  const [input, setInput] = useState('');
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(false); // New state for chatbot visibility
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Scroll to the bottom of the messages display
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async () => {
    if (input.trim() === '') return;

    const userMessage: ChatMessage = { role: 'user', content: input.trim() };
    const updatedMessages = [...messages, userMessage];
    setMessages(updatedMessages);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      // Prepare history for the backend
      const chatHistoryForBackend: ChatMessage[] = updatedMessages.map(msg => ({
        role: msg.role,
        content: msg.content
      }));

      const requestBody: ChatRequest = {
        query: userMessage.content,
        history: chatHistoryForBackend,
        top_k: 5,
      };

      const response = await fetch(`${API_BASE_URL}/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Something went wrong on the server.');
      }

      const data: ChatResponse = await response.json();
      const botMessage: ChatMessage = { role: 'assistant', content: data.answer };
      setMessages((prevMessages) => [...prevMessages, botMessage]);

      if (data.sources && data.sources.length > 0) {
        const sourcesMessage: ChatMessage = {
          role: 'assistant',
          content: `Sources: ${data.sources.join(', ')}`,
        };
        setMessages((prevMessages) => [...prevMessages, sourcesMessage]);
      }
    } catch (err: any) {
      console.error('Error sending message:', err);
      setError(err.message || 'Failed to get a response from the chatbot.');
      setMessages((prevMessages) => [...prevMessages, { role: 'assistant', content: 'Sorry, I am having trouble connecting right now.' }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {!isOpen && (
        <button className={styles.chatToggleButton} onClick={() => setIsOpen(true)}>
          <img src="/img/logo.png" alt="Open Chatbot" className={styles.chatLogo} />
        </button>
      )}

      {isOpen && (
        <div className={styles.chatContainer}>
          <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
            &times;
          </button>
          <div className={styles.messagesDisplay}>
            {messages.length === 0 && !isLoading && !error ? (
              <div className={styles.welcomeMessage}>
                Welcome! Ask me anything about the textbook.
              </div>
            ) : (
              messages.map((msg, index) => (
                <div key={index} className={`${styles.message} ${styles[msg.role]}`}>
                  {msg.content}
                </div>
              ))
            )}
            {isLoading && <div className={`${styles.message} ${styles.assistant}`}>Thinking...</div>}
            {error && <div className={`${styles.message} ${styles.error}`}>Error: {error}</div>}
            <div ref={messagesEndRef} />
          </div>
          <div className={styles.inputArea}>
            <input
              type="text"
              className={styles.chatInput}
              placeholder={isLoading ? 'Thinking...' : 'Type your question...'}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => {
                if (e.key === 'Enter' && !isLoading) {
                  handleSend();
                }
              }}
              disabled={isLoading}
            />
            <button className={styles.sendButton} onClick={handleSend} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;