import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './chat.module.css';

function ChatBot() {
  const [inputValue, setInputValue] = useState('');
  const [messages, setMessages] = useState([
    { role: 'assistant', content: 'Hello! I am a chatbot for the Humanoid Robotics book. Ask me anything about Physical AI & Humanoid Robotics.' }
  ]);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = { role: 'user', content: inputValue };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: inputValue }),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.answer }]);
    } catch (error) {
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: 'Sorry, I encountered an error processing your request. Please try again.' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Chatbot" description="Chat with the Humanoid Robotics Assistant">
      <div className={clsx('container', 'margin-vert--lg')}>
        <div className="row">
          <div className="col col--12">
            <div className={styles.chatContainer}>
              <div className={styles.chatHeader}>
                <h1>Humanoid Robotics Chatbot</h1>
                <p>Ask questions about Physical AI & Humanoid Robotics</p>
              </div>
              
              <div className={styles.chatMessages}>
                {messages.map((msg, index) => (
                  <div 
                    key={index} 
                    className={clsx(
                      styles.message, 
                      msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                    )}
                  >
                    <div className={styles.messageContent}>
                      {msg.content}
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className={clsx(styles.message, styles.assistantMessage)}>
                    <div className={styles.messageContent}>
                      Thinking...
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              <form onSubmit={handleSubmit} className={styles.chatForm}>
                <input
                  type="text"
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  placeholder="Ask a question about humanoid robotics..."
                  className={styles.chatInput}
                  disabled={isLoading}
                />
                <button 
                  type="submit" 
                  className={styles.chatButton}
                  disabled={isLoading || !inputValue.trim()}
                >
                  Send
                </button>
              </form>
              <div className={styles.chatFooter}>
                <p>Note: The chatbot backend runs separately. Make sure to start the backend using `python backend/main.py`</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default ChatBot;