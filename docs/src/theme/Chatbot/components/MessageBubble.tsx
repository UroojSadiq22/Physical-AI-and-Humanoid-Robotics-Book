import React from 'react';
import clsx from 'clsx';
import styles from '../styles.module.css';

interface MessageProps {
  author: 'user' | 'bot';
  content: string;
  timestamp: string;
}

export const MessageBubble = ({ message }: { message: MessageProps }): JSX.Element => {
  const { author, content, timestamp } = message;
  const isUser = author === 'user';

  return (
    <div className={clsx(styles.messageBubble, isUser ? styles.user : styles.bot)}>
      <div className={styles.messageContent}>
        {content}
        <div className={styles.messageAuthor}>{isUser ? 'You' : 'Bot'} - {timestamp}</div>
      </div>
    </div>
  );
};