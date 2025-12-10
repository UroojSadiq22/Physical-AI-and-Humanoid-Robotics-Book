import React from 'react';
import styles from '../styles.module.css';

export const LoadingIndicator = (): JSX.Element => {
  return (
    <div className={styles.loadingIndicator}>
      <span />
      <span />
      <span />
    </div>
  );
};