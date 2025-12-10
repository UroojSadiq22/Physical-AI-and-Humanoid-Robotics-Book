import React from 'react';
import Layout from '@theme-original/Layout';
import Chatbot from '../Chatbot'; // Adjust path as needed

export default function LayoutWrapper(props): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <Chatbot />
    </>
  );
}
