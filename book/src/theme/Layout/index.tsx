import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget';
import Head from '@docusaurus/Head';

export default function LayoutWrapper(props) {
  return (
    <>
      <Head>
        <script src="/robots2026/js/rag-config.js" />
      </Head>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatWidget />
    </>
  );
}