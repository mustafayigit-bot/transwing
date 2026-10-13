import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
import { useApp } from './store/app';
import './index.css';

// Store handle for automated UI tests / debugging from the console.
(window as unknown as { __specter: typeof useApp }).__specter = useApp;

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
