import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    target: 'esnext' // ← This enables top-level await support
  },
  base: './',
});
