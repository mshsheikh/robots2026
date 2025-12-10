---
id: 003
title: Deploy Docusaurus GitHub Actions Setup
stage: tasks
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: docusaurus-deployment
branch: main
user: user
command: /sp.deploy-docusaurus
labels:
  - docusaurus
  - deployment
  - github-actions
  - github-pages
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - book/docusaurus.config.js
  - .github/workflows/deploy-docusaurus.yml
tests: []
---

# Deploy Docusaurus GitHub Actions Setup

## Summary
Set up GitHub Actions workflow for automatic deployment of Docusaurus site to GitHub Pages. Updated configuration to use correct baseUrl and repository name for proper deployment.

## Changes Made
1. Updated baseUrl in docusaurus.config.js from `/ai-humanoid-robotics/` to `/robots2026/` to match repository name
2. Updated projectName in docusaurus.config.js from `ai-humanoid-robotics` to `robots2026` to match repository name
3. Created GitHub Actions workflow file `.github/workflows/deploy-docusaurus.yml` for automatic deployment

## Configuration Details
- baseUrl: `/robots2026/` (for GitHub Pages under user repo)
- projectName: `robots2026`
- Workflow triggers on pushes to main/master branches
- Uses Node.js 18 with npm caching for faster builds
- Builds site in ./book directory and deploys ./book/build to GitHub Pages

## Next Steps
1. Commit and push changes
2. Workflow will automatically run on next push to main branch
3. Check GitHub Actions for deployment status
4. Verify site at https://mshsheikh.github.io/robots2026/