#!/bin/bash
# GitHub Pages Deployment Script for Robotics Documentation
# Builds and deploys the Docusaurus site to GitHub Pages

set -e  # Exit immediately if a command exits with a non-zero status

# Configuration
SITE_DIR="./book"  # Directory containing the Docusaurus site
BUILD_DIR="./book/build"  # Build output directory
GITHUB_PAGES_BRANCH="gh-pages"  # Branch for GitHub Pages

echo "Starting GitHub Pages deployment..."

# Navigate to the site directory
cd "$SITE_DIR" || { echo "Error: Could not find site directory $SITE_DIR"; exit 1; }

# Install dependencies if needed
if [ -f "package.json" ] && [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install
fi

# Build the site
echo "Building the site..."
npm run build

# Check if build was successful
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory $BUILD_DIR does not exist. Build may have failed."
    exit 1
fi

echo "Build completed successfully!"

# Create .nojekyll file to prevent Jekyll processing
echo "Creating .nojekyll file..."
touch "$BUILD_DIR/.nojekyll"

# Deploy to GitHub Pages using gh-pages
if command -v npx &> /dev/null; then
    echo "Deploying to GitHub Pages using gh-pages..."
    npx gh-pages -d "$BUILD_DIR" -b "$GITHUB_PAGES_BRANCH" -m "Deploy documentation to GitHub Pages [skip ci]"
    echo "Deployment completed successfully!"
else
    echo "Error: npx is not available. Please install Node.js and run this script again."
    exit 1
fi

echo "GitHub Pages deployment finished!"