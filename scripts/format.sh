#!/bin/bash

# Auto-format Python code using black and isort

echo "🎨 Auto-formatting Python code..."

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Running black formatter...${NC}"
black .

echo -e "${YELLOW}Running isort to sort imports...${NC}"
isort .

echo -e "${GREEN}✓ Code formatting complete!${NC}"
echo "You may want to review the changes and commit them."
