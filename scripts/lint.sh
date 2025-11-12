#!/bin/bash

# Run all linting checks for the project

echo "🔍 Running Python linting checks..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $2"
    else
        echo -e "${RED}✗${NC} $2"
    fi
}

# Exit code accumulator
exit_code=0

echo "Running flake8..."
flake8 . --count --statistics
flake8_exit=$?
print_status $flake8_exit "Flake8 check"
exit_code=$((exit_code + flake8_exit))

echo -e "\nRunning black..."
black --check --diff .
black_exit=$?
print_status $black_exit "Black formatting check"
exit_code=$((exit_code + black_exit))

echo -e "\nRunning isort..."
isort --check-only --diff .
isort_exit=$?
print_status $isort_exit "Import sorting check"
exit_code=$((exit_code + isort_exit))

echo -e "\nRunning mypy..."
mypy go2_sdk/ go2_auto_dock/ orchestrator/ --ignore-missing-imports
mypy_exit=$?
print_status $mypy_exit "Type checking (mypy)"
# Don't add mypy to exit code for now as we're gradually improving type annotations

echo -e "\nRunning pytest..."
pytest --cov=go2_sdk --cov=go2_auto_dock --cov=orchestrator
pytest_exit=$?
print_status $pytest_exit "Unit tests"
exit_code=$((exit_code + pytest_exit))

# ROS 2 specific linting (if in ROS environment)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "\nRunning ROS 2 specific tests..."
    source /opt/ros/humble/setup.bash

    # Check if colcon is available
    if command -v colcon &> /dev/null; then
        colcon test --packages-select go2_sdk go2_auto_dock orchestrator
        ros_test_exit=$?
        print_status $ros_test_exit "ROS 2 package tests"
        exit_code=$((exit_code + ros_test_exit))
    else
        echo -e "${YELLOW}⚠${NC} ROS 2 colcon not found, skipping ROS-specific tests"
    fi
else
    echo -e "${YELLOW}⚠${NC} ROS 2 not found, skipping ROS-specific tests"
fi

echo -e "\n" + "="*50
if [ $exit_code -eq 0 ]; then
    echo -e "${GREEN}🎉 All linting checks passed!${NC}"
else
    echo -e "${RED}❌ Some linting checks failed (exit code: $exit_code)${NC}"
fi

exit $exit_code
