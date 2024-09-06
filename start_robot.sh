#!/bin/bash

# This is a shortcut file to start the client program
# make this file executable: chmod +x start_robot.sh
# run the file: ./start_robot.sh

# Specify the Anaconda environment name
ENV_NAME="RoboRetriever"

# Specify the directory where the Python file is located
SCRIPT_DIR="/path/to/repo/RoboRetriever/milestone1/computer"

# Specify the Python file to execute
PYTHON_FILE="client.py"

# Activate the Anaconda environment
source ~/anaconda3/bin/activate $ENV_NAME

# Navigate to the script directory
cd $SCRIPT_DIR

# Run the Python file
python $PYTHON_FILE

# Deactivate the environment
conda deactivate
