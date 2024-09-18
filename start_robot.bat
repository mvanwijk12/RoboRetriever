@echo off
REM Specify the Anaconda environment name
SET ENV_NAME="RoboRetriever"

REM Specify the directory where the Python file is located
SET SCRIPT_DIR="\path\to\repo\RoboRetriever\src\computer"

REM Specify the Python file to execute
SET PYTHON_FILE="client.py"

REM Activate the Anaconda environment
CALL C:\Users\%USERNAME%\anaconda3\Scripts\activate.bat %ENV_NAME%

REM Navigate to the script directory
cd %SCRIPT_DIR%

REM Run the Python file
python %PYTHON_FILE%

REM Deactivate the environment
CALL conda deactivate
