# Off Rover Software Installation 
1. Install Anaconda https://docs.anaconda.com/anaconda/install/
2. Create a new python virtual environment for the project using either the Anaconda Navigator or the Anaconda Prompt (installed with Anaconda) using
   `conda create -n RoboRetriever python=3.12`
   Note python 3.12 can be installed with `conda install -c conda-forge python=3.12` if it is not installed
4. Activate conda environment
   `conda activate RoboRetriever`
5. Install pip in the newly created conda environment
  `conda install pip`
6. Navigate to RoboRetriever/research/cv folder
7. Install the required python packages
   `pip install -r requirements.txt`
8. Note on some Windows machines you may encounter an OSError (OSError: [WinError 126] The specified module could not be found.
   Error loading "C:\Users\anaconda3\envs\RoboRetriever\Lib\site-packages\torch\lib\fbgemm.dll" or one of its dependencies.) This seems to be a
   compatibilty issue with torch 2.4.0 and can be resolved by reverting to a previous version of torch (2.3.1). The below command can be used to do this<br /> `pip install torch==2.3.1 torchaudio==2.3.1 torchvision==0.18.1` <br />(Ref. https://stackoverflow.com/questions/78114412/import-torch-how-to-fix-oserror-winerror-126-error-loading-fbgemm-dll-or-depen)
