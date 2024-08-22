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

## Miscellaneous Notes
For some reason I was getting an error from opencv that `cv2.imshow() is not implemented`. Changing the package manager to conda seemed to fix this issue.

The following commands uninstall opencv in pip and install with conda
`pip uninstall opencv-python`<br />
`conda install -c conda-forge opencv`
Ref. https://stackoverflow.com/questions/40207011/opencv-not-working-properly-with-python-on-linux-with-anaconda-getting-error-th 


## CUDA Installation (OPTIONAL)
Good how to guide: https://medium.com/@leennewlife/how-to-setup-pytorch-with-cuda-in-windows-11-635dfa56724b
Pytorch documentation: https://pytorch.org/get-started/locally/

To install torch==2.3.1 torchvision==0.18.1 and torchaudio==2.3.1 with CUDA 12.1 (may need to uninstall and reinstall using `pip uninstall`) <br />
`pip install torch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 --index-url https://download.pytorch.org/whl/cu121`

To test if CUDA has been installed correctly in a python shell <br/>
`import torch` <br/>
`torch.cuda.is_available()` <br/>
This should return `True` if installed correctly

## Windows Schanigans
Using conda with VS Code
https://stackoverflow.com/questions/54828713/working-with-anaconda-in-visual-studio-code