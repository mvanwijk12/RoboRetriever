# Off Rover Software Installation 
1. Install Anaconda https://docs.anaconda.com/anaconda/install/
2. Create a new python virtual environment for the project using either the Anaconda Navigator or the Anaconda Prompt (installed with Anaconda) using
   `conda create -n RoboRetriever python=3.12`
3. Activate conda environment
   `conda activate RoboRetriever`
4. Check the python version is 3.12
   `python --version`
6. Install gstreamer and gstreamer plugins (Ref. https://l.messenger.com/l.php?u=https%3A%2F%2Fstackoverflow.com%2Fquestions%2F73167161%2Ferror-when-trying-to-capture-some-frames-from-a-video-using-open-cv-on-windows&h=AT1L4-CJlgf2nRAigbISdQwqzcugNJKHqjx9FOT7N_6-JLrPcQ-P0EmoRz1szRCdz1GAudkbQ0jjHgUF2hYFcqqUr8Ih-50FSuy64s-Qs4JKBToQByGmlXjBoISLtsKsrEeqPA)
   `conda install gst-plugins-base gst-plugins-good gstreamer`
7. Install pip in the newly created conda environment (this maybe already installed)
  `conda install pip`
8. Navigate to RoboRetriever/milestone1/computer folder
   `cd ../RoboRetriever/milestone1/computer`
9. Install the required python packages
   `pip install -r requirements.txt`
10. Note on some Windows machines you may encounter an OSError (OSError: [WinError 126] The specified module could not be found.
   Error loading "C:\Users\anaconda3\envs\RoboRetriever\Lib\site-packages\torch\lib\fbgemm.dll" or one of its dependencies.) This seems to be a
   compatibilty issue with torch 2.4.0 and can be resolved by reverting to a previous version of torch (2.3.1). The below command can be used to do this (without CUDA, see below for CUDA install)<br /> `pip install torch==2.3.1 torchaudio==2.3.1 torchvision==0.18.1` <br />(Ref. https://stackoverflow.com/questions/78114412/import-torch-how-to-fix-oserror-winerror-126-error-loading-fbgemm-dll-or-depen)
11. Test installation by running <br />
    `cd ../RoboRetriever/milestone1/computer` <br />
    `python client.py` <br />
    Note a internet connection is required the first time this script is run as `ultralytics` will attempt to install some remaining packages. This script will try to access a server on the raspberry pi and a camera stream from the raspberry pi, both of these services need to be running on the pi for `client.py` to execute without errors. For initial testing purposes, the ConnectionClient can be commented/omitted and just the Inference left running, if the installation is successful a image window should appear and update every 0.5s with bounding boxes around any detected tennis balls.

## Bug Fixes (THIS HAS NOT BEEN REPRODUCIBLE!)
For some reason I was getting an error from opencv that `cv2.imshow() is not implemented`. Changing the package manager to conda seemed to fix this issue.

The following commands uninstall opencv in pip and install with conda <br />
`pip uninstall opencv-python`<br />
`conda install -c conda-forge opencv` <br />
Ref. https://stackoverflow.com/questions/40207011/opencv-not-working-properly-with-python-on-linux-with-anaconda-getting-error-th 


## CUDA Installation (OPTIONAL - NVIDIA GRAPHICS CARD REQUIRED)
Good how to guide: https://medium.com/@leennewlife/how-to-setup-pytorch-with-cuda-in-windows-11-635dfa56724b <br />
Pytorch documentation: https://pytorch.org/get-started/locally/

To install torch==2.3.1 torchvision==0.18.1 and torchaudio==2.3.1 with CUDA 12.1 (may need to uninstall and reinstall using `pip uninstall`) <br />
`pip install torch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 --index-url https://download.pytorch.org/whl/cu121`

To test if CUDA has been installed correctly in a python shell <br/>
`import torch` <br/>
`torch.cuda.is_available()` <br/>
This should return `True` if installed correctly

## Windows Schanigans
Using conda with VS Code (follow the "Harder / Powershell" section in the highest scored response)
https://stackoverflow.com/questions/54828713/working-with-anaconda-in-visual-studio-code
