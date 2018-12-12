# How To Install xenopusProject:

## First, Technical Considerations:

- Computer: 

     - For reference, we use e.g. Dell workstations (58xx series) with **Windows 10**  

- Computer Hardware:
     - Ideally, you will use at least 32GB RAM  

- Camera Hardware:
     - The software is implemented with opencv so it can run with pretty much any camera (grayscale, color, or graysale captured under infrared light etc.).
     - We use Basler Ace 1920-150uc camera (with pylon5 library)
     
- Software: 
     - Operating System: Windows 10. (However, it can be run on Ubuntu ...)
     - Anaconda/Python3: Anaconda: a free and open source distribution of the Python programming language (download from: \https://www.anaconda.com/). xenopusProject is written in Python 3 (https://www.python.org/) and not compatible with Python 2. 
     - Opencv
     - GUI based on pyqtgraph
     - Basler pylon5 camera software suite 5.0.11.10913
     - Install pypylon (see below)
            
     
    
## Installation:
 
**Windows:**

-Install opencv :

    `` conda install -c conda-forge opencv ``
    
-Install pyqtgraph :

    `` conda install -c conda-forge pyqtgraph ``
    
-Install swig :

    ``  conda install -c anaconda swig  ``
    
-Install MinGW

-Install visualStudio 2017 with:

      - "Developpement Desktop C++"
      
      - "Developpement Python, python native developpement tool"
    
-Install pypylon :
    - from source
    
           ``
           git clone https://github.com/basler/pypylon.git
           cd pypylon
           pip install
           ``
           
    - from binary : download release pypylon-1.2.0.pylon5.0.12-cp36-cp36m-win_amd64.whl
    
           ``
           pip3 install ...
           ``

## You're ready to run XenopusProject


<p align="center">
<img src="/doc/capturescreen-rostro-caudal-66-60p.gif">
</p>




Return to [readme](../README.md).

