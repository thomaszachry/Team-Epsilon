#FOR USE WITH UBUNTU 16.04.1
- THIS SCRIPT WILL ATTEMPT TO DOWNLOAD AND INSTALL ALL PACKAGES AND DEPENDENCIES FOR AND INCLUDING OPENRAVE.
- THIS WILL PUT ALL FILES IN A FOLDER CALLED "myOpenRAVE" LOCATED IN ~/HOME
- THE SCRIPT WILL ALSO ATTEMPT TO WRITE TWO LINES TO .bashrc SO THAT OPENRAVE WILL BE AVAILABLE FOR USAGE AT STARTUP
- EXAMPLE FOR OPENRAVE USAGE: openrave.py --example hanoi 
 

####INSTRUCTIONS: 
1. Download [script.sh] (https://raw.githubusercontent.com/thomaszachry/Team-Epsilon/master/myOpenRAVE/script.sh)
2. Right-click downloaded file, go to "Permissions" tab, check "Allow executing file as program"
3.	Open up the Terminal in the folder containing script.sh
4.	Run  "sudo ./script.sh" (without quotations)
5.	Enter root password and the installation should be automatic
6.	You know OpenRAVE works after everything done if a simulation for tower of hanoi windows popped up


####SIDE NOTES
1. *In case the script doesn't do this automatically, add these two lines in your* **.bashrc** *or* **.zshrc**
  - export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
  - export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)

