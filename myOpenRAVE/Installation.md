------FOR USE WITH UBUNTU 16.04.1
- THIS SCRIPT WILL ATTEMPT TO DOWNLOAD AND INSTALL ALL PACKAGES AND DEPENDENCIES FOR AND INCLUDING OPENRAVE.
- THIS WILL PUT ALL FILES IN A FOLDER CALLED "myOpenRAVE" LOCATED IN ~/HOME
- THE SCRIPT WILL ALSO ATTEMPT TO WRITE TWO LINES TO .bashrc SO THAT OPENRAVE WILL BE AVAILABLE FOR USAGE AT STARTUP
- EXAMPLE FOR OPENRAVE USAGE: openrave.py --examples hanoi 
 

***INSTRUCTIONS: 

1.	Put the script.sh file in folder where you want the installation to happen
2.	Open up the Terminal in that folder and run  "sudo ./script.sh" (without quotations)
3.	Enter root password, the installation should be automatic
4.	You know OpenRAVE works after everything done if a simulation for tower of hanoi windows popped up


***SIDE NOTES

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)

Write these two lines in your .bashrc or .zshrc to save this configuration between sessions.
