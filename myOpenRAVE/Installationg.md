1.	Put the script.sh file in folder where you want the installation to happen
2.	Open up the Terminal in that folder and run  "sudo ./script.sh" (without quotations)
3.	Enter root password, the installation should be automatic
4.	You know OpenRAVE works after everything done if a simulation windows popped up




export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)

Write these two lines in your .bashrc or .zshrc to save this configuration between sessions.
