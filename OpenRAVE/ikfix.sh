
sudo find /usr -name "sympy*" 
sudo rm -rf /usr/local/lib/python2.7/dist-packages/sympy
sudo rm /usr/local/lib/python2.7/dist-packages/sympy-0.7.6.1.egg-info
wget https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/sympy/sympy-0.7.1.tar.gz
tar zxvf sympy-0.7.1.tar.gz
cd sympy-0.7.1
sudo python setup.py install
cd ..
sudo rm -r sympy-0.7.1
rm sympy-0.7.1.tar.gz
