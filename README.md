# Route Planner - Aurora

## 1. Development Environment

* OS: Ubuntu 14.04,
* Compiler: gcc 4.7+
* Building System: CMake

## 2. Install Dependencies

* Building tools, Git, CMake
```
$ sudo apt-get install build-essential
$ sudo apt-get install git
$ sudo apt-get install cmake
```

* lcm

Download the source code from [website](https://github.com/lcm-proj/lcm).

```
$ git clone https://github.com/lcm-proj/lcm lcm
$ cd lcm
$ ./bootstrap.sh
$ ./configure
$ make
$ sudo make install
```

Post install

```
$ export LCM_INSTALL_DIR=/usr/local/lib
$ echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf
```


* Spot

Download the released package from the [website](https://spot.lrde.epita.fr/install.html). Extract the files into a folder you prefer.

```
$ cd <path-to-spot-source-folder>
$ ./configure
$ make
$ sudo make install
```

By default the library-related files are install at "/usr/local". You need to add this path to "LD_LIBRARY_PATH" if you haven't done so. Otherwise the system may not be able to find the shared library when trying to run programs linked against the spot library. Add the following line to your ~/.bashrc:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/
```

* Eigen

```
$ sudo apt-get install libeigen3-dev
```

## 2. Set Up Workspace
You can set up your workspace at any location you prefer. Here I'm using "~/Workspace/aurora-planner" as an example.
```
$ mkdir -p ~/Workspace/aurora-planner
$ cd ~/Workspace/aurora-planner
$ git init
$ git remote add origin https://github.com/ace-lab-wpi/aurora-planner
$ git pull origin master
```

## 3. Build Project

```
$ cd ~/Workspace/aurora-planner
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Edit the path variable for lcm, edit ~/.bashrc
```
export AURORA_HOME=$HOME/Workspace/aurora-planner
export PYTHONPATH=$AURORA_HOME/src/lcmtypes/python:$AURORA_HOME/src/lcmtypes/python/sampling:$PYTHONPATH

```

## 4. Test the example

Follow the test example in src/tests/test_sample_dubins.cpp. 
Once you set up all the paremeters, tasks and environment, run src/python_vis/sampling_vis.py for visualization first, then run the test example, the result would looks like:

<img src="/data/test_example.png" align="middle" height="500" >

### Comments about the test example
1. Orange parts are the region of interests(ROI), grey parts are obtacles.
2. The task for this example is to visit all ROIs without considering the order.
3. The size of the map in this example is 100*100, the minimun turning radius is 15 for the UAV.
4. Set a large enough iterations to ensure a feasible solution, the solution towards optimal when iterations -> infinite.


## Reference:
Git

* [Git Tutorial](http://cleanercode.com/introduction-to-git-talk/introduction-to-git.pdf)
* [Git Cheat Sheet 1](https://www.atlassian.com/dms/wac/images/landing/git/atlassian_git_cheatsheet.pdf)
* [Git Cheat Sheet 2](https://training.github.com/kit/downloads/github-git-cheat-sheet.pdf)
