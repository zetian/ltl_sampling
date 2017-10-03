# Sampling Based Route Planning with Linear Temporal Logic Specifications

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
* OpenCV

```
$ sudo apt-get install libopencv-dev python-opencv
```
* lcm

Download the source code from [website](https://github.com/lcm-proj/lcm).

'''
$ git clone https://github.com/lcm-proj/lcm lcm
$ cd lcm
$ ./bootstrap.sh
$ ./configure
$ make
$ sudo make install
'''

Post install

'''
$ export LCM_INSTALL_DIR=/usr/local/lib
$ echo $LCM_INSTALL_DIR > /etc/ld.so.conf.d/lcm.conf
'''

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

* Eclipse (Optional)
```
$ sudo apt-get install default-jre
$ cd ~/Downloads/
$ wget -N http://eclipse.mirror.rafal.ca/technology/epp/downloads/release/mars/2/eclipse-cpp-mars-2-linux-gtk-x86_64.tar.gz
$ tar xvf eclipse-cpp-mars-1-linux-gtk-x86_64.tar.gz
$ sudo mv eclipse /opt/
```

* Eclipse CMAKE Plugin (Optional)

You can install this Eclipse plugin from the following source to edit CMAKE files:
```
Name: CMAKE Editor
Location: http://cmakeed.sourceforge.net/eclipse/
```

* Terminator (Optional)

A more powerful alternative to the default terminal application.
```
$ sudo apt-get install terminator
```

## 2. Set Up Workspace
You can set up your workspace at any location you prefer. Here I'm using "~/Workspace/sampling_ltl" as an example.
```
$ mkdir -p ~/Workspace/srcl/srcl_aurora
$ cd ~/Workspace/sampling_ltl
$ git init
$ git remote add origin https://github.com/zetian/ltl_sampling
$ git pull origin master
```
Now you have downloaded code in the master branch to your machine. You can start by creating your development branch from the current master branch. For example:

```
$ git checkout -b rdu_dev
```

## 3. Build Project
You can use any preferred text editors/IDEs to write code. Two methods are provided here to compile code:

First create a "build" folder to contain all temporary files created during the building process so that they don't mix with the source code.

```
$ cd ~/Workspace/sampling_ltl
$ mkdir build
$ cd build
```

Then you can invoke cmake to generate a makefile project or an eclipse project so that you can compile the code.

* Command line
```
$ cmake ..
$ make
```

* Eclipse
```
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../cpp-ltl-hcost/
```
Now you can import generated eclipse project located at build folder into eclipse and build the project. Make sure the "Copy projects into workspace" option is **unchecked** before you click the "Finish" button to import the project. Source files are located at "[Source Directory]".

## 4. Build Documentation

Doxygen is used to generate documentation for the C++ code. The configuration file locates at ./docs/doxygen.

Install doxygen if you haven't.

```
$ sudo apt-get install doxygen
```

Then generate the documentation.

```
$ cd /docs/doxygen
$ doxygen Doxyfile
```

## 5. Debug Eigen code with gdb

Refer to [doc](/Eigen_Debug.md).

## Reference:
Git

* [Git Tutorial](http://cleanercode.com/introduction-to-git-talk/introduction-to-git.pdf)
* [Git Cheat Sheet 1](https://www.atlassian.com/dms/wac/images/landing/git/atlassian_git_cheatsheet.pdf)
* [Git Cheat Sheet 2](https://training.github.com/kit/downloads/github-git-cheat-sheet.pdf)
