## Steps used to "pretty print" Eigen matrices in GDB

This was done with:

* gcc 4.8.4
* gdb 7.7.1

You need two things from Eigen @ https://bitbucket.org/eigen/eigen/src/8b18ff2c7028/debug/gdb/

* __init__.py
* printers.py

Place these in a folder, mine are at
/home/ben/Workspace

Then you need to create or modify a gdbinit file. This file is invoked when gdb is started from the terminal. It is supposed to look in your /home/YOUR_NAME directory, but that wasn't working for me. I found where gdb was taking its gdbinit file by
```
$ locate gdbinit
```
which returned
```
/etc/gdb/.gdbinit
```

Open this file with whatever editor: gedit, nano, vim, .. and add
```
python
import sys
sys.path.insert(0, '/home/ben')
from printers import register_eigen_printers
register_eigen_printers (None)
end
```
but replace /home/ben with the directory you stored __init__.py, printers.py

When you use gdb from terminal for debugging, you should be able to print Eigen matrices and vectors.

Use this as a test program:
```
#test.cpp
#include <eigen3/Eigen/Dense>
#include <iostream>

int main()
{
        Eigen::VectorXd vec = Eigen::VectorXd::Constant(10, 3.14);
        std::cout << vec << std::endl;
        return 0;
}
```
compile with
```
$ g++ -g -o test test.cpp
```

Then you can run debug from terminal
```
$ gdb ./test
```
```
$ (gdb) b main      # Set a breakpoint at main function
$ (gdb) run         # Run debugging
$ (gdb) n           # Step instruction
$ (gdb) print vec   # Print an Eigen matrix value
```

Which should output:
```
$1 = Eigen::Matrix<double,10,1,ColMajor> (data ptr: 0x606010) = {
  [0] = 3.1400000000000001, [1] = 3.1400000000000001,
  [2] = 3.1400000000000001, [3] = 3.1400000000000001,
  [4] = 3.1400000000000001, [5] = 3.1400000000000001,
  [6] = 3.1400000000000001, [7] = 3.1400000000000001,
  [8] = 3.1400000000000001, [9] = 3.1400000000000001}
```

Can't get Eigen printing in Eclipse yet with the Debug mode. Still just memory address and
template information. Although there is one result about doing it with QtCreator, so maybe
I'll try that later.

Reference:

* http://stackoverflow.com/questions/21340866/how-to-use-pretty-debugging-printers-to-see-eigen-objects-in-qtcreator?noredirect=1&lq=1
