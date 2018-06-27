# miam_py python package

This package implements python tools developed for MiAM robotics team. These tools are mostly conserned with
plotting and analysing csv logs created from the robot C code.

Aside from the package (miam_py) itself, the following script are provided:

 - miam_plot: a generic plotter script. Specify a space-separated list of headers to plot them. Separate headers by
 comma for plotting on the same axes. See script for full help.


# Installing

Simply run from the package main directory:

```
python setup.py install --user
```

Alternatively, you may specify a custom path with the ```--prefix``` option instead of ```--user```. In any case, you need
to make sure that the install path is in the python path, by appending to your .bashrc file:

```
export $PYTHONPATH=$HOME/.local/lib/python2.7/site-packages/:$PYTHONPATH
```

(or whichever path you chose).

# MiAM log format

A valid log simple is a simple csv (comma separated value) file. Two lines must be present before the data begins:
 - the first line, called the info line, should first contain a log identifier, in the form: `Robot Log: <log_id>`
   This is used to define the log_name argument, which is made to be an identifier for the log. Additional comments may
   then be added to this line, with the usual comma-separation style, and aquired from the log_info variable.
 - the second line is called the header line. It defines a list of name for the following data column. Only columns with
   a name associated will be parsed: thus, header line should be the same length as the following data file.
