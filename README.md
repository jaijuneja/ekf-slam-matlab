Overview
========

SLAM simulator in MATLAB that uses a graphical interface for visualisation and allows manual mapping of rooms and obstacles.

For example videos of the software and further information, please visit:
www.jaijuneja.com/blog/2013/05/simultaneous-localisation-mapping-matlab/

Copyright Info
=============

This software was written and developed by Jai Juneja as part of an undergraduate project in the Department of Engineering Science, University of Oxford. If you have any questions, I can be contacted at:
jai.juneja@balliol.ox.ac.uk or jai.juneja@gmail.com

Please feel free to use, modify and distribute the software for personal or research purposes, along with acknowledgement of the author and inclusion of this copyright information.

Some of the code included has been adapted from other software. They are acknowledged as follows:
* Code for Jacobian transformations was adapted from a SLAM course by Joan Sola (http://www.joansola.eu/JoanSola/eng/course.html)
* ICP algorithm in doICP.m was adapted from code written by Ajmal Saeed Mian (relevant copyright informtion included in the file).
* Any 3rd party code that has been untouched is in the folder 3rd-party

Instructions
============

1. Navigate to the root folder and run setup.m (you can just type 'setup' in the command window).
2. The GUI should open up. There are a number of available maps saved as .mat files in the root folder that can be loaded, or alternatively you can create your own map.
3. When ready, click 'Execute SLAM Simulation'. You can save the resulting grid map as a matrix in a .mat file (along with the map resolution) or a .tiff image.