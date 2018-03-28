
In this first tutorial, we

1. learn to use the playful executable
2. run a first "hello world" program

In the explanation belows, it is assumed the playful executable has been
download and is in the PATH.


usage of the Playful executable
-------------------------------

1) to see list of files that will be read by the executable, run from the folder this file is in, in a terminal:

   playful trail

2) to see the code and information related of the word "display", run:

   playful word display

   (useful when you read a playful script and would like to know the meaning/location of the vocabulary it uses)

3) to execute the playful program (press 'q' to exit):

   playful execute


what this does
--------------

When running 'playful trail':

- the playful script walks up directories for as long it finds a "__playful__" file. In this tutorial, it goes back up the root of the git folder ('playful-tutorial')

- then it goes down back toward the original folder, at each step reading files in "config", "py" and "play" directories.
  When doing so, it creates its vocubulary

- all the files read by the playful during this process executable are listed when 'playful trail' is called 

When running 'playful executable' 

- same as above but at the end, it reads the file ./play/tutorial1.play file, which has a "program" tree. This program is executed.


display
-------

- When you execute this tutorial ('playful execute'), you should see in the console that a node called 'display' runs.

- When you called 'playful word display', you were given the information that 'display' was declared in a file 'tutorial_play.py'.
  The corresponding python code was also displayed

- when you called 'playful trail' you could this that this 'tutorial_play.py' is part of the trail, i.e. the list of files
  used to create vocabulary.

Look now at the file ./play/tutorial1.play
Based on the comments above, you should understand why the playful interpreter could understand the "display" command

