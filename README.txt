This is a tutorial for Playful.
For more information on Playful, and download of the executable: playful.is.tuebingen.mpg.de
Currently, Playful supports exclusively Ubuntu (any recent version)
This tutorial is a replacement for the documentation (sort of). 
We find tutorials, i.e. examples that users can run, less ambiguous.
This tutorial also assumes you are using Ubuntu (any version).


what is Playful
---------------

Playful is a scripting language for complex runtime orchestration of Python API.
It is meant for professional roboticists who want to acquire the skill to develop 
reactive complex behaviors. 
Going through the tutorial should take between 1 and 2 hours.

On one hand, Playful is simple. It has only 5 keywords. 
On the other hand, Playful is powerful, and manipulates non-trivial concepts:
   - encoding of applications in dynamic behavior trees
   - mixing of logics (conditional, prioritized and state machine)
   - dynamic runtime restructure of the tree and online setup of sensory motor couplings
   - encoding of sensory information into discrete scheme systems.

While going through this tutorial, you may at time feel overwhelmed by these new concepts.
This is normal. You may then take a step back and realize that all the python code and the playful 
scripts are actually very short and, once understood, trivial.

If you feel frustrated the code of this tutorial only interfaces only to a trivial toy-robot, please
be patient as another advanced tutorial with code for controlling a turtlebot (real or simulated)
is under preparation. We also recently started working with Softbank robotics Pepper. 
Playful libraries for manipulation and navigation are also under preparation. 
You may also contact playful-support@tuebingen.mpg.de to ask for tips on how to 
interface to a real robot.

We advice to get through the scientific paper published in IEEE Robotics and Automation Magazine.
This paper gives most the necessary background to understand the scripts used as example in this tutorial. 
You can find a link to this paper: playful.is.tuebingen.mpg.de

If anything in this tutorial is unclear, we are happy to get feedback on playful-support@tuebingen.mpg.de. 
Merge requests for improvement of this tutorial are also welcomed.

The tutorial consists of 10 "subtutorials", enumerated from tutorial1 to tutorial10. To each corresponds a playful script that can be executed.


how-to run one of the tutorial
------------------------------

1. download playful executable from playful.is.tuebingen.mpg.de.
2. make the downloaded file executable (in a terminal: 'chmod +x playful') 
2. the executable does not need installation. Put this file anywhere in the path.
   ( $PATH environment variable, see https://askubuntu.com/questions/60218/how-to-add-a-directory-to-the-path
                                or https://help.ubuntu.com/community/EnvironmentVariables#Persistent_environment_variables 
   or in short, add the following line in ~/.bashrc file, setting the path correctly:
     export PATH=${PATH}:<path to the directory which contains the playful executable
   and start a new terminal. You can check if things work by typing "playful" in this new terminal )   
3. in a terminal: go to any of the tutorial folder (e.g.: 'cd <>/playful_tutorial/tutorial8')
4. int the termina: run the executable passing "execute" as parameter ('playful execute'). Press 'q' to exit


how to go through all tutorials
-------------------------------

1. go the directory 'tutorial1'
2. read the README.txt file there
3. this file will redirect you to another file with more info, which will redirect you to another file with more info, etc
4. it will take you around 1 hour


what is needed for the executable to run ?
------------------------------------------

1. Ubuntu
2. Any (relatively) recent Python 2.7 should be installed as default Python interpreter. (Python 3 will not work).
   (this is default on any version of ubuntu)


things not working ? anything unclear ? more questions ?
--------------------------------------------------------

contact playful-support@tuebingen.mpg.de


We hope Playful will be as useful to you as it has been useful to us :)
