# M151M Project #2 : Gshare predictor

**Description**:

In this project, you will design a Gshare branch predictor for a RISC-V
processor using C++. 

The ISA of this processor is the standard RV32-I base extension.

The provided source code contains an incomplete implementation of the CPU.
The processor implementation consists of the following parts:
* common:       shared directory of utility libraries
* src:          main source directory
* tests:        tests directory
* Makefile:     build make file
* README.md:    this document

You are responsible for completing the code of the out-of-order pipeline and gshare branch predictor such that you can successfully execute the provided tests with the correct timing.

In this part, you will complete the implementation of the processor's branch predictor.
It will consist of a GShare predictor containing an 8-bit BHR and 256-entry BHT, matched with a 256-entry BTB.
Assume all counters, including the BHR to be initialized to zero at boot.
You should complete the implementation of the GShare class by implementing the predict() and update\_predictors() method.
Note that we are only interested in timing. 
Your GShare's predict() method should determine the current predicted outcome.
update\_predictors() update the predictor information.
also note that a successful prediction is a combination of branch direction and branch target hits.

## Testing your code
use command line option (-s) to enable display of CPU performance stats.
The CPU simulator was added a command line option to activate gshare (-g).
Not passing any option will simply enable the baseline in-order CPU pipeline without gshare.

The provided Makefile contains a `test` command to execute all provided tests.

    $ make test     # baseline
    $ make test-g   # gshare enabled

All tests are under the /tests/ folder.
You can execute an individual test by running:

    $ ./tinyrv -s tests/rv32ui-p-sub.hex

If a test succeeds, you will get "PASSED!" output message.

## Debugging your code
You need to build the project with DEBUG=```LEVEL``` where level varies from 0 to 5.
That will turn on the debug trace inside the code and show you what the processor is doing and some of its internal states.

    $ DEBUG=3 make
    $ ./tinyrv -s tests/rv32ui-p-sub.hex

## What to submit
**A zip file of your source code.
When done with your changes, execute ```make submit``` to generate the submission.zip.
Do not use another method for creating the zip file.

    $ make submit

Please submit the **submission.zip** file.

## Build instructions
* run ```make``` at the root of the project to build the simulator

        $ make

* A linux development environment is needed to build the project
* We recommend using a Ubuntu 18.04 or above distribution
* This C++ project requires C++ 11 to compile and it should come installed on Ubuntu 18.04 or above

## Guidelines
* Only modify the following files: gshare.cpp, gshare.h
* If needed it is ok to add new files to the code or modify other parts of the codebase.
* Do not remove an existing file from the project.
* Do not change the Makefile, you can always add a new file as a header. Make sure to include the new file under the **src/** folder.

## Grading
* simulator compiles successfully: 1 pts
* GShare timing is correct: 9 pts.

Based on the coverage of test suites, you will get partial scores.

***Please do not procrastinate.***

## Extra Credit
* Extra credit will be provided for implementing an additional branch predictor that outperforms GShare on average for the required test cases
* See GSharePlus section of the code for details.
* A leaderboard is also available on Gradescope. Good luck on getting the highest accuracy implementation!
* Hint: an easy implementation for GSharePlus is a "tournament predictor". Feel free to implement this for extra credit.

## Resources
* RISC-V ISA Specs: https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf
* Class materials.
