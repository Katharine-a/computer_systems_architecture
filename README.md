Note: Due to the midterm, project 1 skeleton code release is slightly delayed.

# M151M Project #1 : Pipelined RISC-V CPU Simulator

**Description**:

In this project, you will implement a RISC-V 5-stage pipelined processor using
C++.

The ISA of this processor is the standard RV32-I base extension + the zmmul
extension. It is your responsibility to find the details about the zmmul
extension.

The provided source code contains an incomplete implementation of the CPU.
The processor implementation consists of the following parts:
* common:       shared directory of utility libraries
* src:          main source directory
* tests:        tests directory
* Makefile:     build make file
* README.md:    this document

The project is divided into 3 parts:

## Part 1 : Decode
In this part, you will complete the implementation of the decode logic.
Refer to **TODO** entries inside decode.cpp for where to make changes.

## Part 2 : Execute
In this part, you will complete the implementation of the execute logic.
We recommend not adding new types to the codebase, the exiting types are sufficient to complete the project.
Refer to **TODO** entries inside execute.cpp for where to make changes.

## Part 3 : Timing Simulation
In this part, you will complete the implementation of the pipeline logic.
Refer to **TODO** entries inside core.cpp for where to make changes.

## Testing your code
The provided Makefile contains a `test` command to execute all provided tests.

    $ make tests

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
* Do not modify the following files: Makefile, main.cpp, common/*
* If needed it is ok to add new files to the code or modify other parts of the codebase.
* Do not remove an existing file from the project.
* Do not change the Makefile, you can always add a new file as a header. Make sure to include the new file under the **src/** folder.

## Grading
* simulator compiles successfully: 1 pts
* simulator passes all tests: 5 pts
* simulator timing is correct: 4 pts
Based on the coverage of test suites, you will get partial scores.

***Please do not procrastinate.***

## Resources
* RISC-V ISA Specs: https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf
* Class materials.
* The zmmul extension is not included. You will need to find proposal and specification documents yourself.
