![build](https://github.com/pardi/yadq/actions/workflows/cmake.yml/badge.svg) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# yadq
Yet Another Double Quaternion library. This is an efficient library for double quaternion operations written in C++17.

# wiki
Doxygen documentation available at: https://pardi.github.io/yadq/

# Development setup
## VScode + docker
The simplest way to contribute to the repo is using the provided docker container with VScode. 

## Requirements
- VScode
- Docker

## Setup
- Clone the repo to the your defined folder.
```
git clone --recurse-submodules git@github.com:pardi/yadq.git
```
- Open the folder with VSCode
- The IDE will detect the configurations and ask to reload the folder in the container. Say yes.
  - If this doesn't pop up automatically, you can start the process via the blue button on the lower left corner
- Open a terminal
- ```mkidr build && cd build```
- ```cmake .. ```
- ```make ```
Use the option `-DBUILD_TESTS=ON`, if you want to enable the unit testing
