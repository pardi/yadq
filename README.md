![build](https://github.com/pardi/yadq/actions/workflows/cmake.yml/badge.svg) [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# yadq
Yet Another Double Quaternion library. This is an efficient library for double quaternion operations written in C++17.

# wiki
TODO: The wiki will explain the several functions and how to use them

# Development setup
## VScode + docker
The simplest way to contribute to the repo is using the provided docker container with VScode. 

## Requirements
- VScode
- Docker

## Setup
- Clone the repo to the your defined folder.
- Open the folder with VSCode
- The IDE will detect the configurations and ask to reload the folder in the container. Say yes.
  - If this doesn't pop up automatically, you can start the process via the blue button on the lower left corner
- Open a terminal
- ```mkidr build && cd build```
- ```cmake .. ```
- ```make ```

# Libraries
- Quaternion
- DualQuaternion
