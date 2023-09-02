# ORB_SLAM2_ALGD-ORB

## Introduction

This repository contains the code for the paper "ALGD-ORB: an improved image feature extraction algorithm with adaptive threshold and local gray difference." The code aims to improve the ORB (Oriented FAST and Rotated BRIEF) algorithm for feature point detection and description.

## Prerequisites

The library has been tested and developed under the following system configurations:

- **Processor**: Intel Core i7-11800H @ 2.30GHz
- **Memory**: 16GB RAM
- **Operating System**: Ubuntu 20.04
- **Development Environment**: Microsoft Visual Studio Code 2021
- **Programming Language**: C++ with OpenCV 4.2.0
- **Test Dataset**:  [Oxford dataset](https://www.robots.ox.ac.uk/~vgg/research/affine/)

## Installation

1. Clone the repository: 

   ```
   1. git clone https://github.com/cgm-free/ORB_SLAM2_ALGD-ORB.git
   ```

2. Navigate to the project directory:

```
cd ORB_SLAM2_ALGD-ORB
```

## Usage

To run the code, follow these steps:

We provide a script `build.sh` to build the code. Execute:

```
cd ORB_SLAM2_ALGD-ORB
chmod +x build.sh
./build.sh
```

