# microbit-sound-detector

TODO write description

# Installation
You need some open source pre-requisites to build this repo. You can either install these tools yourself, or use the docker image provided below.

- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [Github desktop](https://desktop.github.com/)
- [CMake](https://cmake.org/download/)
- [Python 3](https://www.python.org/downloads/)

# Building

## Without Docker

 - Clone this repository
 - In the root of this repository type `python build.py`
 - The hex file will be built `MICROBIT.HEX` and placed in the root folder.

## Docker

Run the following command to build the builder image locally:

```
    docker build -t microbit-tools .
```

You can use the image you built to build the project sources in the current working directory (equivalent to executing `build.py`).

```
    docker run -v $(pwd):/app --rm microbit-tools
```

You can also provide additional arguments like `--clean`.

```
    docker run -v $(pwd):/app --rm microbit-tools --clean
```

