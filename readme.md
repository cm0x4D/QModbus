# Abstract
LGPL licensed multiplatform Modbus client library supporting Modbus ASCII, Modbus RTU and Modbus TCP connections. 

Based on Qt (4|5) and completely written in C++. 

Build and tested on Linux, Mac OS X and Windows.

# Installation
You need a working Qt version installed on your system in order to be able to build the library. We are using qmake for all installation steps. For further informations about Qt, just visit their website: http://qt-project.org

## Linux
To install the library on linux, you just have to create the Makefiles using qmake and then build the library using make and install the library to /usr/lib, respective /usr/include using make install.

    # cd QModbus
    # qmake
    # make 
    # qmake
    # sudo make install

## Mac OS X
To install the library on Mac OS X, you just have to create the Makefiles using qmake with the spec file for clang and then build the library using make and install the library to /System/Frameworks using make install.

    # cd QModbus
    # qmake -spec macx-llvm
    # make 
    # qmake
    # sudo make install

## Windows
To install the library on Microsoft Windows, you just have to create the Makefiles using qmake and then build the library using make and install the library to c:/windows/system32 using make install. If you like to install the library to another location, you need to adapt the qmake project file QModbus.pro.

    # cd QModbus
    # qmake
    # make
    # qmake
    # sudo make install

Note that this installation instructions are for the mingw version of Qt. For the version based on Microsoft's Visual Studio compiler, you shoud replace make by nmake above.

# Using QModbus
To use the library in your projects, you need to add QModbus the include path to the search path of you compiler and you need to configure your linker to link against the actual library.

The simplest solution is to build your project either using qmake or Qt Creator. You need just to add the following lines to your project file.

## Linux

    INCLUDEPATH += /usr/include/QModbus
    LIBS += QModbus

## Mac OS X

    INCLUDEPATH += /System/Frameworks/QModbus/Headers
    LIBS += -framework QModbus
    
## Windows

    INCLUDEPATH += c:/windows/system32/include/QModbus
    LIBS += -L c:/windows/system32 -lQModbus

### Acknowledgments

Thanks to Jan Verrept at OneClick in Belgium for the artwork used as the project icon.