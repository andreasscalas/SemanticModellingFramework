<img src="docs/framework.png" alt="Schema of the framework" width="750px"/>

The Parametric Template framework allows to visualise shapes belonging to a class, analyse their shape in search for peculiar features and to annotate them onto the geometry. The relations existing between these features can be used for defining constraints useful, for example, for restraining deformations so that the final shape would belong to the original class.

## Quickstart
This software has some dependencies:
 - CMake: 
 	CMake (https://cmake.org/) is used to control the compilation.
 - Qt5:
 	Qt5 (https://www.qt.io/) is mainly used for simply creating GUI. The path to the folder containing the Qt5Config.cmake file should be passed as a parameter to CMake (see in the following).
 - VTK:
 	The Visualization ToolKit (VTK - https://vtk.org/) is used for the visualisation of 3D shapes. The path to the folder containing the built library should be passed as a parameter to CMake (see in the following) .
 - Eigen: 
 	The Eigen library (https://eigen.tuxfamily.org) is used for performing linear algebra operations. The path to the directory containing the header files should be passed a parameter to CMake (see in the following) .
 - RapidJSON:
 	The RapidJSON parser is used for reading/writing files related to annotations and relationships. This is already included in the software and needs no external installation.
 - Nanoflann:
 	The nanoflann library (https://github.com/jlblancoc/nanoflann) is used for building kd-trees and performing related operations (search for nearest neighbors in an efficient way). This is already included in the software and needs no external installation.
 - ImatiSTL: 
 	The ImatiSTL library (https://sourceforge.net/projects/imatistl/) is used for the management of triangular meshes.
	
Once the dependencies are correctly installed, the following steps should be followed.
	
Fetch the repository: 

    $ git clone https://github.com/andreasscalas/SemanticModellingFramework.git

Configure and build:

    $ cd SemanticModellingFramework && mkdir build && cd build && cmake .. -DQt5_DIR=PATH-TO-Qt5 -DVTK_DIR=PATH-TO-VTK -DEIGEN_DIR=PATH-TO-EIGEN && make

Run the app:

    $ ./ParametricTemplate
