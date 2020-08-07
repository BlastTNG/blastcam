<img src="https://avatars0.githubusercontent.com/u/13680500?s=280&v=4" width="150">

BlastTNG Star Camera
====================

This is the repository for the BlastTNG (and SO LAT) Star Camera. `commands` is the high-level interface that allows users to control star camera(s) connected to their machine.

Description
-----------

The Star Camera software solves perpetually for the current pointing of the camera while accepting user commands and transmitting telemetry. It is built on the [Astrometry.net](https://arxiv.org/abs/0910.2233) suite and the [IAU SOFA](http://www.iausofa.org/) library.

Installation
------------

Currently, you can get this software on your computer by cloning this repository using `git clone <https://github.com/BlastTNG/blastcam.git>`.

Usage
-----

`./commands --camhandle [camera handle] --serial [serial adapter descriptor] --port [port for server socket]\`

### Arguments and Options

Bolded arguments are required. If you are unsure about where to find the required arguments for the BlastTNG and SO Star Camera(s), refer to the support section.

**`-c, --camhandle`**

The unique integer identifier for the Star Camera you are running. Values can range from 1 to 254.

**`-s, --serial`**

The descriptor for the serial adapter attached to the lens for control. Valid lens descriptors will be in the form `/dev/ttyLensXportY`, where X is the corresponding camera handle and Y is the socket port. See `--valid` for more information.

**`-p, --port`**

The port to establish a TCP socket on that will allow communications between the camera and the user.

`-v, --verbose`

Increase the verbosity of output. 

`--valid`

Display the valid combinations of handle, lens descriptor, and socket port.

`--number`

Display the number of cameras connected to the computer.

`--network`

Display the Star Camera computer network information and the size of the telemetry package to be sent to users.

`-h, --help`

Display the help message.

Support
-------

Contact Brooke at [bdigia@sas.upenn.edu](mailto:bdigia@sas.upenn.edu), Alex at [am702058@sju.edu](mailto:am702058@sju.edu), and Javier at [javier.romualdez@gmail.com](mailto:javier.romualdez@gmail.com) for questions and support.

Contributing
------------

If you are becoming a new Star Camera developer on this project, contact Brooke, Alex, and Javier to get properly integrated.

Authors and Acknowledgments
---------------------------

The authors of this project are Javier Romualdez, Brooke DiGia, and Alexander Manduca. They would like to thank the members of the Penn Experimental Cosmology Group for their help and guidance on all aspects of the Star Camera implementation. In addition, they would like to thank Jacob Brown and Jacob Stanger for their contributions.  

This project is part of the BlastTNG and Simons Observatory collaborations.

Project Status
--------------

The Star Camera project is nearing completion! The Star Camera control application, housed in the [GUI repository](https://github.com/BlastTNG/GUI) of the larger [BlastTNG repository](https://github.com/BlastTNG), is also ready for use with the f/2.8 Star Camera lens. It is currently being updated for use with the f/2 lens.



