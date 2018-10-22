# pinit

[grpc]:https://grpc.io/
[proto]:https://github.com/ynassar/pinit/tree/master/proto
[mongo]:https://www.mongodb.com/
[ios_source]:https://github.com/ynassar/pinit/tree/master/ios
[ros_source]:https://github.com/ynassar/pinit/tree/master/ros_ws

pinit is a platform for autonomous guidance and waypoint navigation in a pre-mapped environment. Three main components are provided:
* A server application, which responds to navigation and mapping requests by forwarding them onto the appropriate robot. The server application also handles tasks such as storing user and map data, 
    as well as translating data produced by the robot into a format usable by the application.
* A mobile application client that provides a user interface to the application.
* A ROS package that implements the mapping and navigation functionalities.

## Server

pinit uses [gRPC][grpc] for client-server and ROS-server communications. The server is implemented as a series of python handlers for services defined in [protobuf files][proto].

### Running the server

To run local instances of the servers, the [mongodb][mongo] daemon must first be running locally. To install mongodb using apt, run `sudo apt-get install mongodb`. To run the daemon, use `mkdir .db && sudo mongod --dbpath=.db` from the pinit root directory.

Assuming the mongodb daemon is running, the servers may be run locally using `bash run_servers_locally.sh` from the pinit root directory. This script first attempts to install python3 and pip3 if they are not already installed, then creates a virtual environment and installs the required pip packages, runs all necessary code generation, and then starts the server applications on their default ports.

## Mobile Application

The mobile application is an iOS application written in Swift, and communicates with the server using gRPC. The source can be found [here][ios_source].

## ROS package

The ROS package includes multiple nodes for different core functionalities, including a node that handles gRPC communication with the server. The source can be found [here][ros_source].