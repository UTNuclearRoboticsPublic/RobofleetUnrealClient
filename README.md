# RobofleetUnrealClient
Client-side code for Robofleet in the form of an Unreal Engine module.
It is nominally a stripped down clone of [robofleet_client](https://github.com/ut-amrl/robofleet_client), with ROS dependencies and message scheduling removed.

# Code Overview
This Module, other Robofleet clients, and the Robofleet server makes used of flatbuffers and a common library to facilitate common functions. 
These shared components are detailed here.


### flatbuffers and generated schemas
The flatbuffers library is a way to serialize and package data across platforms. The 'blueprint' for how to encode and decode these structures is defined in the flatbuffer schema, 
which is specified in `robofleet_client_lib/include/schema_generated.h`. 

### robofleet_client_lib
This is a shared library that all existing robofleet clients use, in varying degrees. It includes it's own `decode` and `encode` functions that can be defined and used cross-platform.
Additionally, it defines a number of C++ structs in `message_structs.h` that are useful once a flatbuffer object is received and decoded.

### robofleet_client
While not used directly in this Module, it's notable to mention that if you're using robofleet with robots, they will likely be using [this](https://github.com/ut-amrl/robofleet_client) as their client. 
Therefore, you should be aware that new functionality may need to be added here for your robot to publish new data.


## Adding new functionality (new messages)
When adding new message support to the Robofleet system, these are the general steps that need to be carried out:

+ add the new definition to the `schema_generated.h` file in `robofleet_client_lib`
+ add a relevant C++ struct with the same fields in `message_structs.h` in `robofleet_client_lib`
+ add definitions to the `encode.hpp` and `decode.hpp` for the new message type in `robofleet_client_lib`
+ add support for the message type in `encode_ros.hpp` and `decode_ros.hpp` in `robofleet_client` IF your robot needs to publish those messages to robofleet

At this point, unless there are any logic issues with the added support, new messages should be able to be sent by a robot, through robofleet, and received by the client library used by `RobofleetUnrealClient`.
The only thing to do now is to integrate that data within the Module itself. 

This will require subscribing to the new message topic, e.g. in `RobofleetClientBase.cpp`: 

```
	RegisterRobotSubscription("my_topic_name_here", "*");
```

and then catch incoming messages in the function `void URobofleetBase::DecodeMsg()` so you can handle the decoding process correctly for that data.
Once the data is decoded and made available under the `RobotMap` data field, it can be accessed on-demand.

When everything is tested, you'll need to create a pull request for each of the three libraries you modified to add support.

An example of adding message support to `robofleet_client_lib` can be seen [here](https://github.com/ut-amrl/robofleet_client_lib/pull/10).
An example of adding message support to `robofleet_client` can be seen [here](https://github.com/ut-amrl/robofleet_client/pull/54).
