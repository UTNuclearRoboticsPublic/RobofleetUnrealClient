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


## Adding New Messages
When adding new message support to the Robofleet system, these are the general steps that need to be carried out:
1. Check the following directory to see if the flatbuffer definitions for the message you want to add already exists (```/Private/robofleet_client_lib/include/msg_schemas```). If the schema does not exist, follow the next steps to create the schema and add it to the ```msg_schemas``` directory. Click through the drop down menus for images from an example.
    <details>
    <summary>a. Create a Catkin Workspace.</summary>
    
    ![01_create_schema](https://user-images.githubusercontent.com/84527482/235497823-84f2e5c4-52a5-41ef-8bce-b89deb3eb2f5.png)

    </details>
    <details>
    <summary>b. Clone and build both <a href="UTNuclearRobots/robofleet_client">https://github.com/UTNuclearRobotics/robofleet_client</a> and your ROS message package into the catkin workspace.</summary>
    
    ![02_clone_packages](https://user-images.githubusercontent.com/84527482/235497296-45848807-bd7b-4450-90f5-835af53885dd.png)
    
    </details>
    <details>
    <summary>c. Generate robofleet code and schema.</summary>
    
    ![03_create_package](https://user-images.githubusercontent.com/84527482/235498116-b9e87f24-c49e-488c-bc45-306e4ebe5666.png)

    </details>
    <details>
    <summary>d. Copy and paste the schema just geneated in the {your_pkg}_msgs_robofleet directory just created into the msg_schemas directory in `robofleet_client_lib`. (Expand Below)</summary>
    
    ![04_copy_and_paste_schema](https://user-images.githubusercontent.com/84527482/235498904-86d05cab-9e1f-448f-9307-7051525d2600.png)

    </details>
    <details>
    <summary>e. Add an include statement in the general schema header file (../Private/robofleet_client_lib/include/schema_generated.h)</summary>
    
    ![05_add_include_statement](https://user-images.githubusercontent.com/84527482/235499417-593df2a6-88df-4a69-852a-508e857049aa.png)

    </details>
    <details>
    <summary>f. Modify include statements in newly imported schema file to match the screen shot from this drop down menu.</summary>
    
    ![06_modify_imported_schema](https://user-images.githubusercontent.com/84527482/235499939-e6d9db33-a21d-4ec7-bdf0-392a19bb4624.png)

    </details>
2. Add a relevant C++ struct with the same fields in `message_structs.h` in `robofleet_client_lib`
    <details>
    <summary>Add Struct Example:</summary>
    
    ![07_add_message_struct](https://user-images.githubusercontent.com/84527482/235500676-941ef43d-41b8-4069-9863-a25d9b885085.png)

    </details>
3. Add definitions to the `encode.hpp` and `decode.hpp` for the new message type in `robofleet_client_lib`
    <details>
    <summary>Decode Function Example:</summary>
    
    ![08_decode_message](https://user-images.githubusercontent.com/84527482/235500964-a790dbe9-a168-42d9-a23c-cc3bdd12572d.png)

    </details>
    <details>
    <summary>Encode Function Example:</summary>
    
    ![09_encode_message](https://user-images.githubusercontent.com/84527482/235501111-b357909f-09ec-4e06-8dc3-b526047c7a44.png)

    </details>
4. Add Map to store the latest messages received in ```RobofleetClientBase.h```
    <details>
    <summary>Add Map Example:</summary>
    
    ![10_add_message_functionality](https://user-images.githubusercontent.com/84527482/235501813-7fea09e4-e9dc-427c-8aec-aadd2a2fe6a9.png)

    </details>
5. Add a delegate to ```RobofleetClientBase.h``` to allow any actor to know when a new message is received.
    <details>
    <summary>Add Delegate Example:</summary>
    
    ![11_add_delegate](https://user-images.githubusercontent.com/84527482/235502069-72021cbf-477f-4e04-a301-6fa000a12d7b.png)

    </details>
6. In ```RobofleetClientBase.cpp``` add a call to ```RegisterRobotSubscription()``` for your specific topic name in both the ```URobofleetBase::RefreshRobotList()``` and ```URobofleetBase::Initialize()``` functions.
    <details>
    <summary>Register Robot Subscription Example:</summary>
    
    ![12_register_topic](https://user-images.githubusercontent.com/84527482/235502761-b004535c-cbae-47ea-b2ca-64d54b67788e.png)

    </details>
7. Create an ```else if``` statement for your topic name in ```RobofleetClientBase.cpp``` in the ```void URobofleetBase::DecodeMsg(const void* Data, FString topic, FString RobotNamespace) { … }``` function to decode the robofleet message and broadcast that the message was received.
    <details>
    <summary>Add DecodeMsg Logic Example:</summary>
    
    ![13_decode_msg](https://user-images.githubusercontent.com/84527482/235503383-6a2aa937-bb96-4740-a436-12d75b6f48f7.png)

    </details>

8. If you want to publish information on added topics from Blueprints, follow the steps below.
    <details>
    <summary>a. Create Unreal Engine message struct</summary>
    
    ![14_create_unreal_struct](https://user-images.githubusercontent.com/84527482/235503966-98047069-a181-41f7-a54d-991e3327bf6b.png)

    </details>
    <details>
    <summary>b. Create a publisher function in RobofleetClientBase.cpp</summary>
    
    ![15_create_publisher_in_cpp](https://user-images.githubusercontent.com/84527482/235504183-46d9379e-73a4-448d-a766-6c8d915d6118.png)

    </details>
    <details>
    <summary>c. Create a publisher function in RobofleetBPFunctionLibrary.cpp to call the last publisher from Blueprints</summary>
    
    ![16_create_publisher_for_bp](https://user-images.githubusercontent.com/84527482/235504463-0399906a-1ce3-403e-97b2-d5beb4c0d865.png)

    </details>
    
 9. If you want to receive information on the added topics in Blueprints, follow the steps below.
    <details>
    <summary>a. Create a Getter similar to the following in ```RobofleetClientBase.cpp```.</summary>
    
    ![18_adding_getter_funciton](https://user-images.githubusercontent.com/84527482/235506186-0a5cf4d6-d470-4d36-8c48-bc1c5f0bc9cc.png)

    </details>
    <details>
    <summary>b. Create a Getter to call the getter created above in Blueprints in the ```RobofleetBPFunctionLibrary.cpp```</summary>
    
    </details>
    
    
At this point, unless there are any logic issues with the added support, new messages should be able to be sent by a robot, through robofleet, and received by the client library used by `RobofleetUnrealClient`.
    <details>
    <summary>Publish Message on New Topic From Blueprints Example:</summary>
    
   ![17_publish_from_blueprints](https://user-images.githubusercontent.com/84527482/235505024-6e95749b-c855-456e-befd-1aab37da13b6.png)
   
    </details>

When everything is tested, you'll need to create a pull request for each of the three libraries you modified to add support.
