## Node Introduction
![Diagram](/images/node_diagram.PNG)
### Node corner_handle_action_server
This node is going to handle situations in maze except straight road, such as left turn, cross roads, T-shape roads. There are 7 diferent corners total and we classify them into 4 groups (turn left, turn right, move forward and turn back)

### Node lineDetector
This node is going to extract the edges between ground and wall. It subscribes to topic "/raspicam_node/image/compressed" and publishes topic "/line_detection".
Firstly, we use some functions in open_cv to pre-process the image from rospicam_node, call Canny function to detect the edges between the ground and wall and by calling HoughLine function we get parameters about those edges to locate them in images. 
Secondly, we have to filter out the detected edges to get at most 3 lines we want.(leftLine, rightLine, frontLine)
Finally, we have to determine which side or which part alone those lines is road or wall.

### Node main
This node is going control the robot movement in maze, such as how to move straight, how to react to a corner. 
![Diagram](/images/workflow.PNG)

### Node pid_service_server
This node is a kind of a simple version PID. It gets some errors and calculate the pid value to make sure the robot move straight.

### Node turn_action_server
This node is an action server and it is going to make robot rotate a specific angle in degree.

### Node forward_action_server
This node is an action server and it is going to make a robot move forward a specific distance.

### Node visualization
This node is going to subscribes topic "/line_dection" to get lines and visualize them.