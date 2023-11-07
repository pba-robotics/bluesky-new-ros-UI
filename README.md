# bluesky-new-ros-UI
The new BlueSky UI that interfaces the new ROS-based Kaze Platform 

# Introduction
GUI for LCD screen attached to AMRs <br>
Uses React to host the web application<br>
Uses rosbridge_server to connect to rosmaster<br>
Uses Chakra UI for front-end framework<br>

# Getting Started
## Electronics
TBC

## Software
Requires node >= 14

# Build and Test
## Installatiion
Navigate to web application folder (inside web-GUI or repo-name folder) and run 'npm install' <br>

## Usage
If rosbridge_server is not running (it usually already is on startup), run 'roslaunch rosbridge_server rosbridge.websocket.launch' <br>
Navigate to web application folder (inside web-GUI or repo-name folder) and run 'npm start' <br>



## Published Topics
| Topic Name                  | Data Type                       |
| --------------------------- | ------------------------------- |
| /client_count               | std_msgs/Int32                  |
| /connected_clients          | rosbridge_msgs/ConnectedClients |

# Errors and Troubleshooting

# Current Progress

## Use Case Diagram


# Known Issues
