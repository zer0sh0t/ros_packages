this is a ROS package which contains code for creating publisher/subscriber nodes and server/client services\
nodes are written in both C++ and python, any node can talk to any other node irrespective of the language they were programmed in\
this package should be placed under `catkin_workspace/src/`

## build
to build this package, start roscore:

    roscore

and in a new terminal, run these cmds in your workspace:

    cd ~/catkin_workspace
    catkin_make # this builds both C++ and python nodes in the package

## running nodes
to start publisher and subscriber nodes, run subscriber and publisher cmds in two different terminals:

    rosrun pkg_zero listener     # C++ subscriber
    rosrun pkg_zero listener.py  # python subscriber

    rosrun pkg_zero talker       # C++ publisher
    rosrun pkg_zero talker.py    # python publisher

to start server and client services, run server and client cmds in two different terminals:

    rosrun pkg_zero add_two_ints_server                   # C++ server
    rosrun pkg_zero add_two_ints_server.py                # python server

    rosrun pkg_zero add_two_ints_client <num1> <num2>     # C++ client
    rosrun pkg_zero add_two_ints_client.py <num1> <num2>  # python client
