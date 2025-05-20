# smart_headlamp_pkg

### Main PC
* Jetson Orin Nano (Ubuntu 20.04 LTS)
* It should have 'smart_headlamp_pkg' package in local catkin workspace
* It should be connected to the router what dofbot is also connnected (SAME IP ADDRESS)
* To run main launch script, TRY THIS : <br/>
```$ roslaunch smart_headlamp_pkg main_launcher.launch```
* You should check current branch of local repository and do ```git pull``` before modify code

### Dofbot PC
* RaspberryPi 4B (Ubuntu 20.04 LTS)
* It also should have 'smart_headlamp_pkg' package in local catkin workspace (dofbot_ws)
* Just turn on the board ! Every control of headlamp should be made from MAIN PC.
* To compile and upload .ino code to arduino board, TRY THIS : <br/>
```$ uno_compile```, ```$ uno_upload```
* To check serial monitor of arduino, TRY THIS : <br/>
```$ uno_screen```
* You should check current branch of local repository and do ```git pull``` before modify code <br/>
_(This also exist on Main PC)_
