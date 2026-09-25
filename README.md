# **Introduction**  
Welcome to Purdue AI Racing!

This onboarding project introduces you to the tools and workflows used to develop our autonomy software. You'll learn the basics of **Linux/Ubuntu**, **Robot Operating System 2 (ROS2)**, and **PAIR's simulator**, then write  a simple **localization** and **planning** node to make the car drive autonomously around the track!  Finally, you'll make your first contribution to the competition stack.

We expect these exercises to take about a month to complete, so we are targeting the first week of October.
Your first few weeks of classes will likely be the lightest, so we recommend front loading as much as you can.
If you find yourself behind the expected timelines in this guide, reach out to Josh, Dalseong, Alex or Manuel for help.

Many of these tasks are open-ended problems, and as such may take experimentation to solve.  Please use Teams to talk to others doing the onboarding to brainstorm solutions.
If you and a partner cannot solve an issue, always feel free to reach out to a more experienced member (e.g. Josh, Dalseong, Alex or Manuel ) for a quick chat!

Additionally, once you learn how to make a PR, please make corrections, clarifications or improvements to this README!

You're encouraged, and often required, to use **outside resources** for these tasks:  
- Linux documentation via `man` pages  
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)  
- Python or C++ official documentation  
- AI tools like ChatGPT for debugging and explanations  

> **Tip**  
> Don’t just copy instructions into ChatGPT and paste answers back. You’ll learn much more by understanding the process.

## **Requirements**  
Before starting, make sure you have:  
- A **recent-model computer** with enough storage for Ubuntu  
  *(talk to Manuel if you don't have one)*  
- A **USB flash drive**  
- **Time and patience**  

# **Phase 0: Environment Setup & Basic Tutorials (Complete By: 09/11)**

In this phase, you'll install **Ubuntu**, set up **ROS2**, and complete basic ROS2 tutorials.


### **Step 1. Install Ubuntu 22.04**  
It is **critical** to install **Ubuntu 22.04** specifically.

- [Dual boot with Windows](https://help.ubuntu.com/community/WindowsDualBoot)  
- [Clean install (wipes Windows)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)  

> ⚠️ **Warning! Data Loss Risk**  
> Installing Ubuntu incorrectly can erase your files. **Back up your data** before starting.


### **Step 2. Explore Ubuntu**  
Once installed, take time to familiarize yourself:  
- Open Firefox and browse the web  
- Launch the **terminal** (`Ctrl + Alt + T`)  
- Explore the **home directory structure**
  - Type `ls` to view list directory files
  - Type `cd <directory>` to change directories
  - To view manual pages, type `man ls` or `man cd`


### **Step 3. Install Visual Studio Code (VS Code)**  
We'll use VS Code as our primary Integrated Development Environment (IDE).

- Follow the [installation instructions](https://code.visualstudio.com/docs/setup/linux#_debian-and-ubuntu-based-distributions)  
- Open VS Code from a terminal:  
  ```bash
  code ~
  ```
> In Linux, `~` is the path to your home directory (i.e. it's aliased to `/home/your_username/`)
- In VS Code:
  1. Open **Documents** folder in the sidebar
  2. Create `first_file.txt`
  3. Add text and save
- Verify in terminal:
```bash
ls ~/Documents
cat ~/Documents/first_file.txt
man cat
```
- The last command shows you the documentation for the `cat` command.  Try to output the text in `first_file.txt` with line numbers.

### **Step 4. Install ROS2**
Next, we will install ROS2 **Humble**!  ROS2 versions are code named.  We currently use the **Humble Hawksbill** distribution, which is targeted for Ubuntu 22.04.

ROS2 is a **middleware** that allows communication between many **nodes**.  It is very useful for quick development and prototyping of robotic systems.  It also includes many standard implementations of algorithms and tools.  You are encouraged to explore the ROS2 ecosystem.

- Use the official ROS2 Humble installation instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Install the `ros-humble-desktop` package, which includes many standard utilities.
> ⚠️ **Note**  
> Many packages are installable using `apt`, Ubuntu's default [package manager](https://www.debian.org/doc/manuals/apt-guide/index.en.html).
> In general, ROS2 packages can be installed with `sudo apt install ros-humble-<package>`.  In this case, we are installing `ros-humble-desktop` which is a collection of many packages.

###  **Step 5. Follow the introductory ROS2 tutorials**
All tutorials can be found [here](https://docs.ros.org/en/humble/Tutorials.html).  Follow along all of the **Beginner** sections and sub-sections.  Additionally:
- [launch file tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
  - Launch files allow you to bring up multiple nodes at the same time, specify arguments, load parameters and many more useful things.  A brief understanding of general syntax and usefulness is all that's required for now.
- [tf2 tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html).
  - TF2 is ROS2's method of handling static transformations (i.e. between a car's center of gravity and a sensor) as well as dynamic transformations (i.e. a world origin and the car while it's moving).  
For later phases, you will need to understand the tree structure, how to broadcast and listen to transforms, and how to debug potential issues.

You are now done with Phase 0.  Please send an email to Josh, Dalseong, Alex or Manuel to let them know.

# **Phase 1: Repository Setup (Complete By: 09/18)**
We use Git and GitHub to manage development on the team.  For a brief introduction on both of these, see [this page](https://docs.github.com/en/get-started/start-your-journey/about-github-and-git).  It is recommended you follow the "Start your journey" page if you have not used GitHub before.

### **Step 0. Create and add an SSH key to your GitHub account**
An SSH key allows you to access private repositories on GitHub on your terminal.
- Follow [these instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

### **Step 1. Create a repository from this template**
- Click "Use this template" on the upper-right of this page.
- Select yourself as the owner, and name the repository, i.e. `mynames_onboarding`
- Make this a private repository
- On your new repository, go to `Settings` and `Collaborators`.  Add @aschleiz and @Joshsong98.
> **NOTE**  
> You are encouraged to bounce ideas off of each other and collaborate!  Feel free to add extra collaborators here to share your code.

You can now view this README from your own repository.  You have essentially copied all files from this repository into your own!

### **Step 2. Clone your repository**
Cloning a repository means to download the files and all file history for a given repository to your local computer.  This allows you to edit files and push them back to GitHub.

- Open your repository by clicking on your user icon in the upper right, going to "Your Repositories" and selecting the one you just created
- In the upper right, you will see a green "Code" button.  Copy the URL from the "SSH" tab - it should look like `git@github.com:<Your-Username>/your_repo_name.git`
- In your terminal, type `git clone <URL>`
- You should now be able to view all cloned files using `ls` and `cat`.  Verify the directory structure is the same as GitHub.

### **Step 3. Make a change!**
Git allows us to facilitate changes between our local machine and a remote server.  This is essential for collaboration.  Lets make our first change.

- Open the repository in VSCode by using `cd` to enter the repository directory, followed by `code .`
> **NOTE**  
> `.` represents the current directory in Linux.  In the standard Ubuntu Shell/Terminal, this will be the path shown after the colon.

- Fix thsi typo in the README.md file
- View your current changed files:
```bash
git status
```

> **TIP**  
> VS Code has an integrated terminial at the bottom.  If it does not show up, use the terminal drop down in the upper left to add it.

- You will see your change under "Changes not staged for commit:"
- Lets make sure we did the right thing!  Use the following command to view detailed changes.  You can also use this on a per-file basis.
```bash
git diff
```
- Now that we are sure we made the correct change, **stage** the changes:
```bash
git add README.md
```
- Once your change is staged, we can **commit** the changes:
```bash
git commit -m "your change description"
```

> **Tips**
> - **Staging** lets you choose exactly which changes to **commit**
> - Only commit changes you want in your repository history
> - Avoid `git add .` unless you're sure you want all changes staged
> - Write commit messages that complete: "If applied, this commit will..."
> - Keep commits small and focused (~50 character descriptions)

- You have now modified the git history on your local machine.  To push these changes to GitHub, use
```bash
git push
```
- Verify that you have corrected the typo from the web-interface of GitHub.

We typically add a few additional steps when collaborating with others using Git.  As you can imagine,
each person changing the main code at will could get hectic and result in conflicts.  To fix this,
we use **branches**, which are essentially copies of the main code & history that changes can be made to
in a contained manner.  When development is complete, these branches can be **merged** into the main code via a **pull request**.

We will see this workflow later!

# **Phase 2: Run the simulator (Complete By: 09/25)**
In this section, you will build this sample code, then download and run the simulator we use for development and testing.

> NOTE   
> You will often run across the terms 'workspace', 'overlay' and 'sourcing'.
> In any terminal you open and expect to use installed ROS2 packages,
> you must 'source' ROS2 via `source /opt/ros/humble/setup.bash`.  This
> allows your terminal to "see" ROS2 binaries.
>
> When developing ROS2 code, you will create a 'workspace' which typically
> has a `src` directory containing source code for one or more ROS2 packages.
> Once you build this workspace, you can source it as an 'overlay' with `source install/setup.bash`.
> This means that all installed ROS2 packages are available in your terminal, in addition
> to those in your 'overlay' from your built workspace.

## **Building Code**
To interpret the custom ROS2 interfaces provided by the simulator, we must compile and source them.

- Change directory into this repository
- Source the ROS2 environment and build:
```bash
source /opt/ros/humble/setup.bash
colcon build
```
- Once the build is complete, you can source the workspace as an overlay, which will allow you to use the custom messages
```bash
source install/setup.bash
```
> **TIP**   
> From now on, you can simply source the `install/setup.bash` file in this repository and it will automatically source the base (i.e. `/opt/ros/humble/setup.bash`) as well.

## **Starting the Simulation**
- Download the latest release of PAIRSIM (not the headless version) from [here](https://github.com/Purdue-AI-Racing/Purdue-AI-Racing-Simulator/releases)
- Extract the folder and double click `PAIRSIM_v<version>.x86_64`
- Click "Scenario Setup", select your favorite track in the bottom left, tick "Hot start raptor", then "Drive"
- View all of the available topics!  You will see both sensors, vehicle dynamic topics, as well as vehicle inputs.
```bash
ros2 topic list
```
- To make the car move, we are most interested in the `/vehicle_inputs` topic.  Lets explore this a bit more
- First, we need to confirm the message type.
```bash
ros2 topic info /vehicle_inputs
```
> **TIP**   
> You may notice "Subscription count: 1", this is the simulator listening for inputs.
> You can confirm this by running:
> ```bash
> ros2 topic info --verbose /vehicle_inputs
> ```
> You should now see that PAIRSIM is subscribed to this topic.

- Now that we know the message type, we can explore it's fields:
```bash
ros2 interface show autonoma_msgs/msg/VehicleInputs
```
- To make the car move forward, we should focus on sending a message with a non-zero `throttle_cmd`.
```bash
ros2 topic pub /vehicle_inputs autonoma_msgs/msg/VehicleInputs "{throttle_cmd: 30.0}"
```
- Uh oh!  You hit a wall.  Try to adjust the VehicleInputs command to escape by adding a steering command or adjusting the throttle.
<details>
 <summary>Solution</summary>
 
```bash
ros2 topic pub /vehicle_inputs autonoma_msgs/msg/VehicleInputs "{throttle_cmd: 100.0, steering_cmd: -250}"
```

</details>

> TIP   
> You can find PAIRSIM documentation [here](https://alvinye9.github.io/Purdue-AI-Racing-Simulator/)

**Please email Josh, Dalseong, Alex or Manuel to let them know you have finished Phase 2**

# **Phase 3: Teleoperation of Simulator (Complete By: 10/9)**
In this section, you will write a package to teleoperate the car using the keyboard in Python.
### **Step 1: Setting up Git**
We will use the full Git development process to add a teleoperation node to this repository.

- Create an Issue from the GitHub web interface titled "Add teleoperation node".  In a real issue, you might add additional details in the text box.
- In the bottom right, you will see a "Development" section.  Hit "Create branch" to create a development branch named in coordination with the issue.  This is where you will do your development.
- In your local repository,
```bash
git fetch origin # This updates branches on your local repository
git checkout <branch name> # This changes your current local branch
```
### **Step 2: Create teleoperation node**
- Follow [these instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) to create a **Python** package in your repository's `src` directory.
- Research methods of grabbing Keyboard input in Python, and implement a node that publishes a `VehicleInputs` message to control the car from keyboard input.
- Use `git add`, `git commit` and `git push` to add all of your changes to your branch.

> **TIPS**   
> - One potential package for keyboard input could be [Pynput](https://pypi.org/project/pynput/).
> - You can start with sample ROS2 publisher/subscriber examples and add onto them!
> - Commit early and often!  When you're on a development branch, you essentially have a sandbox and can use this to help track experimental changes.
> - You can create more branches with `git checkout -b <new_branch_name>` if you'd like to do further experimentation.
> - To determine what branch you're on, you can either check `git status` or `git branch`.

### **Step 3: Creating a Pull Request and  Requesting Review**
- To request review of your work, first use the GitHub website to create a Pull Request.  You would like to merge your development branch into the Main branch.
- Write a description of your solution in the PR box with a meaningful title.
- Add Alex (aschleiz) and Josh (joshyeram) as reviewers.
> **TIP**   
> You can create the PR before you are done with your changes, and open it in a draft state.  To do this,
> click the drop down arrow next to Create Pull request (if not created yet), or under "Reviewers" if already created.
> This is a great way to track changes and allow others to view your work without signaling that it is ready for a full review!

**Please wait for either Alex or Josh's review before moving on to Phase 4!**

# **Phase 4: Autonomous vehicle control (Complete By: 11/6)**
In this section, you will use the "[Sense-Think-Act](https://www.roboticsbook.org/S10_introduction.html)" robotics paradigm to autonomously control the car in simulation.  Specifically, you will create a **localization** and **planning** module, and connect them to an existing **controller**.  You must use the ROS2 and Git tools you have learned previously to develop these from scratch.  **At least one of these modules (localization or planning) must be written in C++.**  The other can be written in Python.

## **Control System Structure**
We have a demo controller structure for you to use.  No modifications should be required, unless you would like to!
<img width="1093" height="386" alt="image" src="https://github.com/user-attachments/assets/9d7043f1-4260-4ac0-b6e7-47edf5187ec2" />

The demo system consists of two main control nodes working in parallel:

**KinControlNode**: Controls steering using Pure Pursuit algorithm to follow the planned trajectory with a look-ahead distance.

**LongControlNode**: Controls vehicle speed using PID to match the desired velocity from the path planner.

Your planner will publish a path to the provided KinControlNode, which will control the car's steering.  Your planner will also provide a desired velocity to the LongControlNode which will control the throttle/brake/gear.

We have also provided you with a launch file in `src/onboarding_launch` of this repo that will launch both controllers.  You can run this, after building and sourcing, with `ros2 launch onboarding_launch launch.py`.  You are expected to update this launch file with additional nodes you create.

## **Localization Module**
The localization module is, in general, responsible for fusing all the car's sensors (i.e. GPS, IMU, Wheel speeds, LiDAR) for the purpose of generating a position, velocity and orientation estimate.  There are many techniques to do this, such as Kalman filtering, which we use on the real car.  For the purposes of this exercise, you can assume that all sensor data from the car is ground truth, meaning you do not need to employ a statistical filtering technique.

A general outline for this module:
- Subscribe to a topic with GPS Latitude and Longitude
- Subscribe to a topic with GPS Velocity/Ground track
- Use Local ENU coordinate transformation to transform latitude and longitude into a local coordinate frame in meters
- Use TF2 to broadcast a transform from world to car.
> NOTE   
> You may also choose to publish an Odometry message as well.  This can be plotted using [PlotJuggler](https://plotjuggler.io/) for debugging.

- Optionally, integrate GPS velocity to provide position estimates at a higher rate (or, use IMU to provide Position/Velocity/Orientation at higher rate)

> TIPS   
> - Check out the [ROS standard](https://www.ros.org/reps/rep-0105.html) for coordinate conventions!
> - You must choose a Datum for your Local ENU transformation, this should be a point near the track.  Good choices would be spawn point of vehicle or first point in map file.
> - For Python, you can look at [PyProj](https://pyproj4.github.io/pyproj/stable/) for Geodetic transformations
> - For C++, you can use [GeographicLib](https://geographiclib.sourceforge.io) for Geodetic transformations
> - Feel free to poke around the on-vehicle stack's `navigation_filter` package for some ideas, but your solution should be significantly less complex!
> - Your solution should be <100 source lines of code (SLOC)

## **Planning Module**
Your Planning module will listen to your Localization module's output (likely through a TF2 buffer), and it will pass the controller a local path to follow.  This local path will just be a chunk of a global path transformed into the car's frame.  You will read in the global path and desired velocity from a file.  You are encouraged to look through the demo control system code (see below, and kin_control/long_control packages in repo) as well as the diagram below to understand the interface.

**Global Map**
We provide two global maps in the `map` folder of this repository:
- **LVMS**: An oval centerline path (simpler track for initial testing)
- **Monza**: A complex road course with varying speeds

**Map File Format:**
- **Column 1**: Latitude coordinates
- **Column 2**: Longitude coordinates  
- **Column 3** (Monza only): Reference speed at each waypoint

> **Note on Reference Speeds**  
> The Monza reference speeds come from offline optimization since maintaining constant speed on a road course is impossible. You can use these values directly or scale them down to prevent the car from spinning out during testing.
> 
> For LVMS, feel free to send a constant or ramped speed reference. Just try not to make it full throttle + full steer initially.

A general outline for this module:
- Load in a Path's lat/lon/vel from a .csv file (see provided 'maps' folder).
- Transform the entire path into Local ENU
- Create a TF2 buffer and/or subscribe to Localization's Odometry topic
- Using a timer at a specified rate:
  - Find the closest point in the global path to the car
  - Transform the next X meters of path into the car's frame using TF2
  - Publish this as a Path message type to the topic required by the controller
  - Publish the desired velocity for the closest point as required by controller

> TIPS   
> - Break this into chunks!  Start by writing functions to handle .csv loading and processing independent of ROS, and combine them once you have verified it is working
> - There is less documentation for TF2's transformation functions in Python than in C++.  You often must read Python source code to figure out how to use those functions.
>  - For C++, you can find an API reference [here](https://docs.ros.org/en/noetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html)
>  - For Python, refer to examples [here](https://github.com/ros2/geometry2/tree/rolling/examples_tf2_py) and for transformations see [here](https://github.com/ros2/geometry2/blob/1fd8b0feb1d152512c3c8c09eda0a2930a5cdbb2/tf2_geometry_msgs/src/tf2_geometry_msgs/tf2_geometry_msgs.py#L270)
> - Feel free to poke through the on-vehicle stack's `path_server_overhaul` package for some ideas, as with before your solution should not be this complex.
> - You can use [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) to visualize your published path in different frames!


## **Adapter Node**
We're so close.  The controller's publish separate commands to joystick topics, but the simulator requires a single `VehicleInputs` topic to work.
Write a simple Python node to subscribe to the output of the `kin_control` and `long_control` nodes and republish them in the correct topic as you did with the teleop node.


## **Key Interfaces**

**Inputs to Controller:**
- `/powertrain_data` - Vehicle powertrain status contains RPM, GEAR, etc.  This is provided by PAIRSIM already.
- `/vehicle_data` - Contain current wheel speeds (Note: unit of this is in KMPH).  This is provided by PAIRSIM already.
- `/planning/desired_velocity` - Target speed from planner. This is something you need to implement in the planner
- `/planning/front_path` - Path waypoints to follow. This is something you need to implement in the planner
> NOTE   
> This is a local path in the vehicle's base_link frame, meaning the vehicle's current position is always at x=0, y=0. Path points ahead of the vehicle have positive x values.

**Outputs from Controller:**
- `/joystick/steering_cmd` - Steering angle commands (Handwheel, -240 to 240 deg, steering ratio is 15.015, handweel angle / 15.015 = front wheel angle)
- `/joystick/accelerator_cmd` - Throttle commands  (Percentage, 0 to 100)
- `/joystick/brake_cmd` - Brake commands (Kpa)
- `/joystick/gear_cmd` - Gear selection (1 - 6)

## **Additional Requirements**
In addition to the above, you should:
- Create a launch file to launch everything at once
- Use parameters for:
  - Local ENU datum (shared between Localization and Planner)
  - Map file .csv
  - Anything else you find yourself changing!
- You should have at least one parameter file to specify the above

## **Validation**
Complete one lap around either Monza or LVMS.  You may find that the controller is unable to track the speed profile at Monza.  
A good solution would be to add a speed profile scalar parameter to your planner, so you can adjust how fast you are going using `dynamic_reconfigure`!

> TIPS   
> - If you believe you are publishing the path and desired velocities correctly, but the car is not following the path as expected:
>   - Use [rqt_graph](https://wiki.ros.org/rqt_graph) to check topic connections between nodes
>   - Use [Plotjuggler](https://github.com/facontidavide/PlotJuggler) to check real-time topic outputs
>   - As previously mentioned, you can use RViz to view your planner's output path.
> - You can use [rqt_reconfigure](http://wiki.ros.org/dynamic_reconfigure) to change some controller parameters real-time

Add a PR with Josh, Dalseong, Alex or Manuel  to review.  Good job!

# **Phase 5: Contribute to the on-vehicle stack! (Complete By: 11/20)**
Now you're ready to make your first contribution to the competition stack!  We'll start by getting the on-vehicle stack running with the simulator, and then you will find and make a simple fix to a module of your choice.

### **Building and running the on-vehicle stack**
When members get to this point, we will hold a demo session going over the below steps.  Always feel free to reach out, most issues are small intricacies that will take a long time to debug yourself.
- Read through the [README](https://github.com/Purdue-AI-Racing/on-vehicle) of the on-vehicle stack
- Follow the setup instructions and native build section to build and launch the stack
- Follow the PAIRSIM on-vehicle instructions [here](https://github.com/Purdue-AI-Racing/on-vehicle/wiki/Running-PAIRSIM-with-on%E2%80%90vehicle)

### Finding an Issue in On-Vehicle

This should be one of the easiest parts 😉  

1. **Pick a module**  
   Choose an area that interests you: control, localization, perception, planning, etc.

2. **Explore a package**  
   Select a package within that module and read through some of the code.
   - If you find a legitimate bug or issue, great!  
   - Otherwise, look for something simple to improve, such as:
     - A typo  
     - An unclear comment  
     - Extra whitespace  
     - Any other minor cleanup

3. **Make your change**  
   - Create a new issue and branch.  
   - Make your edit.  
   - Submit a pull request (PR) following the instructions in the on-vehicle `README`.

4. **Verify and test**  
   - Build the code.  
   - Run it in simulation to confirm everything still works.

5. **Watch the checks**  
   After creating your PR, GitHub will automatically run checks to ensure your changes:  
   - Compile successfully  
   - Pass all existing tests

6. **Done!**  
   Once your PR is reviewed and merged, You've completed onboarding!!

