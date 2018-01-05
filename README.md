3jsbot
======

Implementation of robot kinematics, control, and decision making in HTML5/JavaScript and threejs

## Getting Started

1. Clone the repo to an easly accessible location on your machine.
2. Open cs148/home.html
3. Play around!

#### Controls

* __WASD__ - Forward, Rotate Left, Backward, Rotate Right
* __QE__ - Strafe Left, Strafe Right
* __M__ - Generate motion plan using Rapidly-Exploring Random Trees between Robot and Goal (X on the ground)
* __NB__ - Move forward and backward through the steps of the motion plan
* __P__ - Show goal box and move robot endeffector to match its position
* __RF__ - Move the goal box up and down

#### Changing Worlds

There are a few different worlds available that make the motion plan generation more complicated. The available worlds are in the cs148/worlds folder. The world is specified at line 158 of cs148/home.html:
```
line 158 <script src="worlds/world_basic.js"></script>
```
You can change this to point to any of the worlds in the cs148/worlds folder.

### Prerequisites

This currently only works on the Firefox or Edge browsers. I haven't had the chance to test it on Safari, but on Chrome it does not render correctly.
