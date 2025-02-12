# MICROMOUSE

## Introduction
MICROMOUSE is an autonomous robot designed for the International Micromouse Challenge 2022 at IIT Bombay. The challenge involves building a robot capable of rapidly navigating a labyrinth to reach the center in the shortest time possible.

### Project Highlights
- **Techfest, IIT Bombay**
- **Duration:** Oct 2022 - Dec 2022


## Overview
Designed and built an autonomous bot with 18cm square cells set over a 16 x 16 grid, achieving the least traversal time to the goal in the maze-solving challenge.

## Technical Details
- Fabricated a custom PCB to boost the bot's performance through integrated electronics.
- Incorporated six sharp infrared (IR) sensors for superior data accuracy during maze exploration.
- Executed maze-solving algorithms, such as flood fill and wall-following, to determine the quickest and most efficient route to the maze's center.
- Implemented PID control for precise and smooth movement, optimizing the robot's path-finding capabilities.

## Code Overview
The project utilizes Arduino Nano and two tires for a compact and speedy design.The code files consist of a wall-following algorithm and on ewit flood fill algorithm. For guaranteed finding of the goal, one must prefer flood fill algorithm. Wall folowing algorithm is successful only only in cases when the goal lies at one of the edges adjacent to the wall i.e on the outer most edges.



