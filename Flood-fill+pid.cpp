// Pin Definitions
const int leftMotorPin = 9;
const int rightMotorPin = 10;
const int irSensorPins[] = {A0, A1, A2, A3, A4};  // 5 IR sensor pins

// PID Variables
float Kp = 5.0, Ki = 0.0, Kd = 15.0;  // PID constants (adjust as necessary)
float prevError = 0.0, integral = 0.0;

// Grid and Positioning
const int gridSize = 16  // 18x18 grid
int maze[gridSize][gridSize];  // Grid representing the maze
int robotX = 0, robotY = 0;   // Starting position of the robot
int goalX = (gridSize / 2)-1, goalY = (gridSize / 2)-1;  // Goal is at the center of the maze

// Flood Fill Algorithm to generate the path map
void floodFill(int x, int y) {
    // Initialize the maze grid (cost 0 for goal, infinite cost for walls, 1 for free space)
    int visited[gridSize][gridSize] = {0};  // 0 means not visited, 1 means visited
    int cost[gridSize][gridSize];  // Stores cost (distance to the goal)
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            if (maze[i][j] == 1) {  // Wall
                cost[i][j] = 1000;  // High cost for walls
            } else {
                cost[i][j] = -1;  // Unvisited space
            }
        }
    }

    // Goal position
    cost[goalX][goalY] = 0;  // Goal has cost 0

    // Flood fill starting from the goal position
    int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};  // Right, Down, Left, Up

    // Simple queue for flood fill
    int queue[gridSize * gridSize][2];  // Queue to process the points
    int head = 0, tail = 0;
    queue[tail][0] = goalX;
    queue[tail][1] = goalY;
    tail++;

    while (head < tail) {
        int cx = queue[head][0];
        int cy = queue[head][1];
        head++;

        // Explore neighbors
        for (int i = 0; i < 4; i++) {
            int nx = cx + directions[i][0];
            int ny = cy + directions[i][1];

            // Check bounds and if the neighbor has not been visited yet
            if (nx >= 0 && nx < gridSize && ny >= 0 && ny < gridSize && maze[nx][ny] == 0 && visited[nx][ny] == 0) {
                visited[nx][ny] = 1;
                cost[nx][ny] = cost[cx][cy] + 1;
                queue[tail][0] = nx;
                queue[tail][1] = ny;
                tail++;
            }
        }
    }

    // Now the cost array has the shortest path costs to the goal
    // You can use this to guide the robot towards the goal
}

// PID Control Variables
float controlX = 0.0, controlY = 0.0;

// Motor Control
void moveRobot(float controlX, float controlY) {
    int motorSpeed = 255;  // Maximum speed
    int leftSpeed = motorSpeed - controlX - controlY;
    int rightSpeed = motorSpeed + controlX + controlY;

    // Constrain speeds to valid motor values (0-255)
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Set motor speeds (adjust for your motor driver)
    analogWrite(leftMotorPin, leftSpeed);
    analogWrite(rightMotorPin, rightSpeed);
}

// Read IR Sensors
void readIRSensors(int sensorPins[], int sensorCount) {
    int sensorValues[sensorCount];
    for (int i = 0; i < sensorCount; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
    }

    // For simplicity, assuming a threshold-based decision for obstacle detection
    for (int i = 0; i < sensorCount; i++) {
        if (sensorValues[i] > 500) {  // Threshold for detecting obstacles (adjust as necessary)
            // If the IR sensor detects a wall, adjust robot movement
            // Simple avoidance logic: back up a little and turn
            moveRobot(-controlX, -controlY);  // Back up
            delay(200);
            moveRobot(-controlX, controlY);  // Turn to avoid wall
            delay(200);
        }
    }
}

// Calculate PID values
float calculatePID(float target, float current) {
    float error = target - current;
    integral += error;
    float derivative = error - prevError;
    prevError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    return output;
}

// Simple function to check if the robot has reached the goal
bool isAtGoal() {
    return (robotX == goalX && robotY == goalY);
}

// Movement logic to navigate the grid
void navigate() {
    // Calculate PID outputs for X and Y direction based on robot position and goal position
    controlX = calculatePID(goalX, robotX);
    controlY = calculatePID(goalY, robotY);

    // Avoid obstacles based on IR sensors
    readIRSensors(irSensorPins, 5);

    // Move the robot using the control values
    moveRobot(controlX, controlY);

    // Update robot position (for demonstration, we'll assume robot moves towards goal)
    // You need to implement proper movement based on sensors and pathfinding
    if (controlX > 0) {
        robotX++;
    } else if (controlX < 0) {
        robotX--;
    }
    if (controlY > 0) {
        robotY++;
    } else if (controlY < 0) {
        robotY--;
    }
}

// Setup function
void setup() {
    Serial.begin(9600);
    // Set motor pins as output
    pinMode(leftMotorPin, OUTPUT);
    pinMode(rightMotorPin, OUTPUT);

    // Initialize the maze grid (for simplicity, you can define it as a 2D array)
    generateGrid();

    // Run flood fill to calculate the path
    floodFill(robotX, robotY);
}

// Main loop function
void loop() {
    if (isAtGoal()) {
        Serial.println("Goal reached!");
        moveRobot(0, 0);  // Stop robot when goal is reached
    } else {
        navigate();  // Keep navigating
    }

    delay(100);  // Small delay for smoother control
}
