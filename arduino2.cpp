#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// LCD Screens
LiquidCrystal_I2C lcd_1(0x27, 16, 2);  // Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd_2(0x26, 16, 2);  // Set the LCD address to 0x26 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd_3(0x26, 16, 2);

// Camera Servo
int pictureServoPin = 3;
Servo pictureServo;

// DISTANCE
int echo = 5, trig = 4;
float groundDist, groundTime;

// Picture Counter
int pic = 0;

// DRONE - Using L293D motor driver pins
// Motor A - Top Left
int topLeftEnable = 6;   // PWM pin for speed control
int topLeftPositive = A0; // Using analog pins as digital output
int topLeftNegative = A1;

// Motor B - Top Right
int topRightEnable = 10;  // PWM pin for speed control
int topRightPositive = A2;
int topRightNegative = A3;

// Motor C - Bottom Left
int bottomLeftEnable = 9;  // PWM pin for speed control
int bottomLeftPositive = 2;
int bottomLeftNegative = 13;

// Motor D - Bottom Right
int bottomRightEnable = 11; // PWM pin for speed control
int bottomRightPositive = 7; // Changed from conflicting pins
int bottomRightNegative = 8; // Changed from conflicting pins

// Navigation system variables
String opInput = "hover";
String lastManualCommand = "hover";
bool isNavigating = false;    // Flag to track if we're in navigation mode
const int MAX_NODES = 10;     // Reduced max nodes to fit in Arduino memory
int nodeCount = 0;            // Will be set dynamically based on map size

// Position tracking (simulated GPS)
float currentX = 0;        // Current X position (cm)
float currentY = 0;        // Current Y position (cm)
float destinationX = 0;    // Target X position (cm)
float destinationY = 0;    // Target Y position (cm)

// Node graph for navigation (simulated map)
float nodeX[MAX_NODES];    // X coordinates of nodes
float nodeY[MAX_NODES];    // Y coordinates of nodes
int graph[MAX_NODES][MAX_NODES];  // Adjacency matrix for the graph
int nextNode;                     // Next node in the optimal path
float distToDestination;          // Distance to final destination

// Precision control
float arrivalThreshold = 5.0;     // Distance threshold to consider arrived at destination
unsigned long lastNavUpdate = 0;  // Time of last navigation update
unsigned long manualOverrideTimeout = 0; // Timeout for manual override

// Dijkstra's algorithm variables
int visited[MAX_NODES];
float distance[MAX_NODES];
int previous[MAX_NODES];

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    
    // Initialize I2C communication
    Wire.begin();
    
    // Initialize LCD screens
    lcd_1.init();                      // Initialize the LCD
    lcd_1.backlight();                 // Turn on backlight
    lcd_1.clear();                     // Clear any previous content
    lcd_1.setCursor(0, 0);
    lcd_1.print("Drone Ready");
    lcd_1.setCursor(0, 1);
    lcd_1.print("Warn: 0");          // Display initial snap count
    
    lcd_2.init();                      // Initialize the LCD
    lcd_2.backlight();                 // Turn on backlight
    lcd_2.clear();                     // Clear any previous content
    lcd_2.setCursor(0, 0);
    lcd_2.print("Drone Ready");
    lcd_2.setCursor(0, 1);
    lcd_2.print("dest: none");          // Display initial snap count

    // Camera Picture Servo
    pinMode(pictureServoPin, OUTPUT);
    pictureServo.attach(pictureServoPin);
    pictureServo.write(0);

    // Drone Motors with L293D Motor Driver
    // Top Left Motor
    pinMode(topLeftEnable, OUTPUT);
    pinMode(topLeftPositive, OUTPUT);
    pinMode(topLeftNegative, OUTPUT);

    // Top Right Motor
    pinMode(topRightEnable, OUTPUT);
    pinMode(topRightPositive, OUTPUT);
    pinMode(topRightNegative, OUTPUT);

    // Bottom Left Motor
    pinMode(bottomLeftEnable, OUTPUT);
    pinMode(bottomLeftPositive, OUTPUT);
    pinMode(bottomLeftNegative, OUTPUT);

    // Bottom Right Motor
    pinMode(bottomRightEnable, OUTPUT);
    pinMode(bottomRightPositive, OUTPUT);
    pinMode(bottomRightNegative, OUTPUT);

    // Distance Sensor
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    
    Serial.println("Drone system initialized");
}

void loop() {
    // Read serial input (high priority)
    checkSerialInput();

    // Distance Sensor update
    updateDistanceSensor();

    // Calculate distance to destination if in navigation mode
    if (isNavigating) {
        distToDestination = calculateDistance(currentX, currentY, destinationX, destinationY);
        
        // If manual override has timed out, determine movement
        if (millis() > manualOverrideTimeout) {
            // Only update navigation logic every 500ms to avoid jitter
            if (millis() - lastNavUpdate > 500) {
                determineMovementDirection();
                lastNavUpdate = millis();
            }
        }
    }
    
    // Display position and status information
    displayStatus();

    // Execute movement based on command
    executeMovement();
    
    // Camera control logic
    handleCameraControl();
    
    // Short delay to prevent overwhelming the system
    delay(100);
}

void checkSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove any whitespace/newlines
        
        // Check for goto command
        if (input.startsWith("goto ")) {
            String coords = input.substring(5); // Remove "goto " prefix
            int commaIndex = coords.indexOf(',');
            if (commaIndex != -1) {
                destinationX = coords.substring(0, commaIndex).toFloat();
                destinationY = coords.substring(commaIndex + 1).toFloat();
                Serial.print("New destination set: ");
                Serial.print(destinationX);
                Serial.print(",");
                Serial.println(destinationY);
                
                // Generate a navigation grid that includes the current position and destination
                generateNavigationGrid();
                
                // Calculate the best path using Dijkstra's algorithm
                calculatePath();
                isNavigating = true;
            }
        } 
        // Check for manual control commands
        else if (input == "forward" || input == "back" || input == "left" || 
                 input == "right" || input == "up" || input == "down" || input == "hover") {
            opInput = input;
            lastManualCommand = input;
            manualOverrideTimeout = millis() + 3000;  // Manual control for 3 seconds
            Serial.print("Manual command: ");
            Serial.println(opInput);
        }
        // Check for stop navigation command
        else if (input == "stop") {
            isNavigating = false;
            opInput = "hover";
            Serial.println("Navigation stopped");
        }
        else {
            Serial.println("Invalid input");
        }
    }
}

void updateDistanceSensor() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    groundTime = pulseIn(echo, HIGH);
    groundDist = groundTime/148.1;
}

void displayStatus() {
    // Print status to Serial
    Serial.print("Current position: ");
    Serial.print(currentX);
    Serial.print(",");
    Serial.println(currentY);
    
    if (isNavigating) {
        Serial.print("Distance to destination: ");
        Serial.println(distToDestination);
    }
    
    Serial.print("Current command: ");
    Serial.println(opInput);
    
    Serial.print("Ground Distance: ");
    Serial.println(groundDist);

    // Update LCD display
    updateLCDDisplay();
}

// Function to update LCD display with consistent format
void updateLCDDisplay() {
    // Update main LCD (lcd_1)
    lcd_1.clear();
    lcd_1.setCursor(0, 0);
    lcd_1.print("Dst:");
    lcd_1.print(distToDestination, 1);
    lcd_1.print("cm");
    lcd_1.setCursor(0, 1);
    lcd_1.print("Alt:");
    lcd_1.print(groundDist, 1);
    lcd_1.print("cm P:");
    lcd_1.print(pic);
    
    // Update secondary LCD (lcd_2)
    lcd_2.clear();
    lcd_2.setCursor(0, 0);
    lcd_2.print("Pos:");
    lcd_2.print(currentX, 0);
    lcd_2.print(",");
    lcd_2.print(currentY, 0);
    lcd_2.setCursor(0, 1);
    lcd_2.print("Tgt:");
    lcd_2.print(destinationX, 0);
    lcd_2.print(",");
    lcd_2.print(destinationY, 0);
  
  
}

void executeMovement() {
    // Update position based on current movement command
    if (opInput == "hover") {
        move(100, 100, 100, 100);
        // No position change when hovering
    } else if (opInput == "forward") {
        move(50, 50, 150, 150);
        // Update position (simulated movement)
        currentY += 2;
    } else if (opInput == "back") {
        move(150, 150, 50, 50);
        // Update position (simulated movement)
        currentY -= 2;
    } else if (opInput == "up") {
        move(140, 140, 140, 140);
        // No X,Y position change when going up
    } else if (opInput == "down") {
        move(50, 50, 50, 50);
        // No X,Y position change when going down
    } else if (opInput == "right") {
        move(150, 50, 150, 50);
        // Update position (simulated movement)
        currentX += 2;
    } else if (opInput == "left") {
        move(50, 150, 50, 150);
        // Update position (simulated movement)
        currentX -= 2;
    }
    
    // Check if we've reached the destination
    if (isNavigating && distToDestination < arrivalThreshold) {
        opInput = "hover";
        isNavigating = false;
        Serial.println("Destination reached!");
    }
}

void handleCameraControl() {
    // Camera and Picture Counter - Take picture when above certain height
    if (groundDist > 60) {
        // Take picture when above 60cm
        pictureServo.write(90);  // Press the camera button
        delay(200);              // Reduced delay for better responsiveness
        pictureServo.write(0);   // Release the camera button
        
        // Display message via Serial
        Serial.println("Picture Taken - Above high limit!");
        pic = pic + 1;  // Increment picture counter
        
        // Show picture taken message on LCD
        lcd_1.clear();
        lcd_1.setCursor(0, 0);
        lcd_1.print("ABOVE HIGH LIMIT!");
        lcd_1.setCursor(0, 1);
        lcd_1.print("Picture Taken");
        delay(1000);
        
        // Update LCD display after taking picture
        updateLCDDisplay();
    }
}

void move(int TL, int TR, int BL, int BR) {
    // Top Left Motor
    analogWrite(topLeftEnable, TL);       // Control speed with PWM
    digitalWrite(topLeftPositive, HIGH);  // Direction control
    digitalWrite(topLeftNegative, LOW);   // Direction control

    // Top Right Motor
    analogWrite(topRightEnable, TR);
    digitalWrite(topRightPositive, HIGH);
    digitalWrite(topRightNegative, LOW);

    // Bottom Left Motor
    analogWrite(bottomLeftEnable, BL);
    digitalWrite(bottomLeftPositive, HIGH);
    digitalWrite(bottomLeftNegative, LOW);

    // Bottom Right Motor
    analogWrite(bottomRightEnable, BR);
    digitalWrite(bottomRightPositive, HIGH);
    digitalWrite(bottomRightNegative, LOW);
}

// Dynamically generate navigation grid based on current position and destination
void generateNavigationGrid() {
    // Clear the node arrays
    nodeCount = 0;
    
    // Add current position as first node
    nodeX[nodeCount] = currentX;
    nodeY[nodeCount] = currentY;
    nodeCount++;
    
    // Add destination as second node
    nodeX[nodeCount] = destinationX;
    nodeY[nodeCount] = destinationY;
    nodeCount++;
    
    // Add intermediate grid points (create a grid that covers the area)
    float minX = min(currentX, destinationX);
    float maxX = max(currentX, destinationX);
    float minY = min(currentY, destinationY);
    float maxY = max(currentY, destinationY);
    
    // Add margin to ensure grid extends beyond start and end points
    minX -= 20;
    maxX += 20;
    minY -= 20;
    maxY += 20;
    
    // Create grid points - fewer points to save memory
    float gridSpacing = 40.0;  // Increased spacing between grid points
    
    for (float x = minX; x <= maxX && nodeCount < MAX_NODES - 1; x += gridSpacing) {
        for (float y = minY; y <= maxY && nodeCount < MAX_NODES; y += gridSpacing) {
            // Don't add points that are too close to existing nodes
            bool tooClose = false;
            for (int i = 0; i < nodeCount; i++) {
                if (calculateDistance(x, y, nodeX[i], nodeY[i]) < 15) {
                    tooClose = true;
                    break;
                }
            }
            
            if (!tooClose) {
                nodeX[nodeCount] = x;
                nodeY[nodeCount] = y;
                nodeCount++;
                
                // Ensure we don't exceed array bounds
                if (nodeCount >= MAX_NODES) {
                    break;
                }
            }
        }
    }
    
    Serial.print("Navigation grid created with ");
    Serial.print(nodeCount);
    Serial.println(" nodes");
    
    // Initialize the graph connections
    initializeGraph();
}

// Initialize the navigation graph with distances between nodes
void initializeGraph() {
    // Clear the graph
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j] = 0;
        }
    }
    
    // Connect nodes based on proximity
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            if (i != j) {
                float dist = calculateDistance(nodeX[i], nodeY[i], nodeX[j], nodeY[j]);
                // Connect nodes that are close enough (direct connections)
                if (dist <= 50) {  // Connection range
                    graph[i][j] = (int)dist;
                }
            }
        }
    }
}

// Calculate Euclidean distance between two points
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Find the closest node to a given point
int findClosestNode(float x, float y) {
    int closestNode = 0;
    float minDistance = calculateDistance(x, y, nodeX[0], nodeY[0]);
    
    for (int i = 1; i < nodeCount; i++) {
        float dist = calculateDistance(x, y, nodeX[i], nodeY[i]);
        if (dist < minDistance) {
            minDistance = dist;
            closestNode = i;
        }
    }
    
    return closestNode;
}

// Determine which direction to move based on current position and next node
void determineMovementDirection() {
    // Check if we're still in navigation mode
    if (!isNavigating) {
        return;
    }
    
    // If we're very close to destination, just hover
    if (distToDestination < arrivalThreshold) {
        opInput = "hover";
        isNavigating = false;
        Serial.println("Destination reached!");
        return;
    }
    
    // Find out which direction to move to reach the destination
    // Use direct path to destination for final approach or the next node in path
    float targetX, targetY;
    
    if (distToDestination < 20) {
        // Direct approach when close to destination
        targetX = destinationX;
        targetY = destinationY;
    } else {
        // Follow path through nodes
        targetX = nodeX[nextNode];
        targetY = nodeY[nextNode];
        
        // If we're close to current node, recalculate path to get next node
        if (calculateDistance(currentX, currentY, targetX, targetY) < 10) {
            calculatePath();
            targetX = nodeX[nextNode];
            targetY = nodeY[nextNode];
        }
    }
    
    // Calculate direction vectors
    float deltaX = targetX - currentX;
    float deltaY = targetY - currentY;
    
    // Simple movement logic based on biggest difference
    if (abs(deltaX) > abs(deltaY)) {
        // Move horizontally
        if (deltaX > 2) {
            opInput = "right";
        } else if (deltaX < -2) {
            opInput = "left";
        } else {
            opInput = "hover";
        }
    } else {
        // Move vertically
        if (deltaY > 2) {
            opInput = "forward";
        } else if (deltaY < -2) {
            opInput = "back";
        } else {
            opInput = "hover";
        }
    }
    
    Serial.print("Navigation: Moving ");
    Serial.print(opInput);
    Serial.print(" toward ");
    Serial.print(targetX);
    Serial.print(",");
    Serial.println(targetY);
}

// Dijkstra's algorithm implementation
void calculatePath() {
    int startNode = findClosestNode(currentX, currentY);
    int endNode = findClosestNode(destinationX, destinationY);
    
    // Initialize variables for Dijkstra's algorithm
    for (int i = 0; i < nodeCount; i++) {
        visited[i] = 0;
        distance[i] = 999999;  // "Infinity"
        previous[i] = -1;
    }
    
    distance[startNode] = 0;
    
    // Main Dijkstra loop
    for (int count = 0; count < nodeCount; count++) {
        // Find node with minimum distance
        int minDist = 999999;
        int u = -1;
        
        for (int i = 0; i < nodeCount; i++) {
            if (!visited[i] && distance[i] < minDist) {
                minDist = distance[i];
                u = i;
            }
        }
        
        if (u == -1) break;  // No more reachable nodes
        
        visited[u] = 1;
        
        // Update distances to adjacent nodes
        for (int v = 0; v < nodeCount; v++) {
            if (!visited[v] && graph[u][v] && 
                distance[u] + graph[u][v] < distance[v]) {
                distance[v] = distance[u] + graph[u][v];
                previous[v] = u;
            }
        }
    }
    
    // Check if a path was found
    if (previous[endNode] == -1) {
        // No path found, try direct path
        Serial.println("No path found - using direct approach");
        nextNode = endNode;
    } else {
        // Reconstruct the path
        int current = endNode;
        while (previous[current] != startNode && previous[current] != -1) {
            current = previous[current];
        }
        
        // If a path was found, set the next node
        if (previous[current] == startNode) {
            nextNode = current;
        } else {
            // Direct connection
            nextNode = endNode;
        }
    }
    
    Serial.print("Next node in path: ");
    Serial.print(nextNode);
    Serial.print(" at position ");
    Serial.print(nodeX[nextNode]);
    Serial.print(",");
    Serial.println(nodeY[nextNode]);
}
