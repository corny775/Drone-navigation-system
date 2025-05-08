#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd_1(0x27, 16, 2);
LiquidCrystal_I2C lcd_2(0x26, 16, 2);

int pictureServoPin = 3;
Servo pictureServo;

int echo = 5, trig = 4;
float groundDist, groundTime;

int pic = 0;

int topLeftEnable = 6;
int topLeftPositive = A0;
int topLeftNegative = A1;

int topRightEnable = 10;
int topRightPositive = A2;
int topRightNegative = A3;

int bottomLeftEnable = 9;
int bottomLeftPositive = 2;
int bottomLeftNegative = 13;

int bottomRightEnable = 11;
int bottomRightPositive = 7;
int bottomRightNegative = 8;

String opInput = "hover";
String lastManualCommand = "hover";
bool isNavigating = false;
const int MAX_NODES = 10;
int nodeCount = 0;

float currentX = 0;
float currentY = 0;
float destinationX = 0;
float destinationY = 0;

float nodeX[MAX_NODES];
float nodeY[MAX_NODES];
int graph[MAX_NODES][MAX_NODES];
int nextNode;
float distToDestination;

float arrivalThreshold = 5.0;
unsigned long lastNavUpdate = 0;
unsigned long manualOverrideTimeout = 0;

int visited[MAX_NODES];
float distance[MAX_NODES];
int previous[MAX_NODES];

void setup() {
    Serial.begin(9600);
    
    Wire.begin();
    
    lcd_1.init();
    lcd_1.backlight();
    lcd_1.clear();
    lcd_1.setCursor(0, 0);
    lcd_1.print("Drone Ready");
    lcd_1.setCursor(0, 1);
    lcd_1.print("Warns: 0");
    
    lcd_2.init();
    lcd_2.backlight();
    lcd_2.clear();
    lcd_2.setCursor(0, 0);
    lcd_2.print("Drone Ready");
    lcd_2.setCursor(0, 1);
    lcd_2.print("dest: 0");

    pinMode(pictureServoPin, OUTPUT);
    pictureServo.attach(pictureServoPin);
    pictureServo.write(0);

    pinMode(topLeftEnable, OUTPUT);
    pinMode(topLeftPositive, OUTPUT);
    pinMode(topLeftNegative, OUTPUT);

    pinMode(topRightEnable, OUTPUT);
    pinMode(topRightPositive, OUTPUT);
    pinMode(topRightNegative, OUTPUT);

    pinMode(bottomLeftEnable, OUTPUT);
    pinMode(bottomLeftPositive, OUTPUT);
    pinMode(bottomLeftNegative, OUTPUT);

    pinMode(bottomRightEnable, OUTPUT);
    pinMode(bottomRightPositive, OUTPUT);
    pinMode(bottomRightNegative, OUTPUT);

    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  
    Serial.println("Drone system initialized");
}

void loop() {
    checkSerialInput();

    updateDistanceSensor();

    if (isNavigating) {
        distToDestination = calculateDistance(currentX, currentY, destinationX, destinationY);
        
        if (millis() > manualOverrideTimeout) {
            if (millis() - lastNavUpdate > 500) {
                determineMovementDirection();
                lastNavUpdate = millis();
            }
        }
    }
    
    displayStatus();

    executeMovement();
    
    handleCameraControl();
    
    delay(100);
}

void checkSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.startsWith("goto ")) {
            String coords = input.substring(5);
            int commaIndex = coords.indexOf(',');
            if (commaIndex != -1) {
                destinationX = coords.substring(0, commaIndex).toFloat();
                destinationY = coords.substring(commaIndex + 1).toFloat();
                Serial.print("New destination set: ");
                Serial.print(destinationX);
                Serial.print(",");
                Serial.println(destinationY);
                
                generateNavigationGrid();
                
                calculatePath();
                isNavigating = true;
            }
        } 
        else if (input == "forward" || input == "back" || input == "left" || 
                 input == "right" || input == "up" || input == "down" || input == "hover") {
            opInput = input;
            lastManualCommand = input;
            manualOverrideTimeout = millis() + 3000;
            Serial.print("Manual command: ");
            Serial.println(opInput);
        }
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

    updateLCDDisplay();
}

void updateLCDDisplay() {
    lcd_1.clear();
    lcd_1.setCursor(0, 0);
    lcd_1.print("Dst:");
    lcd_1.print(distToDestination, 1);
    lcd_1.print("cm");
    lcd_1.setCursor(0, 1);
    lcd_1.print("Alt:");
    lcd_1.print(groundDist, 1);
    lcd_1.print("cm W:");
    lcd_1.print(pic);
    
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
    if (opInput == "hover") {
        move(0, 0, 0, 0);
    } else if (opInput == "forward") {
        move(0, 0, 150, 150);
        currentY += 2;
    } else if (opInput == "back") {
        move(150, 150, 0, 0);
        currentY -= 2;
    } else if (opInput == "up") {
        move(140, 140, 140, 140);
    } else if (opInput == "down") {
        move(50, 50, 50, 50);
    } else if (opInput == "right") {
        move(150, 0, 150, 0);
        currentX += 2;
    } else if (opInput == "left") {
        move(0, 150, 0, 150);
        currentX -= 2;
    }
    
    if (isNavigating && distToDestination < arrivalThreshold) {
        opInput = "hover";
        isNavigating = false;
        Serial.println("Destination reached!");
    }
}

void handleCameraControl() {
    if (groundDist > 60) {
        pictureServo.write(90);
        delay(200);
        pictureServo.write(0);
        
        Serial.println("Picture Taken - Above high limit!");
        pic = pic + 1;
        
        lcd_1.clear();
        lcd_1.setCursor(0, 0);
        lcd_1.print("ABOVE HIGH LIMIT!");
        lcd_1.setCursor(0, 1);
        lcd_1.print("Picture Taken");
        delay(1000);
        
        updateLCDDisplay();
    }
}

void move(int TL, int TR, int BL, int BR) {
    analogWrite(topLeftEnable, TL);
    digitalWrite(topLeftPositive, HIGH);
    digitalWrite(topLeftNegative, LOW);

    analogWrite(topRightEnable, TR);
    digitalWrite(topRightPositive, HIGH);
    digitalWrite(topRightNegative, LOW);

    analogWrite(bottomLeftEnable, BL);
    digitalWrite(bottomLeftPositive, HIGH);
    digitalWrite(bottomLeftNegative, LOW);

    analogWrite(bottomRightEnable, BR);
    digitalWrite(bottomRightPositive, HIGH);
    digitalWrite(bottomRightNegative, LOW);
}

void generateNavigationGrid() {
    nodeCount = 0;
    
    nodeX[nodeCount] = currentX;
    nodeY[nodeCount] = currentY;
    nodeCount++;
    
    nodeX[nodeCount] = destinationX;
    nodeY[nodeCount] = destinationY;
    nodeCount++;
    
    float minX = min(currentX, destinationX);
    float maxX = max(currentX, destinationX);
    float minY = min(currentY, destinationY);
    float maxY = max(currentY, destinationY);
    
    minX -= 20;
    maxX += 20;
    minY -= 20;
    maxY += 20;
    
    float gridSpacing = 40.0;
    
    for (float x = minX; x <= maxX && nodeCount < MAX_NODES - 1; x += gridSpacing) {
        for (float y = minY; y <= maxY && nodeCount < MAX_NODES; y += gridSpacing) {
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
                
                if (nodeCount >= MAX_NODES) {
                    break;
                }
            }
        }
    }
    
    Serial.print("Navigation grid created with ");
    Serial.print(nodeCount);
    Serial.println(" nodes");
    
    initializeGraph();
}

void initializeGraph() {
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            graph[i][j] = 0;
        }
    }
    
    for (int i = 0; i < nodeCount; i++) {
        for (int j = 0; j < nodeCount; j++) {
            if (i != j) {
                float dist = calculateDistance(nodeX[i], nodeY[i], nodeX[j], nodeY[j]);
                if (dist <= 50) {
                    graph[i][j] = (int)dist;
                }
            }
        }
    }
}

float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

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

void determineMovementDirection() {
    if (!isNavigating) {
        return;
    }
    
    if (distToDestination < arrivalThreshold) {
        opInput = "hover";
        isNavigating = false;
        Serial.println("Destination reached!");
        return;
    }
    
    float targetX, targetY;
    
    if (distToDestination < 20) {
        targetX = destinationX;
        targetY = destinationY;
    } else {
        targetX = nodeX[nextNode];
        targetY = nodeY[nextNode];
        
        if (calculateDistance(currentX, currentY, targetX, targetY) < 10) {
            calculatePath();
            targetX = nodeX[nextNode];
            targetY = nodeY[nextNode];
        }
    }
    
    float deltaX = targetX - currentX;
    float deltaY = targetY - currentY;
    
    if (abs(deltaX) > abs(deltaY)) {
        if (deltaX > 2) {
            opInput = "right";
        } else if (deltaX < -2) {
            opInput = "left";
        } else {
            opInput = "hover";
        }
    } else {
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

void calculatePath() {
    int startNode = findClosestNode(currentX, currentY);
    int endNode = findClosestNode(destinationX, destinationY);
    
    for (int i = 0; i < nodeCount; i++) {
        visited[i] = 0;
        distance[i] = 999999;
        previous[i] = -1;
    }
    
    distance[startNode] = 0;
    
    for (int count = 0; count < nodeCount; count++) {
        int minDist = 999999;
        int u = -1;
        
        for (int i = 0; i < nodeCount; i++) {
            if (!visited[i] && distance[i] < minDist) {
                minDist = distance[i];
                u = i;
            }
        }
        
        if (u == -1) break;
        
        visited[u] = 1;
        
        for (int v = 0; v < nodeCount; v++) {
            if (!visited[v] && graph[u][v] && 
                distance[u] + graph[u][v] < distance[v]) {
                distance[v] = distance[u] + graph[u][v];
                previous[v] = u;
            }
        }
    }
    
    if (previous[endNode] == -1) {
        Serial.println("No path found - using direct approach");
        nextNode = endNode;
    } else {
        int current = endNode;
        while (previous[current] != startNode && previous[current] != -1) {
            current = previous[current];
        }
        
        if (previous[current] == startNode) {
            nextNode = current;
        } else {
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