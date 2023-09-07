/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.

 */

/*
 This Documentaion
 */

#include <cstdio>
#include <cstdlib>

#include <cmath>
#include <iostream>

#include <unistd.h>

#include <ncurses.h>
#include <termios.h>

#include <curses-gfx.h>
#include <curses-gfx-3d.h>

// ROS headers:
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"

/*
 Catch ctrl-c for cleaner exits
 */
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
    keepRunning = false;
}

void cleanupConsole() {
    clear();
    endwin();

    std::cout << "Console has been cleaned!" << std::endl;
}

void setupTerminal()
{
    
    setlocale(LC_ALL, "");
    
    // Start up Curses window
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, 1);    // Don't wait at the getch() function if the user hasn't hit a key
    keypad(stdscr, 1); // Allow Function key input and arrow key input

    start_color();
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_YELLOW, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);
    init_pair(5, COLOR_BLUE, COLOR_BLACK);
    init_pair(6, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(7, COLOR_WHITE, COLOR_BLACK);
    
//    init_pair(1, COLOR_RED, -1);
//    init_pair(2, COLOR_YELLOW, -1);
//    init_pair(3, COLOR_GREEN, -1);
//    init_pair(4, COLOR_CYAN, -1);
//    init_pair(5, COLOR_BLUE, -1);
//    init_pair(6, COLOR_MAGENTA, -1);
//    init_pair(7, COLOR_WHITE, -1);


//    init_pair(5, COLOR_BLACK, COLOR_RED );
//    init_pair(6, COLOR_BLACK, COLOR_GREEN );
//    init_pair(7, COLOR_BLACK, COLOR_CYAN );
//    init_pair(8, COLOR_WHITE, COLOR_BLUE );

    curs_set(0);    // no cursor

//    atexit(destroy);
}


class  SteeringListener {
private:
    ros::NodeHandle* nodeHandle;
    ros::Subscriber subscriber;
    
    double steeringAngle;
public:
    
    SteeringListener(ros::NodeHandle* nh) {
        nodeHandle = nh;
        subscriber = nodeHandle->subscribe("/steering_angle", 1000, &SteeringListener::callbackSteerAngle, this);
    }
    
    void callbackSteerAngle(const std_msgs::Float64::ConstPtr& msg) {
        steeringAngle = -msg->data; // For whatever reason the steering angle reported in ROS is reversed?
    }
    
    void draw(Coordinates2D center, double width, double height) {
        Coordinates2D point1 = center;
        Coordinates2D point2 = {.x = 25, .y=0};
        
        for (double i = 1; i <= 60; i+=1) {
            attron(COLOR_PAIR(4));
            double x = center.x + ((double)width)*sin(i *M_PI*2.0/60.0 + steeringAngle*M_PI/180.0) + 0.5;
            double y = center.y - ((double)height)*cos(i *M_PI*2.0/60.0 + steeringAngle*M_PI/180.0) + 0.5;
            drawDotFloat(x,y);
//            mvaddch(y, x, '*');
            attroff(COLOR_PAIR(4));
        }
        
        point1 = center;
        
        attron(A_BOLD);
        attron(COLOR_PAIR(1));
        point2.x = point1.x + width*sin(steeringAngle*M_PI/180.0) + 0.5;
        point2.y = point1.y - height*cos(steeringAngle*M_PI/180.0) + 0.5;
        ln2(point1, point2);
        attroff(COLOR_PAIR(1));
        attroff(A_BOLD);
        
        
        mvprintw(center.y - (height + 1), center.x - 20/2, "Steering Angle %.01f", steeringAngle);
    }
};

class RadarListener {
private:
    ros::Subscriber subscriberATracks[16];
    
public:
    Coordinates3D points[16];
    
    ros::NodeHandle* nodeHandle;
    RadarListener(ros::NodeHandle* nh) {
        nodeHandle = nh;
        
        subscriberATracks[ 0] = nodeHandle->subscribe("/track_a0", 1000, &RadarListener::callbackRadar0, this);
        subscriberATracks[ 1] = nodeHandle->subscribe("/track_a1", 1000, &RadarListener::callbackRadar1, this);
        subscriberATracks[ 2] = nodeHandle->subscribe("/track_a2", 1000, &RadarListener::callbackRadar2, this);
        subscriberATracks[ 3] = nodeHandle->subscribe("/track_a3", 1000, &RadarListener::callbackRadar3, this);
        subscriberATracks[ 4] = nodeHandle->subscribe("/track_a4", 1000, &RadarListener::callbackRadar4, this);
        subscriberATracks[ 5] = nodeHandle->subscribe("/track_a5", 1000, &RadarListener::callbackRadar5, this);
        subscriberATracks[ 6] = nodeHandle->subscribe("/track_a6", 1000, &RadarListener::callbackRadar6, this);
        subscriberATracks[ 7] = nodeHandle->subscribe("/track_a7", 1000, &RadarListener::callbackRadar7, this);
        subscriberATracks[ 8] = nodeHandle->subscribe("/track_a8", 1000, &RadarListener::callbackRadar8, this);
        subscriberATracks[ 9] = nodeHandle->subscribe("/track_a9", 1000, &RadarListener::callbackRadar9, this);
        subscriberATracks[10] = nodeHandle->subscribe("/track_a10", 1000, &RadarListener::callbackRadar10, this);
        subscriberATracks[11] = nodeHandle->subscribe("/track_a11", 1000, &RadarListener::callbackRadar11, this);
        subscriberATracks[12] = nodeHandle->subscribe("/track_a12", 1000, &RadarListener::callbackRadar12, this);
        subscriberATracks[13] = nodeHandle->subscribe("/track_a13", 1000, &RadarListener::callbackRadar13, this);
        subscriberATracks[15] = nodeHandle->subscribe("/track_a14", 1000, &RadarListener::callbackRadar14, this);
        subscriberATracks[15] = nodeHandle->subscribe("/track_a15", 1000, &RadarListener::callbackRadar15, this);
        
    }
    
    void callbackRadar0(const geometry_msgs::PointStamped::ConstPtr& msg) { points[0].x = msg->point.x;points[0].y = msg->point.y; points[0].z = msg->point.z; }
    void callbackRadar1(const geometry_msgs::PointStamped::ConstPtr& msg) { points[1].x = msg->point.x;points[1].y = msg->point.y; points[1].z = msg->point.z;  }
    void callbackRadar2(const geometry_msgs::PointStamped::ConstPtr& msg) { points[2].x = msg->point.x;points[2].y = msg->point.y; points[2].z = msg->point.z;  }
    void callbackRadar3(const geometry_msgs::PointStamped::ConstPtr& msg) { points[3].x = msg->point.x;points[3].y = msg->point.y; points[3].z = msg->point.z;  }
    void callbackRadar4(const geometry_msgs::PointStamped::ConstPtr& msg) { points[4].x = msg->point.x;points[4].y = msg->point.y; points[4].z = msg->point.z;  }
    void callbackRadar5(const geometry_msgs::PointStamped::ConstPtr& msg) { points[5].x = msg->point.x;points[5].y = msg->point.y; points[5].z = msg->point.z;  }
    void callbackRadar6(const geometry_msgs::PointStamped::ConstPtr& msg) { points[6].x = msg->point.x;points[6].y = msg->point.y; points[6].z = msg->point.z;  }
    void callbackRadar7(const geometry_msgs::PointStamped::ConstPtr& msg) { points[7].x = msg->point.x;points[7].y = msg->point.y; points[7].z = msg->point.z;  }
    void callbackRadar8(const geometry_msgs::PointStamped::ConstPtr& msg) { points[8].x = msg->point.x;points[8].y = msg->point.y; points[8].z = msg->point.z;  }
    void callbackRadar9(const geometry_msgs::PointStamped::ConstPtr& msg) { points[9].x = msg->point.x;points[9].y = msg->point.y; points[9].z = msg->point.z;  }
    void callbackRadar10(const geometry_msgs::PointStamped::ConstPtr& msg) { points[10].x = msg->point.x;points[10].y = msg->point.y; points[10].z = msg->point.z;  }
    void callbackRadar11(const geometry_msgs::PointStamped::ConstPtr& msg) { points[11].x = msg->point.x;points[11].y = msg->point.y; points[11].z = msg->point.z;  }
    void callbackRadar12(const geometry_msgs::PointStamped::ConstPtr& msg) { points[12].x = msg->point.x;points[12].y = msg->point.y; points[12].z = msg->point.z;  }
    void callbackRadar13(const geometry_msgs::PointStamped::ConstPtr& msg) { points[13].x = msg->point.x;points[13].y = msg->point.y; points[13].z = msg->point.z;  }
    void callbackRadar14(const geometry_msgs::PointStamped::ConstPtr& msg) { points[14].x = msg->point.x;points[14].y = msg->point.y; points[14].z = msg->point.z;  }
    void callbackRadar15(const geometry_msgs::PointStamped::ConstPtr& msg) { points[15].x = msg->point.x;points[15].y = msg->point.y; points[15].z = msg->point.z;  }
};

void lightModelFs(const FragmentInfo& fInfo) {
    Coordinates3D* colorRGB = (Coordinates3D*)fInfo.data;
    setRGB(fInfo.pixel, *colorRGB);
    
//    Coordinates3D clippedRGB = clipRGB(*colorRGB);
//    fInfo.colorOutput->r = clippedRGB.x*255;
//    fInfo.colorOutput->g = clippedRGB.y*255;
//    fInfo.colorOutput->b = clippedRGB.z*255;
//    fInfo.colorOutput->a = 0;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "radar2ascii", ros::init_options::AnonymousName);
	ROS_INFO("radar2ascii is running...");

	ros::NodeHandle nh;
    
    setupTerminal();
	
    RadarListener mRadarListener(&nh);
    SteeringListener mSteeringListener(&nh);
    
    
    Coordinates4D cube[] = {
        {-1, -1, -1, 1},
        {-1, -1,  1, 1},
        {-1,  1,  1, 1},
        {-1,  1, -1, 1},
        { 1, -1, -1, 1},
        { 1, -1,  1, 1},
        { 1,  1,  1, 1},
        { 1,  1, -1, 1}
    };
    int edgeIndices[12][2] = {
        {0, 1},
        {1, 2},
        {2, 3},
        {3, 0},
        {0, 4},
        {1, 5},
        {2, 6},
        {3, 7},
        {4, 5},
        {5, 6},
        {6, 7},
        {7, 4}
    };
    
    
    int cubeQuadIndices[][4] = {
        {0, 1, 2, 3},    // left
        {7, 6, 5, 4},    // right
        {6, 2, 1, 5},     // top
        {0, 3, 7, 4},     // bottom
        {2, 6, 7, 3},     // front
        {0, 4, 5, 1}     // back
    };
    
#define NUM_GRID_LINES (11)
#define GRID_LINE_UNIT (2)
    Coordinates4D grid[NUM_GRID_LINES*2*2];    // 5x5 grid
    int gridEdgeIndices[NUM_GRID_LINES*2][2];
    for (int i = 0; i < NUM_GRID_LINES*2*2; i++) {
        
        grid[i].z = 0;
        grid[i].w = 1;
    }
    for (int i = 0; i < NUM_GRID_LINES; i++) {
        grid[i].x = -(NUM_GRID_LINES-1)/2 + i;
        grid[i].y = -(NUM_GRID_LINES-1)/2;
        
        grid[i+NUM_GRID_LINES].x = -(NUM_GRID_LINES-1)/2 + i;
        grid[i+NUM_GRID_LINES].y = (NUM_GRID_LINES-1)/2;
        
        grid[i+NUM_GRID_LINES*2].x = -(NUM_GRID_LINES-1)/2;
        grid[i+NUM_GRID_LINES*2].y = -(NUM_GRID_LINES-1)/2 + i;
        
        grid[i+NUM_GRID_LINES*3].x = (NUM_GRID_LINES-1)/2;
        grid[i+NUM_GRID_LINES*3].y = -(NUM_GRID_LINES-1)/2 + i;
        
        gridEdgeIndices[i][0] = i;
        gridEdgeIndices[i][1] = i+NUM_GRID_LINES;
        
        gridEdgeIndices[i+NUM_GRID_LINES][0] = i+NUM_GRID_LINES*2;
        gridEdgeIndices[i+NUM_GRID_LINES][1] = i+NUM_GRID_LINES*3;
    }
    
    for (int i = 0; i < NUM_GRID_LINES*2*2; i++) {
        grid[i].x *= GRID_LINE_UNIT;
        grid[i].y *= GRID_LINE_UNIT;
    }
    
    double characterAspect = 28.0/12.0; // macOs terminal
//    double characterAspect = 28.0/14.0; // raspbian terminal
    //    double characterAspect = 6.0/4.0; // zipitZ2
    
    int screenSizeX, screenSizeY;
    getmaxyx(stdscr, screenSizeY, screenSizeX);
#ifdef FB_SUPPORT    // HACK
    characterAspect = 1.0;    // RGB panel
#define PANEL_X_RES (64)
#define PANEL_Y_RES (64)
    screenSizeY = PANEL_Y_RES;
    screenSizeX = PANEL_X_RES;
#endif
    
    // Depth buffer (old method)
    DepthBuffer depthBuffer;
    depthBuffer.setSize(screenSizeX, screenSizeY);
	
    double screenAspect = (double)screenSizeX/(double)screenSizeY / characterAspect;
    
    
    
    // Model
    Mat4D scaleMat = scaleMatrix(1, 1, 1);
    
    // View
    Mat4D cameraTranslation = translationMatrix(0, 0, -5);
    Coordinates3D cameraAxis = {0, 1, 0};
    cameraAxis = normalizeVector(cameraAxis);
    Mat4D cameraOrientation = rotationFromAngleAndUnitAxis(-M_PI_4, cameraAxis);
    Mat4D viewMatrix = matrixMultiply( cameraOrientation, cameraTranslation );
    
    // Projection
    double zFar = 100;
    double zNear = 0.1;
    double viewAngle = M_PI*0.5;
    Mat4D projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
    
    // Viewport
//    Mat4D windowScale = scaleMatrix((double)screenSizeX/2, (double)screenSizeY/2, 1);
//    Mat4D translationScreen = translationMatrix((double)screenSizeX/2 -0.5, (double)screenSizeY/2 -0.5, 0);
//    Mat4D windowFull = matrixMultiply(translationScreen, windowScale);
    Mat4D windowFull = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);
    
    int numEdges;
    int debugLine = 0;
    double lightAngle = 0;
    double angle = 0;
    double cube2angle = 0;
    double tilt = M_PI/4;
    bool usePerspective = true;
    bool showGrid = true;
    bool autoRotate = true;
    bool showDepth = false;
    
    ros::Rate rate(60);
    
    while(keepRunning && ros::ok()) {
//        usleep(100000);
        rate.sleep();
        debugLine = 0;
        
        depthBuffer.reset();
        erase();
        
        mvprintw(debugLine++, 0, "Hello darkness...");
        mvprintw(debugLine++, 0, "Press ESC to quit");
        
        
        /*
         Build the view/camera matrix:
         */
        cameraAxis.x = 0;
        cameraAxis.y = 0;
        cameraAxis.z = 1;
        Mat4D camPan = rotationFromAngleAndUnitAxis(angle, cameraAxis);
        camPan = transpose(camPan);
        

//        viewMatrix = matrixMultiply( cameraOrientation, cameraTranslation);
        
        cameraAxis.x = 1;
        cameraAxis.y = 0;
        cameraAxis.z = 0;
        Mat4D camTilt = rotationFromAngleAndUnitAxis(tilt, cameraAxis);
        camTilt = transpose(camTilt);
        
        cameraOrientation = matrixMultiply(camTilt, camPan);
        
        if(autoRotate){
            double distance = 2*sin(angle/2);
            cameraTranslation = translationMatrix(-(5+distance)*sin(angle), (5+distance) * cos(angle), -(2*cos(angle/5)+distance)-4);
        }
        viewMatrix = matrixMultiply( cameraOrientation, cameraTranslation);
        
        
        int numObjects = 16;
        double carTranslationLongitude = -8;
        double carScale = 0.2;
        for( int i = 0; i < numObjects; i++) {
            Mat4D scale = scaleMatrix(0.15, 0.15, 0.15);
//            Mat4D translation = translationMatrix(30 + 40*sin(lightAngle*(i+1)/(double)numObjects), i-numObjects/2, 1);
            // Notice I swapped x and y here:
            Mat4D translation = translationMatrix(mRadarListener.points[i].y*carScale , mRadarListener.points[i].x*carScale+ carTranslationLongitude, 1.7272*carScale/2);
            Mat4D model = matrixMultiply(translation, scale);
            Mat4D objectModelView = matrixMultiply(viewMatrix, model);
            numEdges = sizeof(cubeQuadIndices)/sizeof(cubeQuadIndices[0]);
            
            Coordinates3D color = {0,0,1};
            
            rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, objectModelView, projection, windowFull, (void*)&color, &depthBuffer, lightModelFs, debugLine);
            
        }
        
        // Grid:
        if (showGrid) {
            Coordinates4D grid2[sizeof(grid)/sizeof(grid[0])];
            for (int i = 0; i < sizeof(grid2)/sizeof(grid2[0]); i++) {
                grid2[i] = grid[i];
                // Model
                
                // View
                grid2[i] = matrixVectorMultiply(viewMatrix, grid2[i]);

                // Projection
                grid2[i] = matrixVectorMultiply(projection, grid2[i]);
            }
            numEdges = sizeof(gridEdgeIndices)/sizeof(gridEdgeIndices[0]);
            
            rasterize(grid2, gridEdgeIndices, numEdges, windowFull,  &depthBuffer);
        }
        
        // Car:
        Coordinates3D carColor = {0,1,1};
        Mat4D carScaleMatrix = scaleMatrix(1.8542*carScale/2, 4.6228*carScale/2, 1.7272*carScale/2);
        Mat4D carTranslation = translationMatrix(0, carTranslationLongitude, 1.7272*carScale/2 );
        Mat4D carModel = matrixMultiply(carTranslation, carScaleMatrix);
        Mat4D carModelView = matrixMultiply(viewMatrix, carModel);
        numEdges = sizeof(cubeQuadIndices)/sizeof(cubeQuadIndices[0]);
        
        rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, carModelView, projection, windowFull, (void*)&carColor, &depthBuffer, lightModelFs, debugLine);
        
        
//        for (int i = 15; i >= 0; i--) {
//            mvaddch(mRadarListener.points[0].x + 10 + i, mRadarListener.points[0].y + 10, '0' + (char)i);
//        }
        
        
        Coordinates2D steeringCenter;
//        double steeringWidth = 15;
//        double steeringHeight = steeringWidth / characterAspect;
        double steeringHeight;
        double steeringWidth;
        if(screenSizeY / screenAspect > screenSizeX) {
            steeringWidth = 0.125 * screenSizeX;
            steeringHeight = steeringWidth / characterAspect;
        } else {
            steeringHeight = 0.125 * screenSizeY;
            steeringWidth = steeringHeight * characterAspect;
        }
        
        steeringCenter.x = screenSizeX - steeringWidth - 1;
        steeringCenter.y = steeringHeight + 1.5;  // +1.5 to allow for text
        mSteeringListener.draw(steeringCenter, steeringWidth, steeringHeight);
        
        if (autoRotate) {
            angle -= 0.01;
        }
        
        int ch;
        if ((ch = getch()) == 0x1B) {    // Escape
            keepRunning = false;
        } else if (ch == KEY_RESIZE) {
#ifdef FB_SUPPORT    // HACK
            //screenSizeY = 64;
            //screenSizeX = 64;
#else
            getmaxyx(stdscr, screenSizeY, screenSizeX);
#endif
            
//            mRenderPipeline.resize(screenSizeX, screenSizeY);
            
            screenAspect = (double)screenSizeX/(double)screenSizeY / characterAspect;
            windowFull = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);
            
            if (usePerspective) {
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                projection = projectionMatrixOrtho(5*screenAspect, 5, zFar, zNear);
            }
//            depthBuffer.setSize(screenSizeX, screenSizeY);
            depthBuffer.setSize(screenSizeX, screenSizeY);
        } else if (ch == 'o' || ch == 'O') {
            usePerspective    = !usePerspective;
            if (usePerspective) {
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                projection = projectionMatrixOrtho(5*screenAspect, 5, zFar, zNear);
            }
        } else if ( ch == 'g' || ch == 'G') {
            showGrid = !showGrid;
        } else if ( ch == KEY_LEFT) {
            angle += 0.05;
        } else if ( ch == KEY_RIGHT) {
            angle -= 0.05;
        } else if ( ch == KEY_UP) {
            tilt += 0.05;
        } else if ( ch == KEY_DOWN) {
            tilt -= 0.05;
        } else if ( ch == 'a' || ch == 'A') {
            // Left
            Mat4D motion = translationMatrix(0.25, 0, 0);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 'd' || ch == 'D') {
             // Right
            Mat4D motion = translationMatrix(-0.25, 0, 0);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
           
        } else if ( ch == 'w' || ch == 'W') {
            // Forward
            Mat4D motion = translationMatrix(0, 0, 0.25);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 's' || ch == 'S') {
            // Back
            Mat4D motion = translationMatrix(0, 0, -0.25);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 'r' || ch == 'R') {
            if (usePerspective) {
                viewAngle += M_PI * 0.0125;
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            }
        } else if ( ch == 't' || ch == 'T') {
            if (usePerspective) {
                viewAngle -= M_PI * 0.0125;
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            }
        } else if ( ch == ' ' ) {
            autoRotate = !autoRotate;
        } else if ( ch == 'd' || ch == 'D' ) {
            showDepth = !showDepth;
        }
        
        ros::spinOnce();
    }

    cleanupConsole();
    return 0;
}
