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
#include <fstream>

#include <unistd.h>
#include <chrono>

#include <ncurses.h>
#include <termios.h>

#include <curses-gfx.h>
#include <curses-gfx-3d.h>
#include <curses-gfx-handler.h>
#include <curses-gfx-texture.h>
#include <curses-gfx-loader.h>
#include <curses-gfx-resources.h>

// ROS headers:
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"   // Just for Tf functions
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "geodesy/utm.h"

// project includes
#include "osm-loader.h"
#include "resources.h"


#define RAV4_WIDTH (1.8542)
#define RAV4_LENGTH (4.6228)
#define RAV4_HEIGHT (1.7272)

//std::string readFileContents(const char* filename) {
//    std::ifstream ifs(filename);
//    std::string result;
//    ifs >> result;
//    return result;
//}

REGISTER_VERTEX_LAYOUT(MeshVertexInfo)
    MEMBER(location),
    MEMBER(normal)
//    MEMBER(textureCoord)
//    MEMBER(color)
END_VERTEX_LAYOUT(MeshVertexInfo)

typedef struct _UniformInfo {
    Mat4D modelView;
    Mat4D modelViewProjection;
    Mat4D normalMatrix;
} UniformInfo;

template <class T, class U> void myVertexShader(U* uniformInfo, T& output, const T& input) {
    output.vertex = matrixVectorMultiply(uniformInfo->modelViewProjection, input.vertex);
//    output.location = matrixVectorMultiply(uniformInfo->modelView, input.vertex);
//    output.normal = matrixVectorMultiply(uniformInfo->modelView, input.normal);
    output.textureCoord = input.textureCoord;
}

template <class T, class U> void covarianceVertexShader(U* uniformInfo, T& output, const T& input) {
    output.vertex = matrixVectorMultiply(uniformInfo->modelViewProjection, input.vertex);
    output.location = matrixVectorMultiply(uniformInfo->modelView, input.vertex);
    output.normal = matrixVectorMultiply(uniformInfo->normalMatrix, input.normal);
//    output.textureCoord = input.textureCoord;
}

template <class T, class U> void boxVertexShader(U* uniformInfo, T& output, const T& input) {
    output.vertex = matrixVectorMultiply(uniformInfo->modelViewProjection, input.vertex);
    output.location = matrixVectorMultiply(uniformInfo->modelView, input.vertex);
    output.normal = matrixVectorMultiply(uniformInfo->normalMatrix, input.normal);
//    output.textureCoord = input.textureCoord;
}

template <class T, class U> void genericCubeVertexShader(U* uniformInfo, T& output, const T& input) {
    output.vertex = matrixVectorMultiply(uniformInfo->modelViewProjection, input.vertex);
    output.location = matrixVectorMultiply(uniformInfo->modelView, input.vertex);
    output.normal = matrixVectorMultiply(uniformInfo->normalMatrix, input.normal);
}

typedef struct _MapVertexInfo {
    Coordinates4D vertex;
    Coordinates2Df textureCoord;
} MapVertexInfo;

REGISTER_VERTEX_LAYOUT(MapVertexInfo)
    MEMBER(textureCoord)
END_VERTEX_LAYOUT(MapVertexInfo)

void mapShader(const FragmentInfo2& fInfo) {
    MapVertexInfo* vertexInfo = (MapVertexInfo*)fInfo.interpolated;

    Texture* mapTexture = (Texture*) fInfo.data;
    double factor =  1.0;///(vertexInfo->vertex.z +2);//(Q_rsqrt(vertexInfo->vertex.z*100 + 2);
    *fInfo.colorOutput = mapTexture->sample(vertexInfo->textureCoord.x, vertexInfo->textureCoord.y);
    *fInfo.colorOutput = *fInfo.colorOutput * factor;
    fInfo.colorOutput->a = 0;
}

void lightModelFs(const FragmentInfo2& fInfo) {
    Coordinates3D* colorRGB = (Coordinates3D*)fInfo.data;
//    setRGB(fInfo.pixel, *colorRGB);
    
    Coordinates3D clippedRGB = clipRGB(*colorRGB);
    fInfo.colorOutput->r = clippedRGB.x*255;
    fInfo.colorOutput->g = clippedRGB.y*255;
    fInfo.colorOutput->b = clippedRGB.z*255;
    fInfo.colorOutput->a = 0;
}

void covarianceFs(const FragmentInfo2& fInfo) {
    Coordinates3D* colorRGB = (Coordinates3D*)fInfo.data;
    MeshVertexInfo* vertexInfo = (MeshVertexInfo*)fInfo.interpolated;
//    setRGB(fInfo.pixel, *colorRGB);
    
    Coordinates3D normal = normalizeVectorFast(vertexInfo->normal);
    
    Coordinates3D viewDir = normalizeVectorFast(vertexInfo->location);
    
    double magnitude = fabs(dotProduct(viewDir, normal));// fabs(normal.z);
//    magnitude *= magnitude;
//    Coordinates3D clippedRGB = clipRGB(*colorRGB);
    fInfo.colorOutput->r = ((double)fInfo.colorOutput->r)*0.8 + colorRGB->x*255.0*0.2* magnitude;
    fInfo.colorOutput->g = ((double)fInfo.colorOutput->g)*0.8 + colorRGB->y*255.0*0.2* magnitude;
    fInfo.colorOutput->b = ((double)fInfo.colorOutput->b)*0.8 + colorRGB->z*255.0*0.2* magnitude;
    fInfo.colorOutput->a = 0;
}

void boxFs(const FragmentInfo2& fInfo) {
    Coordinates3D* colorRGB = (Coordinates3D*)fInfo.data;
    MeshVertexInfo* vertexInfo = (MeshVertexInfo*)fInfo.interpolated;
//    setRGB(fInfo.pixel, *colorRGB);
    
    Coordinates3D normal = normalizeVectorFast(vertexInfo->normal);
    
    Coordinates3D viewDir = normalizeVectorFast(vertexInfo->location);
    
    double magnitude = fabs(dotProduct(viewDir, normal));// fabs(normal.z);
//    magnitude *= magnitude;
//    Coordinates3D clippedRGB = clipRGB(*colorRGB);
    fInfo.colorOutput->r = colorRGB->x*255.0 * magnitude;
    fInfo.colorOutput->g = colorRGB->y*255.0 * magnitude;
    fInfo.colorOutput->b = colorRGB->z*255.0 * magnitude;
    fInfo.colorOutput->a = 0;
}

void genericNormalFs(const FragmentInfo2& fInfo) {
    Coordinates3D* colorRGB = (Coordinates3D*)fInfo.data;
    MeshVertexInfo* vertexInfo = (MeshVertexInfo*)fInfo.interpolated;
//    setRGB(fInfo.pixel, *colorRGB);
    
    Coordinates3D normal = normalizeVectorFast(vertexInfo->normal);
    
    Coordinates3D viewDir = normalizeVectorFast(vertexInfo->location);
    
    double magnitude = fabs(dotProduct(viewDir, normal));// fabs(normal.z);
//    magnitude *= magnitude;
//    Coordinates3D clippedRGB = clipRGB(*colorRGB);
    fInfo.colorOutput->r = ((double)fInfo.colorOutput->r)*0.8 + colorRGB->x*255.0*0.2* magnitude;
    fInfo.colorOutput->g = ((double)fInfo.colorOutput->g)*0.8 + colorRGB->y*255.0*0.2* magnitude;
    fInfo.colorOutput->b = ((double)fInfo.colorOutput->b)*0.8 + colorRGB->z*255.0*0.2* magnitude;
    fInfo.colorOutput->a = 0;
}

/*
 Catch ctrl-c for cleaner exits
 */
static volatile bool keepRunning = true;
void killPanda(int killSignal) {
    keepRunning = false;
}

class ImuListener {
private:
    ros::Subscriber subscriber;
    
    ros::Subscriber subscriberMagnetic;
    
public:
    
    Mat4D orientation;
    Coordinates3D acceleration;
    Coordinates3D rotationRate;
    double q[4];
    
    sensor_msgs::MagneticField magnetic;
    
    void callback(const sensor_msgs::Imu::ConstPtr& msg) {
//        tf::Quaternion myQuaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
//        tf::Matrix3x3 mat(myQuaternion);
//        tf::Matrix3x3 mat(msg->orientation);
//        mat.getEulerYPR(yaw, pitch, roll);
        q[0] = msg->orientation.w;
        q[1] = msg->orientation.x;
        q[2] = msg->orientation.y;
        q[3] = msg->orientation.z;
        
        
        double q00 = msg->orientation.w*msg->orientation.w;
        double q11 = msg->orientation.x*msg->orientation.x;
        double q22 = msg->orientation.y*msg->orientation.y;
        double q33 = msg->orientation.z*msg->orientation.z;
        
        double q01 = msg->orientation.w*msg->orientation.x;
        double q02 = msg->orientation.w*msg->orientation.y;
        double q03 = msg->orientation.w*msg->orientation.z;
        
        double q12 = msg->orientation.x*msg->orientation.y;
        double q13 = msg->orientation.x*msg->orientation.z;
        
        double q23 = msg->orientation.y*msg->orientation.z;
        
        orientation.d[0][0] = 2*(q00 + q11) - 1;
        orientation.d[0][1] = 2*(q12 - q03);
        orientation.d[0][2] = 2*(q13 + q02);
        
        orientation.d[1][0] = 2*(q12 + q03);
        orientation.d[1][1] = 2*(q00 + q22) - 1;
        orientation.d[1][2] = 2*(q23 - q01);
        
        orientation.d[2][0] = 2*(q13 - q02);
        orientation.d[2][1] = 2*(q23 + q01);
        orientation.d[2][2] = 2*(q00 + q33) - 1;
        
        
        rotationRate.x = msg->angular_velocity.x;
        rotationRate.y = msg->angular_velocity.y;
        rotationRate.z = msg->angular_velocity.z;
        
        acceleration.x = msg->linear_acceleration.x;
        acceleration.y = msg->linear_acceleration.y;
        acceleration.z = msg->linear_acceleration.z;
        
    }
    void callbackMagnetic(const sensor_msgs::MagneticField::ConstPtr& msg) {
        magnetic = *msg;
        
        
    }
    void printMagnetic(int x, int y) {
        int i = 0;
        mvprintw(y+i++, x, "IMU:   Mag   Gyro  Accel");
        mvprintw(y+i++, x, "  x: % 0.02f  % 0.02f  % 0.02f", magnetic.magnetic_field.x/0.00005, // constant to convert Tesla to "EFU"
                 rotationRate.x,
                 acceleration.x);
        mvprintw(y+i++, x, "  y: % 0.02f  % 0.02f  % 0.02f", magnetic.magnetic_field.y/0.00005,
                 rotationRate.y,
                 acceleration.y);
        mvprintw(y+i++, x, "  z: % 0.02f  % 0.02f  % 0.02f", magnetic.magnetic_field.z/0.00005,
                 rotationRate.z,
                 acceleration.z);
        
        mvprintw(y+i++, x, " L2: % 0.02f  % 0.02f  % 0.02f", sqrt(magnetic.magnetic_field.x*magnetic.magnetic_field.x + magnetic.magnetic_field.y*magnetic.magnetic_field.y + magnetic.magnetic_field.z*magnetic.magnetic_field.z)/0.00005, // constant to convert Tesla to "EFU"
                 sqrt(rotationRate.x*rotationRate.x + rotationRate.y*rotationRate.y + rotationRate.z*rotationRate.z),
                 sqrt(acceleration.x*acceleration.x + acceleration.y*acceleration.y + acceleration.z*acceleration.z));
    }
    ImuListener(ros::NodeHandle* nh) {
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
        acceleration.x = 0;
        acceleration.y = 0;
        acceleration.z = 0;
        rotationRate.x = 0;
        rotationRate.y = 0;
        rotationRate.z = 0;
        magnetic.magnetic_field.x = 0;
        magnetic.magnetic_field.y = 0;
        magnetic.magnetic_field.z = 0;
        subscriber = nh->subscribe("/microstrain/imu", 1000, &ImuListener::callback, this);
        subscriberMagnetic = nh->subscribe("/microstrain/mag", 1000, &ImuListener::callbackMagnetic, this);
        acceleration.z = 1;
        rotationRate.z = 1;
        orientation = translationMatrix(0, 0, 0);
        
    }
};


class GpsListener {
private:
    ros::NodeHandle* nodeHandle;
    ros::Subscriber subscriberGps;
    ros::Subscriber subscriberGpsHeading;
    ros::Subscriber subscriberCarSpeed;
    ros::Subscriber subscriberOdom;
    ros::Subscriber subscriberOdomFilteredLocal;
    ros::Subscriber subscriberOdomFilteredGlobal;
    double priorLong;
    double priorLat;
    
    // tf stuff
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    
public:
    
    double longitudeRaw;
    double latitudeRaw;
    
    double longitude;
    double latitude;
    double diffLong, diffLat;
    
    double headingFromImu;
    
    double heading; // calculated form lat/long
    bool first;
    bool headingFromGpsFix;
    
    double speed;
    
    double carX, carY, carTheta;
    double odomFilteredGlobalX, odomFilteredGlobalY, odomFilteredGlobalTheta;
    double odomFilteredLocalX, odomFilteredLocalY, odomFilteredLocalTheta;
    
    double x, y, theta; // final output
    double covX, covY;
    Mat4D covarianceGps;
    
    typedef enum {
        LOCALIZATION_GPS,
        LOCALIZATION_CAR_ODOM,
        LOCALIZATION_FILTERED_LOCAL,
        LOCALIZATION_FILTERED_GLOBAL,
        LOCALIZATION_TF_MAP_TO_BASE_LINK,
        LOCALIZATION_TF_ODOM_TO_BASE_LINK,
        LOCALIZATION_TF_UTM_TO_BASE_LINK,
        LOCALIZATION_COUNT
    } LocalizationMethod;
    
    LocalizationMethod localizationMethodToShow;
    
    GpsListener(ros::NodeHandle* nh) {
        longitude = latitude = 0;
        diffLong = diffLat = 0;
        longitudeRaw = latitudeRaw = 0;
        
        headingFromImu = 0;
        
        heading = 0; // calculated form lat/long
        first = true;
        headingFromGpsFix = true;
        
        speed = 0;
        
        carX = carY = carTheta = 0;
        odomFilteredGlobalX = odomFilteredGlobalY = odomFilteredGlobalTheta = 0;
        odomFilteredLocalX = odomFilteredLocalY = odomFilteredLocalTheta = 0;
        
        x = y = theta = 0; // final output
        covX = covY = 0;
        covarianceGps = scaleMatrix(1, 1, 1);
        
        localizationMethodToShow = LOCALIZATION_GPS;

        nodeHandle = nh;
        subscriberGps = nodeHandle->subscribe("/car/gps/fix", 1000, &GpsListener::callbackGps, this);
        subscriberGpsHeading = nodeHandle->subscribe("/car/gps/heading", 1000, &GpsListener::callbackGpsHeading, this);
        subscriberCarSpeed = nodeHandle->subscribe("/car/state/vel_x", 1000, &GpsListener::callbackCarSpeed, this);
        
        subscriberOdom = nodeHandle->subscribe("/car/odometry/differential_drive", 1000, &GpsListener::callbackCarOdom, this);
        subscriberOdomFilteredLocal  = nodeHandle->subscribe("/odometry/filtered/local", 1000, &GpsListener::callbackOdomFilteredLocal, this);
        subscriberOdomFilteredGlobal = nodeHandle->subscribe("/odometry/filtered/global", 1000, &GpsListener::callbackOdomFilteredGlobal, this);
        
        tfListener = new tf2_ros::TransformListener(tfBuffer);
    }
    
    const char* localizationMethodToString( LocalizationMethod method ) {
        switch (method) {
            case LOCALIZATION_GPS: return "GPS";
            case LOCALIZATION_CAR_ODOM: return "Car Odometry";
            case LOCALIZATION_FILTERED_LOCAL: return "Kalman Odometry";
            case LOCALIZATION_FILTERED_GLOBAL: return "Kalman Global";
            case LOCALIZATION_TF_ODOM_TO_BASE_LINK: return "tf odom->base_link";
            case LOCALIZATION_TF_MAP_TO_BASE_LINK: return "tf map->base_link";
            case LOCALIZATION_TF_UTM_TO_BASE_LINK: return "tf utm->base_link";
//            default:
//                break;
        }
        return "";
    }

    void setLocalizationMethod(LocalizationMethod method) {
        localizationMethodToShow = method;
    }
    
    void updateTf() {
        geometry_msgs::TransformStamped transformStamped;
        try {
            if(localizationMethodToShow == LOCALIZATION_TF_MAP_TO_BASE_LINK) {
                transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            } else if(localizationMethodToShow == LOCALIZATION_TF_UTM_TO_BASE_LINK) {
                transformStamped = tfBuffer.lookupTransform("utm", "base_link", ros::Time(0));
            } else if(localizationMethodToShow == LOCALIZATION_TF_ODOM_TO_BASE_LINK) {
                transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
            }
        } catch (tf2::TransformException &ex) {
            return;
        }
        
        if((localizationMethodToShow == LOCALIZATION_TF_MAP_TO_BASE_LINK) ||
           (localizationMethodToShow == LOCALIZATION_TF_UTM_TO_BASE_LINK) ||
           (localizationMethodToShow == LOCALIZATION_TF_ODOM_TO_BASE_LINK)) {
            x = transformStamped.transform.translation.x;
            y = transformStamped.transform.translation.y;
            
            tfScalar yaw, pitch, roll;
            tf::Quaternion myQuaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf::Matrix3x3 mat(myQuaternion);
            mat.getEulerYPR(yaw, pitch, roll);
            
            theta = yaw;
        }
    }
    
    void callbackGpsHeading(const std_msgs::Float64::ConstPtr& msg) {
//        heading = (msg->data+180-4.04) * M_PI/180.0;
//        heading = -(msg->data-90) * M_PI/180.0 + (speed < 0 ? M_PI : 0);
        heading = -(msg->data-90) * M_PI/180.0 + (speed < 0 ? M_PI : 0);
	headingFromGpsFix = false;
    }
    
    void callbackCarSpeed(const std_msgs::Float64::ConstPtr& msg) {
        speed = msg->data;
    }
    
    void callbackGps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // helpful on how to use conversions: https://github.com/ros-geographic-info/geographic_info/blob/master/geodesy/tests/test_utm.cpp
        geographic_msgs::GeoPoint fromLL;
        fromLL.latitude = msg->latitude;
        fromLL.longitude = msg->longitude;
        fromLL.altitude = msg->altitude;
        geodesy::UTMPoint gpsAsUtm(fromLL);
        latitudeRaw = msg->latitude;
        longitudeRaw = msg->longitude;
        if(first) {
            first = false;
            latitude = gpsAsUtm.northing;
            longitude = gpsAsUtm.easting;
            priorLong = longitude;
            priorLat = latitude;
            heading = 0;
            return;
        }
        
        
        if(headingFromGpsFix){
	        longitude = 0.85*longitude + 0.15*gpsAsUtm.easting;
        	latitude = 0.85*latitude + 0.15*gpsAsUtm.northing;
	}
	else{
	        longitude = gpsAsUtm.easting;
        	latitude = gpsAsUtm.northing;
	}
        
        diffLong = (longitude - priorLong);// * METERS_PER_DEGREE;
        diffLat = (latitude - priorLat);// * METERS_PER_DEGREE;
        if( (diffLong*diffLong + diffLat*diffLat) > 0.75) {
//            heading = -atan2(diffLong, diffLat);
            if(headingFromGpsFix){
	            heading = atan2(diffLat,diffLong);// + 3.1415926535897/180.0/2.0;
		}
            priorLong = longitude;
            priorLat = latitude;
            diffLong = 0;
            diffLat = 0;
        }
        
        
        
        covX = msg->position_covariance[0];
        covY = msg->position_covariance[4];
        
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                covarianceGps.d[i][j] = msg->position_covariance[i + j*3];
            }
        }
        
        if(localizationMethodToShow == LOCALIZATION_GPS) {
            
            
//            x = latitude * METERS_PER_DEGREE;
//            y = longitude * METERS_PER_DEGREE;
            x = gpsAsUtm.easting;
            y = gpsAsUtm.northing;
//            theta = -heading + 3.1415926535897/180.0/2.0;
            theta = heading;
//            theta = headingFromImu; // HACK
        }
        
    }
    
    void callbackCarOdom(const nav_msgs::Odometry::ConstPtr& msg) {
            
        carX = msg->pose.pose.position.x;
        carY = msg->pose.pose.position.y;
        
        tfScalar yaw, pitch, roll;
        tf::Quaternion myQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 mat(myQuaternion);
        mat.getEulerYPR(yaw, pitch, roll);
        
        carTheta = yaw;
        
        if(localizationMethodToShow == LOCALIZATION_CAR_ODOM) {
            x = carX;
            y = carY;
            theta = carTheta;
        }
    }
    
    void callbackOdomFilteredGlobal(const nav_msgs::Odometry::ConstPtr& msg) {
        odomFilteredGlobalX = msg->pose.pose.position.x;
        odomFilteredGlobalY = msg->pose.pose.position.y;
        
        tfScalar yaw, pitch, roll;
        tf::Quaternion myQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 mat(myQuaternion);
        mat.getEulerYPR(yaw, pitch, roll);
        
        odomFilteredGlobalTheta = yaw;
        if(localizationMethodToShow == LOCALIZATION_FILTERED_GLOBAL) {
            x = odomFilteredGlobalX;
            y = odomFilteredGlobalY;
            theta = odomFilteredGlobalTheta;
        }
    }
    void callbackOdomFilteredLocal(const nav_msgs::Odometry::ConstPtr& msg) {
        odomFilteredLocalX = msg->pose.pose.position.x;
        odomFilteredLocalY = msg->pose.pose.position.y;
        
        tfScalar yaw, pitch, roll;
        tf::Quaternion myQuaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3 mat(myQuaternion);
        mat.getEulerYPR(yaw, pitch, roll);
        
        odomFilteredLocalTheta = yaw;
        if(localizationMethodToShow == LOCALIZATION_FILTERED_LOCAL) {
            x = odomFilteredLocalX;
            y = odomFilteredLocalY;
            theta = odomFilteredLocalTheta;
        }
    }
    
    void draw(int x, int y) {
        int i = 0;
        mvprintw(y+i++, x, "Current Method %s", localizationMethodToString(localizationMethodToShow));
        mvprintw(y+i++, x, "Longitude (x) %.06f", this->x );
        mvprintw(y+i++, x, "Latitude  (y) %.06f", this->y);
        mvprintw(y+i++, x, "Heading       %.02f", theta* 180/M_PI);
        mvprintw(y+i++, x, "GPS diff Long %.06f", longitude - this->x);
        mvprintw(y+i++, x, "GPS diff Lat  %.06f", latitude - this->y );
        mvprintw(y+i++, x, "GPS diff head %.02f", (heading-theta)  * 180/M_PI);
        mvprintw(y+i++, x, "Car Speed     %.01f mph", speed/0.44704);

        //mvprintw(y+i++, x, "diffMag   %.02f", (diffLong*diffLong + diffLat*diffLat));
        
        
    }
};


void drawGuage(double* angles, int angleCount, Coordinates2D center, double width, double height, const char* labels[]) {
    Coordinates2D point1 = center;
    Coordinates2D point2 = {.x = 25, .y=0};
    
    if(angleCount > 8) {
        exit(EXIT_FAILURE);
    }
    static Coordinates3D colors[8] = {
        {0, 1, 0},
        {1, 0, 0},
        {0, 0, 1},
        {1, 1, 0},
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 1},
        {0.5, 0.5, 0.5}
    };
    
    double level; // will be unused
    CursesGfxTerminal::enableThinAscii();
    CursesGfxTerminal::enableColor(colors[0], level);
    for (double i = 1; i <= 60; i+=1) {
        double x = center.x + ((double)width)*sin(i *M_PI*2.0/60.0 + angles[0]) + 0.5;
        double y = center.y - ((double)height)*cos(i *M_PI*2.0/60.0 + angles[0]) + 0.5;
        drawDotFloat(x,y);
    }
    CursesGfxTerminal::disableColor();
    CursesGfxTerminal::disableThinAscii();
        
    
    point1 = center;
    
    
    for (int a = 0; a < angleCount; a++) {
        CursesGfxTerminal::enableColor(colors[a], level);
        point2.x = point1.x + width*cos(angles[a]) + 0.5;
        point2.y = point1.y - height*sin(angles[a]) + 0.5;
        ln2(point1, point2);
        CursesGfxTerminal::disableColor();
    }
    
    int offset = 0;
    for (int a = 0; a < angleCount; a++) {
        CursesGfxTerminal::enableColor(colors[a], level);
        mvprintw(center.y - (height + 1), center.x - 20/2 + offset, labels[a]);
        offset += 1 + strlen(labels[a]);
        CursesGfxTerminal::disableColor();
    }
}

class SteeringListener {
private:
    ros::NodeHandle* nodeHandle;
    ros::Subscriber subscriber;
    
    double steeringAngle;
public:
    
    SteeringListener(ros::NodeHandle* nh) {
        nodeHandle = nh;
        subscriber = nodeHandle->subscribe("/car/state/steering", 1000, &SteeringListener::callbackSteering, this);
    }
    
    void callbackSteering(const std_msgs::Float64MultiArray::ConstPtr& msg) {
//        steeringAngle = -msg->data; // For whatever reason the steering angle reported in ROS is reversed?
        steeringAngle = -msg->data[0]; // For whatever reason the steering angle reported in ROS is reversed?
    }
    
    void draw(Coordinates2D center, double width, double height) {
        Coordinates2D point1 = center;
        Coordinates2D point2 = {.x = 25, .y=0};
        
        double level; // will be unused
        CursesGfxTerminal::enableThinAscii();
        CursesGfxTerminal::enableColor({1,0,1}, level);
        for (double i = 1; i <= 60; i+=1) {
            double x = center.x + ((double)width)*sin(i *M_PI*2.0/60.0 + steeringAngle*M_PI/180.0) + 0.5;
            double y = center.y - ((double)height)*cos(i *M_PI*2.0/60.0 + steeringAngle*M_PI/180.0) + 0.5;
            drawDotFloat(x,y);
        }
        CursesGfxTerminal::disableColor();
        CursesGfxTerminal::disableThinAscii();
        
        point1 = center;
        
        CursesGfxTerminal::enableColor({1,0,0}, level);
        point2.x = point1.x + width*sin(steeringAngle*M_PI/180.0) + 0.5;
        point2.y = point1.y - height*cos(steeringAngle*M_PI/180.0) + 0.5;
        ln2(point1, point2);
        CursesGfxTerminal::disableColor();
        
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
        
        subscriberATracks[ 0] = nodeHandle->subscribe("/car/radar/track_a0", 1000, &RadarListener::callbackRadar0, this);
        subscriberATracks[ 1] = nodeHandle->subscribe("/car/radar/track_a1", 1000, &RadarListener::callbackRadar1, this);
        subscriberATracks[ 2] = nodeHandle->subscribe("/car/radar/track_a2", 1000, &RadarListener::callbackRadar2, this);
        subscriberATracks[ 3] = nodeHandle->subscribe("/car/radar/track_a3", 1000, &RadarListener::callbackRadar3, this);
        subscriberATracks[ 4] = nodeHandle->subscribe("/car/radar/track_a4", 1000, &RadarListener::callbackRadar4, this);
        subscriberATracks[ 5] = nodeHandle->subscribe("/car/radar/track_a5", 1000, &RadarListener::callbackRadar5, this);
        subscriberATracks[ 6] = nodeHandle->subscribe("/car/radar/track_a6", 1000, &RadarListener::callbackRadar6, this);
        subscriberATracks[ 7] = nodeHandle->subscribe("/car/radar/track_a7", 1000, &RadarListener::callbackRadar7, this);
        subscriberATracks[ 8] = nodeHandle->subscribe("/car/radar/track_a8", 1000, &RadarListener::callbackRadar8, this);
        subscriberATracks[ 9] = nodeHandle->subscribe("/car/radar/track_a9", 1000, &RadarListener::callbackRadar9, this);
        subscriberATracks[10] = nodeHandle->subscribe("/car/radar/track_a10", 1000, &RadarListener::callbackRadar10, this);
        subscriberATracks[11] = nodeHandle->subscribe("/car/radar/track_a11", 1000, &RadarListener::callbackRadar11, this);
        subscriberATracks[12] = nodeHandle->subscribe("/car/radar/track_a12", 1000, &RadarListener::callbackRadar12, this);
        subscriberATracks[13] = nodeHandle->subscribe("/car/radar/track_a13", 1000, &RadarListener::callbackRadar13, this);
        subscriberATracks[15] = nodeHandle->subscribe("/car/radar/track_a14", 1000, &RadarListener::callbackRadar14, this);
        subscriberATracks[15] = nodeHandle->subscribe("/car/radar/track_a15", 1000, &RadarListener::callbackRadar15, this);
        
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


int main(int argc, char **argv) {
	ros::init(argc, argv, "radar2ascii", ros::init_options::AnonymousName);
	ROS_INFO("radar2ascii is running...");

	ros::NodeHandle nh;
    
//    Polygon4D unitCircle;
//    generateCirclePolygon(&unitCircle);
    
//    std::string homeDirectory = "/home/" + readFileContents("/etc/libpanda.d/libpanda_usr") + "/";
    Scene unitSphereScene;
//    if(unitSphereScene.load("/home/matt/curses-gfx/resources/unit-sphere-low-poly.dae")) {
    if(unitSphereScene.load( (std::string(CURSES_GFX_RESOURCE_PATH) + "unit-sphere-low-poly.dae").c_str()  ) ) {
        return -1;
    }
    
    Scene unitCubeScene;
    if(unitCubeScene.load( (std::string(CURSES_GFX_RESOURCE_PATH) + "unit-cube.stl").c_str()  ) ) {
        return -1;
    }
    
    UniformInfo mUniformInfo;
    
    CursesGfxTerminal mCursesGfxTerminal;
    mCursesGfxTerminal.setupTerminal();
    
    
    
    GpsListener::LocalizationMethod mLocalizationMethod = GpsListener::LOCALIZATION_FILTERED_GLOBAL;
	
    RadarListener mRadarListener(&nh);
    SteeringListener mSteeringListener(&nh);
    GpsListener mGpsListener(&nh);
    ImuListener mImuListener(&nh);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    
    
    
//    Coordinates4D cube[] = {
//        {-1, -1, -1, 1},
//        {-1, -1,  1, 1},
//        {-1,  1,  1, 1},
//        {-1,  1, -1, 1},
//        { 1, -1, -1, 1},
//        { 1, -1,  1, 1},
//        { 1,  1,  1, 1},
//        { 1,  1, -1, 1}
//    };
//    int edgeIndices[12][2] = {
//        {0, 1},
//        {1, 2},
//        {2, 3},
//        {3, 0},
//        {0, 4},
//        {1, 5},
//        {2, 6},
//        {3, 7},
//        {4, 5},
//        {5, 6},
//        {6, 7},
//        {7, 4}
//    };
//
//
//    int cubeQuadIndices[][4] = {
//        {0, 1, 2, 3},    // left
//        {7, 6, 5, 4},    // right
//        {6, 2, 1, 5},     // top
//        {0, 3, 7, 4},     // bottom
//        {2, 6, 7, 3},     // front
//        {0, 4, 5, 1}     // back
//    };
    
//#define NUM_GRID_LINES (15)
//#define GRID_LINE_UNIT (10)
//    Coordinates4D grid[NUM_GRID_LINES*2*2];    // 5x5 grid
//    int gridEdgeIndices[NUM_GRID_LINES*2][2];
//    for (int i = 0; i < NUM_GRID_LINES*2*2; i++) {
//
//        grid[i].z = 0;
//        grid[i].w = 1;
//    }
//    for (int i = 0; i < NUM_GRID_LINES; i++) {
//        grid[i].x = -(NUM_GRID_LINES-1)/2 + i;
//        grid[i].y = -(NUM_GRID_LINES-1)/2;
//
//        grid[i+NUM_GRID_LINES].x = -(NUM_GRID_LINES-1)/2 + i;
//        grid[i+NUM_GRID_LINES].y = (NUM_GRID_LINES-1)/2;
//
//        grid[i+NUM_GRID_LINES*2].x = -(NUM_GRID_LINES-1)/2;
//        grid[i+NUM_GRID_LINES*2].y = -(NUM_GRID_LINES-1)/2 + i;
//
//        grid[i+NUM_GRID_LINES*3].x = (NUM_GRID_LINES-1)/2;
//        grid[i+NUM_GRID_LINES*3].y = -(NUM_GRID_LINES-1)/2 + i;
//
//        gridEdgeIndices[i][0] = i;
//        gridEdgeIndices[i][1] = i+NUM_GRID_LINES;
//
//        gridEdgeIndices[i+NUM_GRID_LINES][0] = i+NUM_GRID_LINES*2;
//        gridEdgeIndices[i+NUM_GRID_LINES][1] = i+NUM_GRID_LINES*3;
//    }
//
//    for (int i = 0; i < NUM_GRID_LINES*2*2; i++) {
//        grid[i].x *= GRID_LINE_UNIT;
//        grid[i].y *= GRID_LINE_UNIT;
//    }
    
    double characterAspect = 28.0/12.0; // macOs terminal
//    double characterAspect = 28.0/14.0; // raspbian terminal
    //    double characterAspect = 6.0/4.0; // zipitZ2
    
    int screenSizeX, screenSizeY;
    getmaxyx(stdscr, screenSizeY, screenSizeX);

    RenderPipeline mRenderPipeline;
    mRenderPipeline.resize(screenSizeX, screenSizeY);
    
    // Depth buffer (old method)
//    DepthBuffer depthBuffer;
//    depthBuffer.setSize(screenSizeX, screenSizeY);
	
    Coordinates3D colorRed = {1, 0, 0};
    Coordinates3D colorGreen = {0, 1, 0};
    Coordinates3D colorBlue = {0, 0, 1};
    Coordinates3D colorRadar[16];// = colorBlue;
    Coordinates3D colorCar = {1,1,0};
    Coordinates3D colorCovariance = {0.5, 0, 1};
    
    double screenAspect = (double)screenSizeX/(double)screenSizeY / characterAspect;
    
    MapVertexInfo squareVi[4];
    int squareViIndices[2][3];
    squareVi[0].vertex = {0, -1, 0, 1}; // "top left corner" is the OSM standard
    squareVi[1].vertex = { 1, -1, 0, 1};
    squareVi[2].vertex = { 1,  0, 0, 1};
    squareVi[3].vertex = {0,  0, 0, 1};
    
    squareVi[0].textureCoord = {0, 1};
    squareVi[1].textureCoord = {1, 1};
    squareVi[2].textureCoord = {1, 0};
    squareVi[3].textureCoord = {0, 0};
    
    squareViIndices[0][0] = 0;  // right handed
    squareViIndices[0][1] = 1;
    squareViIndices[0][2] = 2;
    squareViIndices[1][0] = 0;  // left handed
    squareViIndices[1][1] = 2;
    squareViIndices[1][2] = 3;
    
    
//    OsmTile mOsmTile = latLongToTile(17, -86.7931535, 36.14507466666666);
//    char tileFile[200];
//    if(downloadTile(mOsmTile, "/home/matt", tileFile)) {
//        printf("Failed to download file!\n");
//    }
//    printf("File result: %s\n", tileFile);
//    Texture mapTexture(tileFile);
//    mapTexture.offsetAvergageToCenter(0.75);
//    mapTexture.normalize(1.);
//    mapTexture.invert();
    
    MapHandler mMapHandler;
//    double testIncrementor = 0;
//    double testLong = -86.7931535;
//    double testLat = 36.14507466666666;
//    mMapHandler.setLongLat(-86.7931535, 36.14507466666666);
    
    
    // Model
    Mat4D scaleMat = scaleMatrix(1, 1, 1);
    
    // View
    Mat4D cameraTranslation = translationMatrix(0, 0, -25);
    Coordinates3D cameraAxis = {0, 1, 0};
    cameraAxis = normalizeVector(cameraAxis);
    Mat4D cameraOrientation = rotationFromAngleAndUnitAxis(-M_PI_4, cameraAxis);
    Mat4D viewMatrix = matrixMultiply( cameraOrientation, cameraTranslation );
    
    // Projection
    double zFar = 1000;
    double zNear = 1;
    double viewAngle = M_PI*0.5;
    double orthoScale = 5*5;
    Mat4D projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
    
    // Viewport
//    Mat4D windowScale = scaleMatrix((double)screenSizeX/2, (double)screenSizeY/2, 1);
//    Mat4D translationScreen = translationMatrix((double)screenSizeX/2 -0.5, (double)screenSizeY/2 -0.5, 0);
//    Mat4D windowFull = matrixMultiply(translationScreen, windowScale);
//    Mat4D windowFull = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);
    mRenderPipeline.viewport = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);
//    windowFull.d[0][0] *= -1;   // HACK the car is aligned with GPS, heading, and radar but mirrores over x.  This mirrors the viewport
    
    int zoom = 18; // for the MapHandler
    int numEdges;
    int debugLine = 0;
    double lightAngle = 0;
    double angle = -M_PI/2; // default points
    double cube2angle = 0;
    double tilt = M_PI/4;
    bool usePerspective = true;
    bool showGrid = true;
    bool autoRotate = false;
    bool showDepth = false;
    bool showUnitVectors = false;
    
    ros::Rate rate(20);
    
    // Performance metrics:
    auto now = std::chrono::high_resolution_clock::now();
    auto before = now;
    std::chrono::duration<double, std::milli> float_ms = now - before;
    while(keepRunning && ros::ok()) {
        rate.sleep();
        debugLine = 0;
        
        mRenderPipeline.reset();
        
        
        
        mGpsListener.updateTf();
        
        /*
         Build the view/camera matrix:
         */
        cameraAxis.x = 0;
        cameraAxis.y = 0;
        cameraAxis.z = 1;
        Mat4D camPan = rotationFromAngleAndUnitAxis(angle, cameraAxis);
        camPan = transpose(camPan);
        
        // HACK HACK HACK
        tfScalar yaw = 0, pitch = 0, roll = 0;
        tf::Quaternion myQuaternion(mImuListener.q[1], mImuListener.q[2], mImuListener.q[3], mImuListener.q[0]);
        tf::Matrix3x3 mat(myQuaternion);
        mat.getEulerYPR(yaw, pitch, roll);
//        mGpsListener.heading = yaw -4.0*M_PI/180.0;   // magnetic declination Nashville from https://www.magnetic-declination.com/USA/Nashville/2807841.html#
        // Also https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        mGpsListener.headingFromImu = yaw - 4.04*M_PI/180.0;
        
        cameraAxis.x = 1;
        cameraAxis.y = 0;
        cameraAxis.z = 0;
        Mat4D camTilt = rotationFromAngleAndUnitAxis(tilt, cameraAxis);
        camTilt = transpose(camTilt);
        
        cameraOrientation = matrixMultiply(camTilt, camPan);
        
        if(autoRotate){
            double distance = 10*sin(angle/2);
            cameraTranslation = translationMatrix(-(25+distance)*sin(angle), (25+distance) * cos(angle), -(10*cos(angle/5)+distance)-30);
        }
        viewMatrix = matrixMultiply( cameraOrientation, cameraTranslation);
        
        double kalmanYaw = 0;
        if(showUnitVectors) {
#define NUM_FRAMES (5)
            Mat4D frame[NUM_FRAMES];
            
            // 0, 0, 0
            frame[0] = rotationFromAngleAndUnitAxis(0, cameraAxis);
            
            // TF base_link -> odom
            geometry_msgs::TransformStamped transformStamped;
            // base_link -> odom frame
            tfScalar yaw = 0, pitch = 0, roll = 0;
            try {
                transformStamped = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0));
                tf::Quaternion myQuaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
                tf::Matrix3x3 mat(myQuaternion);
                mat.getEulerYPR(yaw, pitch, roll);
            } catch (tf2::TransformException &ex) {
                yaw = pitch = roll = 0;
            }
            cameraAxis.x = 0;
            cameraAxis.y = 0;
            cameraAxis.z = 1;
            Mat4D frameRotation = rotationFromAngleAndUnitAxis(yaw, cameraAxis);
            Mat4D frameTranslation = translationMatrix(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            frame[1] = matrixMultiply(frameTranslation, frameRotation);
            
            // Again, but get utm -> base_link heading (not a unit vector)
            try {
                transformStamped = tfBuffer.lookupTransform("utm", "base_link", ros::Time(0));
                tf::Quaternion myQuaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
                tf::Matrix3x3 mat(myQuaternion);
                mat.getEulerYPR(kalmanYaw, pitch, roll);
            } catch (tf2::TransformException &ex) {
                kalmanYaw = pitch = roll = 0;
            }
            
            
            frame[2] = mImuListener.orientation;
            
            frameTranslation = translationMatrix(20, 20, 0);
            frame[3] = scaleMatrix( mImuListener.acceleration.x/9.81+1, mImuListener.acceleration.y/9.81+1, mImuListener.acceleration.z/9.81+1);
            frame[3] = matrixMultiply(frameTranslation, frame[3]);

            frameTranslation = translationMatrix(-20, 20, 0);
            frame[4] = scaleMatrix( mImuListener.rotationRate.x+1, mImuListener.rotationRate.y+1, mImuListener.rotationRate.z+1);
            frame[4] = matrixMultiply(frameTranslation, frame[4]);
            
            for(int i = 0; i < NUM_FRAMES; i++) {
//                numEdges = sizeof(cubeQuadIndices)/sizeof(cubeQuadIndices[0]);
                
                Mat4D scale = scaleMatrix(10, 1, 1);
                Mat4D translation = translationMatrix(1, 0, 0);
                Mat4D model = matrixMultiply(scale, translation);
                model = matrixMultiply(frame[i], model);
                Mat4D modelView = matrixMultiply(viewMatrix, model);
//                Coordinates3D color = {1, 0, 0};
////                rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, windowFull, (void*)&color, &depthBuffer, lightModelFs, debugLine);
//                mRenderPipeline.setFragmentShader(lightModelFs);
//                mRenderPipeline.rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, mRenderPipeline.viewport, (void*)&color, debugLine);
                mRenderPipeline.setFragmentShader(boxFs);
                mUniformInfo.modelView = modelView;
                mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
                mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
                mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
                mRenderPipeline.rasterizeShader(unitCubeScene.meshes[0].vi, &mUniformInfo, unitCubeScene.meshes[0].numTriangles, (void*)&colorRed, boxVertexShader);
                
                scale= scaleMatrix(1, 10, 1);
                translation = translationMatrix(0, 1, 0);
                model = matrixMultiply(scale, translation);
                model = matrixMultiply(frame[i], model);
                modelView = matrixMultiply(viewMatrix, model);
//                color = {0, 1, 0};
////                rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, windowFull, (void*)&color, &depthBuffer, lightModelFs, debugLine);
//                mRenderPipeline.setFragmentShader(lightModelFs);
//                mRenderPipeline.rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, mRenderPipeline.viewport, (void*)&color, debugLine);
                
                mRenderPipeline.setFragmentShader(boxFs);
                mUniformInfo.modelView = modelView;
                mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
                mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
                mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
                mRenderPipeline.rasterizeShader(unitCubeScene.meshes[0].vi, &mUniformInfo, unitCubeScene.meshes[0].numTriangles, (void*)&colorGreen, boxVertexShader);
                
                
                scale = scaleMatrix(1, 1, 10);
                translation = translationMatrix(0, 0, 1);
                model = matrixMultiply(scale, translation);
                model = matrixMultiply(frame[i], model);
                modelView = matrixMultiply(viewMatrix, model);
//                color = {0, 0, 1};
////                rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, windowFull, (void*)&color, &depthBuffer, lightModelFs, debugLine);
//                mRenderPipeline.setFragmentShader(lightModelFs);
//                mRenderPipeline.rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, modelView, projection, mRenderPipeline.viewport, (void*)&color, debugLine);
                
                mRenderPipeline.setFragmentShader(boxFs);
                mUniformInfo.modelView = modelView;
                mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
                mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
                mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
                mRenderPipeline.rasterizeShader(unitCubeScene.meshes[0].vi, &mUniformInfo, unitCubeScene.meshes[0].numTriangles, (void*)&colorBlue, boxVertexShader);
            }
        }
        
        int numObjects = 16;
        for( int i = 0; i < numObjects; i++) {
            Mat4D scale = scaleMatrix(1,1,1);//0.15*5, 0.15*5, 0.15*5);
//            Mat4D translation = translationMatrix(30 + 40*sin(lightAngle*(i+1)/(double)numObjects), i-numObjects/2, 1);
            // Notice I swapped x and y here:
            Mat4D translation = translationMatrix(mRadarListener.points[i].x + RAV4_LENGTH/2, -mRadarListener.points[i].y, RAV4_HEIGHT/2);
            Mat4D model = matrixMultiply(translation, scale);
            Mat4D objectModelView = matrixMultiply(viewMatrix, model);
//            numEdges = sizeof(cubeQuadIndices)/sizeof(cubeQuadIndices[0]);
            
//            Coordinates3D color = {0,0,1};
            
//            rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, objectModelView, projection, windowFull, (void*)&color, &depthBuffer, lightModelFs, debugLine);
//            mRenderPipeline.setFragmentShader(lightModelFs);
//            mRenderPipeline.rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, objectModelView, projection, mRenderPipeline.viewport, (void*)&color, debugLine);
            
            if(mRadarListener.points[i].z > 0) {
                colorRadar[i] = {1,0,0};
            } else {
                colorRadar[i] = {0,0,1};
            }
            mRenderPipeline.setFragmentShader(boxFs);
            mUniformInfo.modelView = objectModelView;
            mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
            mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
            mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
            mRenderPipeline.rasterizeShader(unitCubeScene.meshes[0].vi, &mUniformInfo, unitCubeScene.meshes[0].numTriangles, (void*)&colorRadar[i], boxVertexShader);
            
        }
        
        // Grid:
        if (showGrid) {
////            double latitudeInMeters = mGpsListener.longitude * METERS_PER_DEGREE;
////            double longitudeInMeters = mGpsListener.latitude * METERS_PER_DEGREE;
//            double latitudeInMeters = -mGpsListener.y;
//            double longitudeInMeters = -mGpsListener.x;
//
//            double offestLat = fmod(latitudeInMeters *carScale, GRID_LINE_UNIT );
//            double offestLong = fmod(longitudeInMeters*carScale, GRID_LINE_UNIT );
//
//            Mat4D gridOffset = translationMatrix(offestLong, offestLat, 0 );
//
//            cameraAxis.x = 0;
//            cameraAxis.y = 0;
//            cameraAxis.z = 1;
////            Mat4D gridRotation = rotationFromAngleAndUnitAxis(-mGpsListener.heading - M_PI/2.0, cameraAxis);
//            Mat4D gridRotation = rotationFromAngleAndUnitAxis(-mGpsListener.theta, cameraAxis);
//
//            Coordinates4D grid2[sizeof(grid)/sizeof(grid[0])];
//            for (int i = 0; i < sizeof(grid2)/sizeof(grid2[0]); i++) {
//                grid2[i] = grid[i];
//                // Model
//                grid2[i] = matrixVectorMultiply(gridOffset, grid2[i]);
//                grid2[i] = matrixVectorMultiply(gridRotation, grid2[i]);
//
//                // View
//                grid2[i] = matrixVectorMultiply(viewMatrix, grid2[i]);
//
//                // Projection
//                grid2[i] = matrixVectorMultiply(projection, grid2[i]);
//            }
////            numEdges = sizeof(gridEdgeIndices)/sizeof(gridEdgeIndices[0]);
//
////            rasterize(grid2, gridEdgeIndices, numEdges, windowFull,  &depthBuffer);
            
            
            // Square floor:
            
//            testIncrementor += 0.03;
//            testLong += 0.000010 * cos(testIncrementor*0.1);
//            testLat  += 0.00001 * sin(testIncrementor*0.1);
//            mMapHandler.setLongLat(testLong, testLat);
            
            mMapHandler.setLongLat(mGpsListener.longitudeRaw, mGpsListener.latitudeRaw, zoom);
            MapTextureSet mMapTextureSet;
            mMapHandler.getCurrentSet(mMapTextureSet);
            
            //        mRenderPipeline.trianglesFill(squareVi, squareViIndices, 2);
//            Mat4D modelTranslation = translationMatrix(0, 0, 0 );
//            Mat4D modelScale = mMapHandler.mapModel;// scaleMatrix(30, 30, 30);
//            Mat4D modelSquare = matrixMultiply(modelTranslation, modelScale);
            
            
            // Map
            
            mRenderPipeline.setFragmentShader(mapShader);
            Coordinates3D mapRotationAxis = {0,0,1};
            Mat4D mapRotation = rotationFromAngleAndUnitAxis(-mGpsListener.theta, mapRotationAxis);
            for(int i = 0; i < mMapTextureSet.length; i++) {
//            for(int i = 0; i < 4; i++) {
//                Mat4D modelViewMap = matrixMultiply(mapRotation, mMapHandler.mapModel[i]);
                Mat4D modelViewMap = matrixMultiply(mapRotation, *mMapTextureSet.models[i]);
                modelViewMap = matrixMultiply(viewMatrix, modelViewMap);
                mUniformInfo.modelView = modelViewMap;
                mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
//                mRenderPipeline.rasterizeShader(squareVi, &mUniformInfo, squareViIndices, 2, (void*)&mMapHandler.map[i], myVertexShader);
                mRenderPipeline.rasterizeShader(squareVi, &mUniformInfo, squareViIndices, 2, mMapTextureSet.textures[i], myVertexShader);
            }
            
        }
        
        // Car:
        Mat4D carScaleMatrix = scaleMatrix(RAV4_LENGTH/2, RAV4_WIDTH/2, RAV4_HEIGHT/2);
        Mat4D carTranslation = translationMatrix(0, 0, RAV4_HEIGHT/2 );
        Mat4D carModel = matrixMultiply(carTranslation, carScaleMatrix);
        Mat4D carModelView = matrixMultiply(viewMatrix, carModel);
//        numEdges = sizeof(cubeQuadIndices)/sizeof(cubeQuadIndices[0]);
//
//        rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, carModelView, projection, windowFull, (void*)&carColor, &depthBuffer, lightModelFs, debugLine);
//        mRenderPipeline.setFragmentShader(lightModelFs);
//        mRenderPipeline.rasterizeQuadsShader(cube, cubeQuadIndices, numEdges, carModelView, projection, mRenderPipeline.viewport, (void*)&carColor, debugLine);
        
        mRenderPipeline.setFragmentShader(boxFs);
        mUniformInfo.modelView = carModelView;
        mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
        mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
        mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
        mRenderPipeline.rasterizeShader(unitCubeScene.meshes[0].vi, &mUniformInfo, unitCubeScene.meshes[0].numTriangles, (void*)&colorCar, boxVertexShader);
        
        // GPS Covariance sphere:
        if(mGpsListener.covX == mGpsListener.covX &&
           mGpsListener.covY == mGpsListener.covY &&
           mGpsListener.covX != 0 &&
           mGpsListener.covY != 0) {
            cameraAxis.x = 0;
            cameraAxis.y = 0;
            cameraAxis.z = 1;
            Mat4D covScale = scaleMatrix(100, 100, 100);
            Mat4D covRotation = rotationFromAngleAndUnitAxis(-mGpsListener.theta, cameraAxis);
//            Mat4D covScale = scaleMatrix(10, 10, 1);
//            Mat4D covTranslation = translationMatrix(0, 0, 0.5 );
            Mat4D covModel = matrixMultiply(covRotation, mGpsListener.covarianceGps);
            covModel = matrixMultiply(covModel, covScale);
//            covModel = matrixMultiply(covTranslation, covModel);
            Mat4D covModelView = matrixMultiply(viewMatrix, covModel);
//            Coordinates3D covColor = {0.5, 0, 1};
            
            
//            rasterizePolygonsShader(&unitCircle, 1, covModelView, projection, windowFull, (void*)&covColor, &depthBuffer, lightModelFs, debugLine);
            mRenderPipeline.setFragmentShader(covarianceFs);
//            mRenderPipeline.rasterizePolygonsShader(&unitCircle, 1, covModelView, projection, mRenderPipeline.viewport, (void*)&covColor, debugLine);
            
            mUniformInfo.modelView = covModelView;
            mUniformInfo.modelViewProjection = matrixMultiply(projection, mUniformInfo.modelView);
            mUniformInfo.normalMatrix = invert3x3Slow(mUniformInfo.modelView);
            mUniformInfo.normalMatrix = transpose(mUniformInfo.normalMatrix);
            
            mRenderPipeline.backfaceCulling = false;
            mRenderPipeline.rasterizeShader(unitSphereScene.meshes[0].vi, &mUniformInfo, unitSphereScene.meshes[0].numTriangles, (void*)&colorCovariance, covarianceVertexShader);
            mRenderPipeline.backfaceCulling = true;
            mRenderPipeline.rasterizeShader(unitSphereScene.meshes[0].vi, &mUniformInfo, unitSphereScene.meshes[0].numTriangles, (void*)&colorCovariance, covarianceVertexShader);
            
            
        }
        
        /*
         End of 3D rendering
         */
        mRenderPipeline.renderBufferToTerminal();
        /*
         Start of HUD rendering
         */
        
        now = std::chrono::high_resolution_clock::now();
        float_ms = now - before;
        before = now;
        
        mvprintw(debugLine++, 0, "Hello darkness...");
        mvprintw(debugLine++, 0, "Press ESC to quit");
        mvprintw(debugLine++, 0, "FPS: % 0.2f", 1000/float_ms.count());
        
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
        
        mGpsListener.draw(0,3);
        
        steeringCenter.x -= steeringWidth*2 + 1;
        double angles[2] ;
        angles[0] = mGpsListener.heading;// yaw -4.0*M_PI/180.0;
        angles[1] = kalmanYaw;
        const char* labels[2];
        labels[0] = "Heading";
        labels[1] = "Kalman";
        drawGuage(angles, 2, steeringCenter, steeringWidth, steeringHeight, labels);
//        drawGuage(yaw -4.0*M_PI/180.0, steeringCenter, steeringWidth, steeringHeight, "Heading");
//
//        steeringCenter.x -= steeringWidth*2 + 1;
//        drawGuage(kalmanYaw, steeringCenter, steeringWidth, steeringHeight, "Kalman Heading");
        
        mImuListener.printMagnetic(30, 0);
        
        if (autoRotate) {
            angle -= 0.01;
        }
        
        
        
        int ch;
        if ((ch = getch()) == 0x1B) {    // Escape
            keepRunning = false;
        } else if (ch == KEY_RESIZE) {

            getmaxyx(stdscr, screenSizeY, screenSizeX);

            
            mRenderPipeline.resize(screenSizeX, screenSizeY);
            
            screenAspect = (double)screenSizeX/(double)screenSizeY / characterAspect;
//            windowFull = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);
            mRenderPipeline.viewport = makeWindowTransform(screenSizeX, screenSizeY, characterAspect);

            
            if (usePerspective) {
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                projection = projectionMatrixOrtho(orthoScale*screenAspect, orthoScale, zFar, zNear);
            }
//            depthBuffer.setSize(screenSizeX, screenSizeY);
//            depthBuffer.setSize(screenSizeX, screenSizeY);
        } else if (ch == 'o' || ch == 'O') {
            usePerspective    = !usePerspective;
            if (usePerspective) {
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                projection = projectionMatrixOrtho(orthoScale*screenAspect, orthoScale, zFar, zNear);
            }
        } else if ( ch == 'g' || ch == 'G') {
            showGrid = !showGrid;
        } else if ( ch == KEY_LEFT) {
            angle += M_PI/60.0;
        } else if ( ch == KEY_RIGHT) {
            angle -= M_PI/60.0;
        } else if ( ch == KEY_UP) {
            tilt += M_PI/120.0;
        } else if ( ch == KEY_DOWN) {
            tilt -= M_PI/120.0;
        } else if ( ch == 'a' || ch == 'A') {
            // Left
            Mat4D motion = translationMatrix(+1.00, 0, 0);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 'd' || ch == 'D') {
             // Right
            Mat4D motion = translationMatrix(-1.00, 0, 0);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
           
        } else if ( ch == 'w' || ch == 'W') {
            // Forward
            Mat4D motion = translationMatrix(0, 0, 1.00);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 's' || ch == 'S') {
            // Back
            Mat4D motion = translationMatrix(0, 0, -1.00);
            Mat4D camTrans = transpose(cameraOrientation);
            Mat4D offset = matrixMultiply(camTrans, motion);
            offset.d[0][0] = 1; offset.d[0][1] = 0;  offset.d[0][2] = 0;
            offset.d[1][0] = 0; offset.d[1][1] = 1;  offset.d[1][2] = 0;
            offset.d[2][0] = 0; offset.d[2][1] = 0;  offset.d[2][2] = 1;
            cameraTranslation = matrixMultiply(offset, cameraTranslation);
        } else if ( ch == 'r' || ch == 'R') {
            if (usePerspective) {
                viewAngle += M_PI * 0.0125;
                if(viewAngle >= M_PI) {
                    viewAngle = M_PI - 0.0001;
                }
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                orthoScale += 0.5;  // this can grow as large as we want
                projection = projectionMatrixOrtho(orthoScale*screenAspect, orthoScale, zFar, zNear);
            }
        } else if ( ch == 't' || ch == 'T') {
            if (usePerspective) {
                viewAngle -= M_PI * 0.0125;
                if(viewAngle < 0.0001) {
                    viewAngle = 0.0001;
                }
                projection = projectionMatrixPerspective(viewAngle, screenAspect, zFar, zNear);
            } else {
                orthoScale -= 0.5;
                if(orthoScale  < 0.0100) {
                    orthoScale = 0.01;
                }
                projection = projectionMatrixOrtho(orthoScale*screenAspect, orthoScale, zFar, zNear);
            }
        } else if ( ch == ' ' ) {
            autoRotate = !autoRotate;
        } else if ( ch == '-' || ch == '_' ) {
            if(zoom > 2)
                zoom--;
        } else if ( ch == '=' || ch == '+' ) {
            if(zoom < 19)
                zoom++;
        } else if ( ch == 'd' || ch == 'D' ) {
            showDepth = !showDepth;
        } else if ( ch == 'u' || ch == 'U' ) {
            showUnitVectors = !showUnitVectors;
        } else if ( ch == 'l' || ch == 'L') {
            mLocalizationMethod = (GpsListener::LocalizationMethod)(((int)mLocalizationMethod) + 1);
            if(mLocalizationMethod == GpsListener::LOCALIZATION_COUNT) {
                mLocalizationMethod = (GpsListener::LocalizationMethod)0;
            }
            
            mGpsListener.setLocalizationMethod(mLocalizationMethod);
        }
        
       
        ros::spinOnce();
    }

    return 0;
}
