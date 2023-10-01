/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.

 */

/*
 This Documentaion
 */

#include "osm-loader.h"

#include <cstdlib>
#include <cstdio>
#include <math.h>

// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

OsmTile latLongToTile(int zoom, double longitude, double latitude) {
    OsmTile result;
    int n = 1 << zoom;
    result.x = n*((longitude + 180) / 360);
    double lat_rad = latitude*3.1415926535897/180.0;
    result.y = n*(1.0 - (log(tan(lat_rad) + 1.0/cos(lat_rad))/3.1415926535897))/2;
    result.zoom = zoom;
    return result;
}

void longLatToOffsetMeters(int zoom, double longitude, double latitude, double& xOffset, double& yOffset) {
    OsmTile tile = latLongToTile( zoom, longitude, latitude);
    
    double longTile, latTile;
    tileToLatLong(tile, longTile, latTile);
    
    const double EARTH_CIR_METERS = 40075016.686;
    const double degreesPerMeter = 360 / EARTH_CIR_METERS;
    
    double metersPerPixelEW = EARTH_CIR_METERS / (double)(1 << (zoom + 8));
    double metersPerPixelNS = EARTH_CIR_METERS / (double)(1 << (zoom + 8)) * cos(latitude*3.1415926535897/180.0);
    
//    const shiftMetersEW = width/2 * metersPerPixelEW;
//      const shiftMetersNS = height/2 * metersPerPixelNS;
    
    xOffset =  ( longTile - longitude)/ degreesPerMeter* cos(latitude*3.1415926535897/180.0);
    yOffset =  ( latTile - latitude)/ degreesPerMeter;
    
}

void tileToLatLong(const OsmTile& tile, double& longitude, double& latitude) {
    int n = 1 << tile.zoom;
    longitude = ((double)tile.x/(double)n) * 360.0 - 180.0;
    double lat_rad = atan(sinh(3.1415926535897*(1.0 - 2.0*((double)tile.y/(double)n))));
    latitude = lat_rad*180.0/3.1415926535897;
}

double tileWidthInMeters(double latitude, int zoom) {
    return 156543.03 * cos(latitude*3.1415926535897/180.0) / (double)(1 << zoom) * 256; // each image is 256 pixels
}

bool downloadTile(const OsmTile& tile, const char* cachedirectory, char* resultPath) {
    char command[256];
    
    snprintf(resultPath, 200, "%s/%d/%d/%d.png", cachedirectory, tile.zoom, tile.x, tile.y);
    
    snprintf(command, 256, "[ -f %s ]", resultPath);
//    printf("Executing command: %s\n", command);
    if(system(command) == 0) {
        return 0;
    }
    snprintf(command, 256, "mkdir -p %s/%d/%d", cachedirectory, tile.zoom, tile.x);
//    printf("Executing command: %s\n", command);
    system(command);
//    snprintf(command, 256, "curl --remove-on-error --max-time 3 https://tile.openstreetmap.org/%d/%d/%d.png -o %s", tile.zoom, tile.x, tile.y, resultPath);
    snprintf(command, 256, "curl --silent --max-time 2 https://tile.openstreetmap.org/%d/%d/%d.png -o %s", tile.zoom, tile.x, tile.y, resultPath);
//    printf("Executing command: %s\n", command);
    
    if(system(command) != 0) {
        return 1;
    }
    
    return 0;
}
