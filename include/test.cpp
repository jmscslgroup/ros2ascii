/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.

 */

/*
 This Documentaion
 */
#include <iostream>
#include <cstdio>

#include "osm-loader.h"

int main(void) {
    OsmTile result = latLongToTile(19, -86.7931535, 36.14507466666666);
    
    printf("https://tile.openstreetmap.org/%d/%d/%d.png\n", result.zoom, result.x, result.y);
    
    char tileFile[100];
    if(downloadTile(result, ".", tileFile)) {
        printf("Failed to download file!\n");
        return 1;
    }
    printf("File result: %s\n", tileFile);
    
    
    double latitude, longitude;
    tileToLatLong(result, longitude, latitude);
    printf("Upper left tile Longitude/Latitude: %f,%f\n", longitude, latitude);
    
    
    printf("Width of tile (meters): %f\n", tileWidthInMeters(latitude, result.zoom));
    return 0;
};
