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

void tileToOffsetMeters(const OsmTile& tile, double longitude, double latitude, double& xOffset, double& yOffset) {
//    OsmTile tile = latLongToTile( zoom, longitude, latitude);
    
    double longTile, latTile;
    tileToLatLong(tile, longTile, latTile);
    
    const double EARTH_CIR_METERS = 40075016.686;
    const double degreesPerMeter = 360 / EARTH_CIR_METERS;
    
    double metersPerPixelEW = EARTH_CIR_METERS / (double)(1 << (tile.zoom + 8));
    double metersPerPixelNS = EARTH_CIR_METERS / (double)(1 << (tile.zoom + 8)) * cos(latitude*3.1415926535897/180.0);
    
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


MapHandler::MapHandler() {
    longitude = 0;
    latitude = 0;
    zoom = 18;
    for (int i = 0; i < 4; i++) {
        mapGood[i] = false;
    }
    
    currentTileWidth = 0;
    currentTileTopLeftLong = 0;
    currentTileTopLeftLat = 0;
    mapModel[0] = translationMatrix(0,0,0);
}



bool MapHandler::insideMap( double longitude, double latitude) {
//        OsmTile tile = {zoom, longitude, latitude};
//        tileToLatLong( tile, longitude, latitude);
    OsmTile mOsmTile = latLongToTile(zoom, longitude, latitude);
    return
    (currentTile.x == mOsmTile.x) &&
    (currentTile.y == mOsmTile.y) &&
    (currentTile.zoom == mOsmTile.zoom)
    ;
   
}

void MapHandler::loadTextureFromTile( const OsmTile& tile, int index ) {
//        tileToLatLong( currentTile, currentTileTopLeftLong, currentTileTopLeftLat);
//        currentTileWidth = tileWidthInMeters( latitude, zoom);
//        mapModel = scaleMatrix(currentTileWidth,currentTileWidth,0);
    
    char tileFile[200];
//        if(downloadTile(tile, "/home/matt", tileFile)) {
    std::string homeDirectory = "/home/" + readFileContents("/etc/libpanda.d/libpanda_usr") + "/";
    
    if(downloadTile(tile, homeDirectory.c_str(), tileFile)) {
        map[index].resize(1,1);
        
    } else {
        map[index].loadPng(tileFile);
//            map[index].offsetAvergageToCenter(0.75);
//            map[index].normalize(1.);
//            map[index].invert();
        
        
        map[index].invert();
        map[index].scale(3);
//            map[index].normalize(1.);
    }
    updateModelMatrix(tile, index);
}

void MapHandler::updateModelMatrix(const OsmTile& tile, int index) {
    double xOffset, yOffset;
//        longLatToOffsetMeters(zoom, longitude, latitude, xOffset, yOffset);
    tileToOffsetMeters(tile, longitude, latitude, xOffset, yOffset);
    
    Mat4D translation = translationMatrix(xOffset,yOffset,0);
    Mat4D scale = scaleMatrix(currentTileWidth,currentTileWidth,0);
    mapModel[index] = matrixMultiply( translation, scale);
}

void MapHandler::updateAdjacentTiles() {
    double xOffset, yOffset;
    tileToOffsetMeters(currentTile, longitude, latitude, xOffset, yOffset);
    
    OsmTile adjacentTileX = currentTile;
    if(-xOffset < currentTileWidth/2.0) {
        adjacentTileX.x--;
    } else {
        adjacentTileX.x++;
    }
    loadTextureFromTile( adjacentTileX, 1 );
    
    OsmTile adjacentTileY = currentTile;
    if(yOffset < currentTileWidth/2.0) {
        adjacentTileY.y--;
    } else {
        adjacentTileY.y++;
    }
    loadTextureFromTile( adjacentTileY, 2 );
    
    
    OsmTile adjacentTileXY = currentTile;
    adjacentTileXY.x = adjacentTileX.x;
    adjacentTileXY.y = adjacentTileY.y;
    loadTextureFromTile( adjacentTileXY, 3 );
    
}

void MapHandler::setLongLat( double longitude, double latitude) {
    this->longitude = longitude;
    this->latitude = latitude;
    
    if(insideMap(longitude, latitude)){
        updateModelMatrix(currentTile, 0);
    } else {
        currentTile = latLongToTile(zoom, longitude, latitude);
        tileToLatLong( currentTile, currentTileTopLeftLong, currentTileTopLeftLat);
        currentTileWidth = tileWidthInMeters( latitude, zoom);
        loadTextureFromTile( currentTile, 0 );
    }
    
    updateAdjacentTiles();
}
