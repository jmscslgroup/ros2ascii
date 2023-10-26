/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.
 
 */

/*
 This Documentaion
 */

#include "osm-loader.h"

#include <future>
#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <unistd.h> // for usleep testing

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
    //    currentSet.setSize(4);
    circleBufferIndex = 0;
    borderTiles = 2;
    tileRows = 1 + 2*borderTiles;
    tileCols = 1 + 2*borderTiles;
    
    map = new Texture[tileRows*tileCols*2];
    mapModel = new Mat4D[tileRows*tileCols*2];
    tileOffset = new Coordinates2D[tileRows*tileCols*2];
    tiles = new OsmTile[tileRows*tileCols*2];
    
    mapValid = new Texture*[tileRows*tileCols];
    mapModelValid = new Mat4D*[tileRows*tileCols];
    tilesValid = new OsmTile*[tileRows*tileCols];
    
    for (int r = 0; r < tileRows; r++) {
        for (int c = 0; c < tileCols; c++) {
            
            tileOffset[r + tileRows*c].x = 0;
            tileOffset[r + tileRows*c].y = 0;
            
            mapValid[r + tileRows*c] = &map[r + tileRows*c];
            mapModelValid[r + tileRows*c] = &mapModel[r + tileRows*c];
            tilesValid[r + tileRows*c] = &tiles[r + tileRows*c];
        }
    }
    
//    currentTileWidth = 0;
    currentTileTopLeftLong = 0;
    currentTileTopLeftLat = 0;
    mapModel[0] = translationMatrix(0,0,0);
    
    //    backHalf = false;
    threadBusy = false;
}

MapHandler::~MapHandler() {
    if(def_fut.valid())
    {
        def_fut.wait();
    }
    delete [] map;
    delete [] mapModel;
    delete [] tileOffset;
    delete [] tiles;
    
    delete [] mapValid;
    delete [] mapModelValid;
    delete [] tilesValid;
}


bool MapHandler::insideCurrentMapTile() {
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
    
    //    if(backHalf) {
    //        index += 4;
    //    }
    
    if(downloadTile(tile, homeDirectory.c_str(), tileFile)) {
        map[index].resize(1,1);
        
    } else {
        map[index].loadPng(tileFile);
        //            map[index].offsetAvergageToCenter(0.75);
        //            map[index].normalize(1.);
        //            map[index].invert();
        
        
        map[index].invert();
        map[index].scale(2);
        //            map[index].normalize(1.);
    }
    //    updateModelMatrix(tile, index);
}


void MapHandler::updateModelMatrix(const OsmTile& tile, Mat4D& model) {
    double xOffset, yOffset;
    //        longLatToOffsetMeters(zoom, longitude, latitude, xOffset, yOffset);
    tileToOffsetMeters(tile, longitude, latitude, xOffset, yOffset);
    double width = tileWidthInMeters( latitude, tile.zoom);
    Mat4D translation = translationMatrix(xOffset, yOffset, 0);
    Mat4D scale = scaleMatrix(width, width, 0);
    model = matrixMultiply( translation, scale);
}



void MapHandler::updateValidTiles() {
    //    int indexOffset = 0;
    //    if(backHalf) {
    //        indexOffset = tileRows*tileCols;
    //    }
    
    //    OsmTile currentTile = latLongToTile(zoom, longitude, latitude);
    //    updateModelMatrix(currentTile, indexOffset);
    //    for(int i = 1; i < 4; i++) {
    //        OsmTile adjacentTile = currentTile;
    //        adjacentTile.x += tileOffset[i+indexOffset].x;
    //        adjacentTile.y += tileOffset[i+indexOffset].y;
    //        updateModelMatrix(adjacentTile, i+indexOffset);
    //    }
    
//    OsmTile currentTile = latLongToTile(tilesValid[0].zoom, longitude, latitude);
    //    updateModelMatrix(currentTile, indexOffset);
    for (int r = 0; r < tileRows; r++) {
        for (int c = 0; c < tileCols; c++) {
//            OsmTile adjacentTile = currentTile;
//            adjacentTile.x += c-1;
//            adjacentTile.y += r-1;
            //            updateModelMatrix(adjacentTile, i+indexOffset);
//            updateModelMatrix(adjacentTile, *mapModelValid[r + tileRows*c]);
            updateModelMatrix(*tilesValid[r + tileRows*c], *mapModelValid[r + tileRows*c]);
            
            
            //            updateModelMatrix(adjacentTile, i+indexOffset);
        }
    }
}

void MapHandler::setLongLat( double longitude, double latitude, int zoom) {
    this->longitude = longitude;
    this->latitude = latitude;
    this->zoom = zoom;
    
    updateValidTiles();
    
    if(!insideCurrentMapTile()) {
        //        updateTiles();
        if(!def_fut.valid() || def_fut.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            //            threadBusy = true;
            def_fut = std::async(std::launch::async, &MapHandler::updateTiles, this);
        }
    }
}


void MapHandler::updateTiles() {
    //    if(threadBusy) {
    //        return;
    //    }
    //    if(insideCurrentMapTile(longitude, latitude)){
    ////        updateModelMatrix(currentTile, 0);
    //    } else {
    OsmTile oldTile = currentTile;
    currentTile = latLongToTile(zoom, longitude, latitude);
    tileToLatLong( currentTile, currentTileTopLeftLong, currentTileTopLeftLat);
//    currentTileWidth = tileWidthInMeters( latitude, zoom);
    
    
    Texture *soonToBeValidMap[tileRows*tileCols];
    Mat4D *soonToBeValidModel[tileRows*tileCols];
    OsmTile *soonToBeValidTiles[tileRows*tileCols];
    
    for(int r = 0; r < tileRows; r++) {
        for(int c = 0; c < tileCols; c++) {
            OsmTile tile;
            tile.zoom = zoom;
            tile.x = currentTile.x + (c-borderTiles);
            tile.y = currentTile.y + (r-borderTiles);
//            usleep(100000);
            int index = checkIfTileExists(tile);
            if(index < 0) {
                index = getIndexOfFreeTile();
                loadTextureFromTile( tile, index );
                updateModelMatrix(tile, mapModel[index]);
            }
//            usleep(100000);
//            {
//                std::unique_lock<std::mutex> lock(copyMutex);
//                mapModelValid[r + c*tileRows] = &mapModel[index];
//                mapValid[r + c*tileRows] = &map[index];
//                tiles[index] = tile;
//                tilesValid[r + c*tileRows] = &tiles[index];
//            }
            soonToBeValidModel[r + c*tileRows] = &mapModel[index];
            soonToBeValidMap[r + c*tileRows] = &map[index];
            tiles[index] = tile;
            soonToBeValidTiles[r + c*tileRows] = &tiles[index];
        }
    }
    
    std::unique_lock<std::mutex> lock(copyMutex);
    for(int i = 0; i < tileRows*tileCols; i++) {
        mapModelValid[i] = soonToBeValidModel[i];
        mapValid[i] = soonToBeValidMap[i];
        tilesValid[i] = soonToBeValidTiles[i];
    }
    
    
    //
    //    //        updateModelMatrix( currentTile 0);
    //    loadTextureFromTile( currentTile, !backHalf ? 4 : 0 );
    //    //    }
    //
    //
    //
    //    updateAdjacentTiles();
    //
    //    int indexOffset = 0;
    ////    if(!backHalf) {
    ////        indexOffset = 4;
    ////    }
    //    updateModelMatrix(currentTile, indexOffset );
    //    for(int i = 1; i < 4; i++) {
    //        OsmTile adjacentTile = currentTile;
    //        adjacentTile.x += tileOffset[i+indexOffset].x;
    //        adjacentTile.y += tileOffset[i+indexOffset].y;
    //        updateModelMatrix(adjacentTile, i+indexOffset);
    //    }
    //
    ////    if(backHalf) {
    ////        backHalf = false;
    ////    } else {
    ////        backHalf = true;
    ////    }
    
    //    threadBusy = false;
}

//MapTextureSet MapHandler::getCurrentSet() {
//    return currentSet;
//}

void MapHandler::getCurrentSet(MapTextureSet& mts) {
    int indexOffset = 0;
    // mutex lock here
    std::unique_lock<std::mutex> lock(copyMutex);
    //    if(backHalf) {
    //        indexOffset = 4;
    //    }
    //
    //    mts.setSize(4);
    //    for(int i = 0; i < 4; i++) {
    //        mts.models[i] = &mapModel[i+indexOffset];
    //        mts.textures[i] = &map[i+indexOffset];
    //    }
    
    mts.setSize(tileRows*tileCols);
    for (int r = 0; r < tileRows; r++) {
        for (int c = 0; c < tileCols; c++) {
            mts.models[r + tileRows*c] = mapModelValid[r + tileRows*c];
            mts.textures[r + tileRows*c] = mapValid[r + tileRows*c];
        }
    }
    
    // then unlock
}


int MapHandler::checkIfTileExists(const OsmTile& tile) {
    for(int i = 0; i < 2*tileRows*tileCols; i++) {
        if(tile.x == tiles[i].x &&
           tile.y == tiles[i].y &&
           tile.zoom == tiles[i].zoom) {
            return i;
        }
    }
    return -1;
}

int MapHandler::getIndexOfFreeTile() {
    int result = -1;
    for(int i = 0; i < 2*tileRows*tileCols; i++) {
        //        if(abs(currentTile.x - tiles[i].x) > 1 ||
        //           abs(currentTile.y - tiles[i].y) > 1 ||
        //           currentTile.zoom != tiles[i].zoom) {
        //            result = i;
        //            break;
        //        }
        int index = -1;
        if(++circleBufferIndex >= 2*tileRows*tileCols) {
            circleBufferIndex = 0;
        }
        for(int r = 0; r < tileRows && index == -1; r++) {
            for(int c = 0; c < tileCols; c++) {
                if(mapModelValid[r + c*tileRows] == &mapModel[circleBufferIndex]) {
                    index = 0;  // index? more like a flag
                    break;
                }
            }
        }
        if(index < 0)
            return circleBufferIndex;
    }
    return result;
}
