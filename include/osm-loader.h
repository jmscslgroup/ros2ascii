/*
 Author: Matt Bunting
 Copyright (c) 2023 Vanderbilt
 All rights reserved.

 */

/*
 This Documentaion
 */

// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames

#ifndef OSM_LOADER_H
#define OSM_LOADER_H

#include <mutex>
#include <future>
//#include <curses-gfx-types.h>
#include <curses-gfx-texture.h>
//#include <mogi/thread.h>

#include "resources.h"

typedef struct _OsmTile {
    int zoom;
    int x;
    int y;
} OsmTile;

OsmTile latLongToTile(int zoom, double longitude, double latitude);


void tileToLatLong(const OsmTile& tile, double& longitude, double& latitude);
double tileWidthInMeters(double latitude, int zoom);
bool downloadTile(const OsmTile& tile, const char* cachedirectory, char* resultPath);

void longLatToOffsetMeters(int zoom, double longitude, double latitude, double& xOffset, double& yOffset);
void tileToOffsetMeters(const OsmTile& tile, double longitude, double latitude, double& xOffset, double& yOffset);

class MapTextureSet {
public:
    Mat4D** models;
    Texture** textures;
    int length;
    
    MapTextureSet() : models(NULL), textures(NULL), length(0) {};
    ~MapTextureSet() {
        length = 0;
        if(models != NULL) {
            delete [] models;
        }
        if(textures != NULL) {
            delete [] textures;
        }
    }
    
    void setSize(int size) {
        if(length != size ) {
            models = new Mat4D*[size];
            textures = new Texture*[size];
            length = size;
        }
    }
};

class MapHandler {
private:
    double longitude;
    double latitude;
    int zoom;
    
    int borderTiles;
    int tileRows;
    int tileCols;
    int circleBufferIndex;
    
    // These variables are part of a pool
    Texture *map;
    Mat4D *mapModel;
    Coordinates2D *tileOffset;
    OsmTile *tiles;
    
    // These are refernces to the pool:
    Texture **mapValid;
    Mat4D **mapModelValid;
    OsmTile **tilesValid;
    
    std::future<void> def_fut;
    
    std::mutex copyMutex;
    bool threadBusy;
    
    OsmTile currentTile;
//    double currentTileWidth;
    double currentTileTopLeftLong;
    double currentTileTopLeftLat;
    
//    MapAndTextureSet scratchSpace;
//    MapTextureSet currentSet;
    
    void updateTiles();
    void updateValidTiles();
    
    int checkIfTileExists(const OsmTile& tile);
    int getIndexOfFreeTile();
    
public:
    MapHandler();
    ~MapHandler();
    
    bool insideCurrentMapTile();
    void loadTextureFromTile( const OsmTile& tile, int index );
    
//    void updateModelMatrix(const OsmTile& tile, int index);
    void updateModelMatrix(const OsmTile& tile, Mat4D& model);
    
//    void updateAdjacentTiles();
    void setLongLat( double longitude, double latitude, int zoom);

    void getCurrentSet(MapTextureSet& mts);
};

#endif //OSM_LOADER_H
