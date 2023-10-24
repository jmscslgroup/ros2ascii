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

//#include <curses-gfx-types.h>
#include <curses-gfx-texture.h>

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


class MapHandler {
private:
    double longitude;
    double latitude;
    
    OsmTile currentTile;
    double currentTileWidth;
    double currentTileTopLeftLong;
    double currentTileTopLeftLat;
    
public:
    int zoom;
    
    Texture map[4];
    bool mapGood[4];
    
    Mat4D mapModel[4];
    
    MapHandler();
    
    bool insideMap( double longitude, double latitude);
    void loadTextureFromTile( const OsmTile& tile, int index );
    
    void updateModelMatrix(const OsmTile& tile, int index);
    
    void updateAdjacentTiles();
    void setLongLat( double longitude, double latitude);
};

#endif //OSM_LOADER_H
