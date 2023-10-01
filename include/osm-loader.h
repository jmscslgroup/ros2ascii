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

#endif //OSM_LOADER_H
