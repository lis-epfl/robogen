/*
 * @(#) Terrain.cpp   1.0   Mar 12, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef DISABLE_HEIGHT_MAP
#include <osgDB/ReadFile>
#endif
#include <osg/Image>
#include "scenario/Terrain.h"



namespace robogen {

Terrain::Terrain(dWorldID odeWorld, dSpaceID odeSpace) :
		odeWorld_(odeWorld), odeSpace_(odeSpace), type_(TerrainConfig::EMPTY),
		heightField_(NULL), odeGeometry_(NULL) {

}

Terrain::~Terrain() {
	if (this->heightField_ != NULL) {
		dGeomHeightfieldDataDestroy(this->heightField_);
	}
}

TerrainConfig::TerrainType Terrain::getType() {
	return type_;
}

bool Terrain::initFlat(float width, float depth) {

	if (this->heightField_ != NULL) {
		dGeomHeightfieldDataDestroy(this->heightField_);
	}

	dCreatePlane(odeSpace_, 0.0, 0.0, 1.0, 0.0);

	heightFieldWidth_ = width;
	heightFieldDepth_ = depth;
	type_ = TerrainConfig::FLAT;

	return true;

}

#ifndef DISABLE_HEIGHT_MAP
bool Terrain::initRough(const std::string& heightMapFileName, float width,
		float depth, float height) {

	osg::ref_ptr<osg::Image> image = osgDB::readImageFile(heightMapFileName);
	if (image == NULL) {
		std::cout << "Cannot load the height map file '" << heightMapFileName
				<< "' for the terrain. Quit." << std::endl;
		return false;
	}

	type_ = TerrainConfig::ROUGH;

	// Try with trimesh!!
	heightFieldData_ = image;

	heightFieldWidth_ = width;
	heightFieldDepth_ = depth;
	heightFieldHeight_ = height;

	int xCount = image->s();
	int yCount = image->t();

	float startX = -width / 2;
	float startY = -depth / 2;

	float spacingX = width / (xCount - 1);
	float spacingY = depth / (yCount - 1);

	double *mapData = new double[xCount * yCount * 3];
	for (int x = 0; x < xCount; ++x) {
		for (int y = 0; y < yCount; ++y) {

			int j = x + xCount * y;

			mapData[3 * j] = startX + spacingX * x;
			mapData[3 * j + 1] = startY + spacingY * y;
			mapData[3 * j + 2] = (*image->data(x, y) / 255.0) * height;

		}
	}

	dTriIndex *indices = new dTriIndex[xCount * yCount * 3];
	int index = 0;
	for (int c = 0; c < xCount * yCount; ++c) {
		indices[index++] = c;
		indices[index++] = c + xCount;
		indices[index++] = c + 1;
	}

	dTriMeshDataID trimesh = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildDouble(trimesh, &mapData[0], 3 * sizeof(double),
			xCount * yCount, &indices[0], xCount * yCount * 3,
			3 * sizeof(dTriIndex));

	odeGeometry_ = dCreateTriMesh(odeSpace_, trimesh, 0, 0, 0);

	return true;

}
#endif

osg::ref_ptr<osg::Image> Terrain::getHeightFieldData() {
	return heightFieldData_;
}

float Terrain::getWidth() const {
	return heightFieldWidth_;
}

float Terrain::getHeightFieldHeight() const {
	return heightFieldHeight_;
}

float Terrain::getDepth() const {
	return heightFieldDepth_;
}

void Terrain::getBB(double& minX, double& maxX, double& minY, double& maxY,
		double& minZ, double& maxZ) {

	minX = 100000000;
	minY = 100000000;
	minZ = 100000000;
	maxX = -100000000;
	maxY = -100000000;
	maxZ = -100000000;

	if (odeGeometry_ != NULL) {

		dReal aabb[6];
		dGeomGetAABB(odeGeometry_, aabb);

		if (aabb[0] < minX) {
			minX = aabb[0];
		}

		if (aabb[1] > maxX) {
			maxX = aabb[1];
		}

		if (aabb[2] < minY) {
			minY = aabb[2];
		}

		if (aabb[3] > maxY) {
			maxY = aabb[3];
		}

		if (aabb[4] < minZ) {
			minZ = aabb[4];
		}

		if (aabb[5] > maxZ) {
			maxZ = aabb[5];
		}

	}
}

}

