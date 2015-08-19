/*
 * @(#) TerrainRender.cpp   1.0   Mar 12, 2013
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
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgTerrain/GeometryTechnique>
#include <osgTerrain/Terrain>

#include "scenario/Terrain.h"
#include "render/objects/TerrainRender.h"

namespace robogen {

TerrainRender::TerrainRender(boost::shared_ptr<Terrain> terrain) {

	rootNode_ = osg::ref_ptr<osg::PositionAttitudeTransform>(
			new osg::PositionAttitudeTransform);

	if (terrain->isFlat()) {

		// Generate a flat terrain

		osg::ref_ptr<osg::Shape> shape(
				new osg::Box(osg::Vec3(0, 0, -2.5),
						fromOde(terrain->getWidth()),
						fromOde(terrain->getDepth()), 5));

		osg::ref_ptr<osg::ShapeDrawable> osgPlaneDrawable(
				new osg::ShapeDrawable(shape.get()));

		osg::ref_ptr<osg::Geode> osgGroundGeode(new osg::Geode());
		osgGroundGeode->addDrawable(osgPlaneDrawable.get());

		rootNode_->addChild(osgGroundGeode);

	} else {

		// Generate terrain from height field

		// 1) Load the height field

		unsigned int xPointsCount = terrain->getHeightFieldData()->s();
		unsigned int yPointsCount = terrain->getHeightFieldData()->t();

		osg::ref_ptr<osg::HeightField> heightField(new osg::HeightField);
		heightField->allocate(xPointsCount, yPointsCount);
		heightField->setOrigin(osg::Vec3(0, 0, 0));
		heightField->setXInterval(fromOde(terrain->getWidth()) / xPointsCount);
		heightField->setXInterval(fromOde(terrain->getDepth()) / yPointsCount);

		heightField->setSkirtHeight(1.0f);

		// Copy height information to the glfloat array
		for (unsigned int i = 0; i < xPointsCount; ++i) {
			for (unsigned int j = 0; j < yPointsCount; ++j) {
				heightField->setHeight(i, j,
						((*terrain->getHeightFieldData()->data(i, j)) / 255.0)
								* fromOde(terrain->getHeightFieldHeight()));
			}
		}

		// Create a terrain tile
		osg::ref_ptr<osgTerrain::Locator> terrainTileLocator(
				new osgTerrain::Locator());
		terrainTileLocator->setCoordinateSystemType(
				osgTerrain::Locator::PROJECTED);
		terrainTileLocator->setTransformAsExtents(-fromOde(terrain->getWidth())/2, -fromOde(terrain->getDepth())/2,
				fromOde(terrain->getWidth())/2, fromOde(terrain->getDepth())/2);

		osg::ref_ptr<osgTerrain::HeightFieldLayer> heightFieldLayer =
				new osgTerrain::HeightFieldLayer(heightField.get());
		heightFieldLayer->setLocator(terrainTileLocator);

		osg::ref_ptr<osgTerrain::GeometryTechnique> terrainGeomTechnique(
				new osgTerrain::GeometryTechnique());

		osg::ref_ptr<osgTerrain::TerrainTile> terrainTile(
				new osgTerrain::TerrainTile());
		terrainTile->setElevationLayer(heightFieldLayer.get());
		terrainTile->setTerrainTechnique(terrainGeomTechnique.get());

		// Generate an OSG terrain object

		osg::ref_ptr<osgTerrain::Terrain> osgTerrain(new osgTerrain::Terrain());
		osgTerrain->setSampleRatio(1.0f);
		osgTerrain->addChild(terrainTile.get());

		rootNode_->addChild(osgTerrain.get());


		// Uncomment to debug terrain
		/*double minX, minY, minZ, maxX, maxY, maxZ;
		terrain->getBB(minX, maxX, minY, maxY, minZ, maxZ);

		std::cout
					<< "Terrain AABB(minX, maxX, minY, maxY, minZ, maxZ) ("
					<< minX << ", " << maxX << ", " << minY << ", " << maxY << ", "
					<< minZ << ", " << maxZ << ")" << std::endl;

		osg::ref_ptr<osg::Shape> shape(
						new osg::Box(osg::Vec3(fromOde(minX + (maxX-minX)/2),
								fromOde(minY + (maxY-minY)/2), fromOde(minZ +(maxZ-minZ))/2),
								fromOde(maxX-minX),
								fromOde(maxY-minY), fromOde(maxZ-minZ)));

		osg::ref_ptr<osg::ShapeDrawable> osgPlaneDrawable(
				new osg::ShapeDrawable(shape.get()));

		osg::ref_ptr<osg::Geode> osgGroundGeode(new osg::Geode());
		osgGroundGeode->addDrawable(osgPlaneDrawable.get());

		rootNode_->addChild(osgGroundGeode);*/

	}

}

TerrainRender::~TerrainRender() {

}

osg::ref_ptr<osg::PositionAttitudeTransform> TerrainRender::getRootNode() {
	return rootNode_;
}

}

