/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2020-  Equinor ASA
//
//  ResInsight is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  ResInsight is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.
//
//  See the GNU General Public License at <http://www.gnu.org/licenses/gpl.html>
//  for more details.
//
/////////////////////////////////////////////////////////////////////////////////

#include "RigFractureModelLogExtractor.h"

#include "RiaLogging.h"

#include "RigActiveCellInfo.h"
#include "RigEclipseCaseData.h"
#include "RigEclipseWellLogExtractor.h"
#include "RigMainGrid.h"
#include "RigResultAccessor.h"
#include "RigWellLogExtractionTools.h"
#include "RigWellPath.h"
#include "RigWellPathIntersectionTools.h"

#include "cvfBoundingBox.h"
#include "cvfGeometryTools.h"
#include "cvfPlane.h"

#include <iostream>
#include <map>
#include <sstream>

std::string toString( const cvf::Vec3d& vec )
{
    std::stringstream stream;
    stream << "[" << vec.x() << " " << vec.y() << " " << vec.z() << "]";
    return stream.str();
}

//==================================================================================================
///
//==================================================================================================
RigFractureModelLogExtractor::RigFractureModelLogExtractor( const RigEclipseCaseData* aCase,
                                                            const cvf::Vec3d&         position,
                                                            const cvf::Vec3d&         direction,
                                                            double                    measuredDepth )
    : m_caseData( aCase )
{
    // Create a "fake" well path which from top to bottom of formation
    // passing through the point and with the given direction

    m_wellPath = new RigWellPath;

    const cvf::BoundingBox& geometryBoundingBox = aCase->mainGrid()->boundingBox();

    std::cout << "All cells bounding box: " << toString( geometryBoundingBox.min() ) << " "
              << toString( geometryBoundingBox.max() ) << std::endl;
    std::cout << "Position: " << toString( position ) << std::endl;
    std::cout << "Direction: " << toString( direction ) << std::endl;

    // Create plane on top and bottom of formation
    cvf::Plane topPlane;
    topPlane.setFromPointAndNormal( geometryBoundingBox.max(), cvf::Vec3d::Z_AXIS );
    cvf::Plane bottomPlane;
    bottomPlane.setFromPointAndNormal( geometryBoundingBox.min(), cvf::Vec3d::Z_AXIS );

    // Convert direction up for z
    cvf::Vec3d directionUp( direction.x(), direction.y(), -direction.z() );

    // Find and add point on top plane
    cvf::Vec3d abovePlane = position + ( directionUp * 10000.0 );
    cvf::Vec3d topPosition;
    topPlane.intersect( position, abovePlane, &topPosition );
    double topMD = 0.0;
    m_wellPath->m_wellPathPoints.push_back( topPosition );
    m_wellPath->m_measuredDepths.push_back( topMD );
    std::cout << "TOP:    " << toString( topPosition ) << " MD: " << topMD << std::endl;

    // The anchor position
    double dist = ( topPosition - position ).length();
    m_wellPath->m_wellPathPoints.push_back( position );
    m_wellPath->m_measuredDepths.push_back( dist );
    std::cout << "ANCHOR: " << toString( position ) << " MD: " << dist << std::endl;

    // Find and add point on bottom plane
    cvf::Vec3d belowPlane = position + ( directionUp * -10000.0 );
    cvf::Vec3d bottomPosition;
    bottomPlane.intersect( position, belowPlane, &bottomPosition );

    double dist2 = ( topPosition - bottomPosition ).length();
    m_wellPath->m_wellPathPoints.push_back( bottomPosition );
    m_wellPath->m_measuredDepths.push_back( dist2 );
    std::cout << "BOTTOM: " << toString( bottomPosition ) << " MD: " << dist2 << std::endl;

    m_wellLogExtractor = new RigEclipseWellLogExtractor( aCase, &*m_wellPath, "fracture model" );
}

//==================================================================================================
///
//==================================================================================================
void RigFractureModelLogExtractor::curveData( const RigResultAccessor* resultAccessor, std::vector<double>* values )
{
    m_wellLogExtractor->curveData( resultAccessor, values );
}

//==================================================================================================
///
//==================================================================================================
const std::vector<double>& RigFractureModelLogExtractor::cellIntersectionTVDs() const
{
    return m_wellLogExtractor->cellIntersectionTVDs();
}

//==================================================================================================
///
//==================================================================================================
const std::vector<double>& RigFractureModelLogExtractor::cellIntersectionMDs() const
{
    return m_wellLogExtractor->cellIntersectionMDs();
}

//==================================================================================================
///
//==================================================================================================
const RigWellPath* RigFractureModelLogExtractor::wellPathData() const
{
    return m_wellLogExtractor->wellPathData();
}
