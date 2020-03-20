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

    const cvf::BoundingBox& geometryBoundingBox = aCase->activeCellInfo( RiaDefines::MATRIX_MODEL )->geometryBoundingBox();

    // Position on top of formation
    cvf::Plane topPlane;
    topPlane.setFromPointAndNormal( geometryBoundingBox.max(), cvf::Vec3d::Z_AXIS );
    cvf::Vec3d abovePlane = position + ( direction * 10000.0 );
    cvf::Vec3d topPosition;
    topPlane.intersect( position, abovePlane, &topPosition );

    double depthDiff = 0.0 - topPosition.z();
    m_wellPath->setDatumElevation( topPosition.z() );

    std::cout << "TOP POSITION: " << topPosition.z() << " MD: " << 0.0 << std::endl;

    m_wellPath->m_wellPathPoints.push_back( topPosition );
    m_wellPath->m_measuredDepths.push_back( 0.0 ); // topPosition.z() ); // geometryBoundingBox.max().z() );
                                                   // // measuredDepth );

    // The anchor position
    m_wellPath->m_wellPathPoints.push_back( position );
    double dist = ( topPosition - position ).length();
    std::cout << "ANCHOR: " << position.z() << " MD: " << dist << std::endl;
    m_wellPath->m_measuredDepths.push_back( dist ); // measuredDepth + depthDiff );

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
