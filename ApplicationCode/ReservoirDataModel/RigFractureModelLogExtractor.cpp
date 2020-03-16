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

#include <map>

//==================================================================================================
///
//==================================================================================================
RigFractureModelLogExtractor::RigFractureModelLogExtractor( const RigEclipseCaseData* aCase,
                                                            const cvf::Vec3d&         position,
                                                            const cvf::Vec3d&         direction,
                                                            double                    measuredDepth,
                                                            const cvf::BoundingBox&   geometryBoundingBox )
    : m_caseData( aCase )
{
    // Create a "fake" well path which from top to bottom of formation
    // passing through the point and with the given direction

    m_wellPath = new RigWellPath;

    // m_rimReservoirView->currentActiveCellInfo()->geometryBoundingBox().max();

    // Position on top of formation
    cvf::Plane topPlane;
    topPlane.setFromPointAndNormal( geometryBoundingBox.max(), cvf::Vec3d::Z_AXIS );
    cvf::Vec3d abovePlane = position + ( direction * 10000.0 );
    cvf::Vec3d topPosition;
    topPlane.intersect( position, abovePlane, &topPosition );

    m_wellPath->m_wellPathPoints.push_back( topPosition );
    // TODO: not correct
    m_wellPath->m_measuredDepths.push_back( 0.0 ); // measuredDepth );

    // The anchor position
    m_wellPath->m_wellPathPoints.push_back( position );
    m_wellPath->m_measuredDepths.push_back( measuredDepth );

    m_wellLogExtractor = new RigEclipseWellLogExtractor( aCase, &*m_wellPath, "fracture model" );
}

//==================================================================================================
///
//==================================================================================================
void RigFractureModelLogExtractor::curveData( const RigResultAccessor* resultAccessor, std::vector<double>* values )
{
    m_wellLogExtractor->curveData( resultAccessor, values );
}
