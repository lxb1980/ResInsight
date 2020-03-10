/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2020 Equinor ASA
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

#include "RiaApplication.h"
//#include "RiaColorTables.h"
//#include "RiaFractureDefines.h"

// #include "RigCellGeometryTools.h"
// #include "RigFractureCell.h"
// #include "RigFractureGrid.h"
#include "RigMainGrid.h"
#include "RigTesselatorTools.h"
#include "RigWellPath.h"

#include "RimCase.h"
#include "RimEclipseCase.h"
#include "RimEclipseView.h"
#include "RimFractureModel.h"
#include "RimWellPath.h"
#include "RimWellPathCollection.h"

#include "RivFractureModelPartMgr.h"
#include "RivObjectSourceInfo.h"
#include "RivPartPriority.h"
//#include "RivPipeGeometryGenerator.h"

#include "cafDisplayCoordTransform.h"
#include "cafEffectGenerator.h"

#include "cvfAssert.h"
#include "cvfDrawableGeo.h"
#include "cvfGeometryTools.h"
#include "cvfModelBasicList.h"
#include "cvfPart.h"
#include "cvfPrimitiveSet.h"
#include "cvfPrimitiveSetIndexedUInt.h"
#include "cvfRenderStateDepth.h"
#include "cvfTransform.h"

#include <cmath>

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RivFractureModelPartMgr::RivFractureModelPartMgr( RimFractureModel* fractureModel )
    : m_rimFractureModel( fractureModel )
{
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RivFractureModelPartMgr::~RivFractureModelPartMgr()
{
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RivFractureModelPartMgr::appendGeometryPartsToModel( cvf::ModelBasicList* model, const RimEclipseView& eclView )
{
    if ( !m_rimFractureModel->isChecked() ) return;

    cvf::Collection<cvf::Part> parts;
    auto                       part = createEllipseSurfacePart( eclView );
    if ( part.notNull() ) parts.push_back( part.p() );

    for ( auto& part : parts )
    {
        model->addPart( part.p() );
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
const QString RivFractureModelPartMgr::resultInfoText( const RimEclipseView& activeView,
                                                       cvf::Vec3d            domainIntersectionPoint ) const
{
    QString text;

    if ( m_rimFractureModel.isNull() ) return text;

    text.append( "Fracture Model: " ).append( m_rimFractureModel->name() ).append( "\n" );

    return text;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
cvf::ref<cvf::Part> RivFractureModelPartMgr::createEllipseSurfacePart( const RimEclipseView& activeView )
{
    auto displayCoordTransform = activeView.displayCoordTransform();
    if ( displayCoordTransform.isNull() ) return nullptr;

    if ( m_rimFractureModel )
    {
        std::vector<cvf::uint>  triangleIndices;
        std::vector<cvf::Vec3f> nodeDisplayCoords;

        {
            RigEllipsisTesselator tesselator( 20 );

            float height      = 100.0;
            float halfLength  = 75.0;
            float scaleFactor = 1.0;
            float a           = halfLength * scaleFactor; // m_halfLengthScaleFactor;
            float b           = height / 2.0f * scaleFactor;

            std::vector<cvf::Vec3f> nodeCoords;
            tesselator.tesselateEllipsis( a, b, &triangleIndices, &nodeCoords );

            cvf::Mat4d fractureXf = m_rimFractureModel->transformMatrix();
            nodeDisplayCoords     = transformToFractureDisplayCoords( nodeCoords, fractureXf, *displayCoordTransform );
        }

        if ( triangleIndices.empty() || nodeDisplayCoords.empty() )
        {
            return nullptr;
        }

        cvf::ref<cvf::DrawableGeo> geo = buildDrawableGeoFromTriangles( triangleIndices, nodeDisplayCoords );
        CVF_ASSERT( geo.notNull() );

        cvf::ref<cvf::Part> surfacePart = new cvf::Part( 0, "FractureModelSurfacePart_ellipse" );
        surfacePart->setDrawable( geo.p() );
        surfacePart->setSourceInfo( new RivObjectSourceInfo( m_rimFractureModel ) );

        cvf::Color3f fractureColor = cvf::Color3f::RED;

        caf::SurfaceEffectGenerator surfaceGen( fractureColor, caf::PO_1 );
        cvf::ref<cvf::Effect>       eff = surfaceGen.generateCachedEffect();
        surfacePart->setEffect( eff.p() );

        return surfacePart;
    }

    return nullptr;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
std::vector<cvf::Vec3f>
    RivFractureModelPartMgr::transformToFractureDisplayCoords( const std::vector<cvf::Vec3f>&    coordinatesVector,
                                                               cvf::Mat4d                        m,
                                                               const caf::DisplayCoordTransform& displayCoordTransform )
{
    std::vector<cvf::Vec3f> polygonInDisplayCoords;
    polygonInDisplayCoords.reserve( coordinatesVector.size() );

    for ( const cvf::Vec3f& v : coordinatesVector )
    {
        cvf::Vec3d vd( v );
        vd.transformPoint( m );
        cvf::Vec3d displayCoordsDouble = displayCoordTransform.transformToDisplayCoord( vd );
        polygonInDisplayCoords.push_back( cvf::Vec3f( displayCoordsDouble ) );
    }

    return polygonInDisplayCoords;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
cvf::ref<cvf::DrawableGeo>
    RivFractureModelPartMgr::buildDrawableGeoFromTriangles( const std::vector<cvf::uint>&  triangleIndices,
                                                            const std::vector<cvf::Vec3f>& nodeCoords )
{
    CVF_ASSERT( triangleIndices.size() > 0 );
    CVF_ASSERT( nodeCoords.size() > 0 );

    cvf::ref<cvf::DrawableGeo> geo = new cvf::DrawableGeo;

    cvf::ref<cvf::UIntArray>  indices  = new cvf::UIntArray( triangleIndices );
    cvf::ref<cvf::Vec3fArray> vertices = new cvf::Vec3fArray( nodeCoords );

    geo->setVertexArray( vertices.p() );
    geo->addPrimitiveSet( new cvf::PrimitiveSetIndexedUInt( cvf::PT_TRIANGLES, indices.p() ) );
    geo->computeNormals();

    return geo;
}
