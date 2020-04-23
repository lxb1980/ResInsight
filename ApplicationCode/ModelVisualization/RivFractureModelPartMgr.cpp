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
#include "RigEclipseCaseData.h"
#include "RigFractureModelLogExtractor.h"
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
#include "RivPipeGeometryGenerator.h"

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

    auto ellipsePart = createEllipseSurfacePart( eclView );
    if ( ellipsePart.notNull() ) parts.push_back( ellipsePart.p() );

    // TODO: This is a bit weird..
    RimEclipseCase* eclipseCase = eclView.eclipseCase();
    if ( eclipseCase )
    {
        RigFractureModelLogExtractor extractor( eclipseCase->eclipseCaseData(),
                                                m_rimFractureModel->anchorPosition(),
                                                m_rimFractureModel->thicknessDirection(),
                                                0.0 );

        auto pipePart = createPipeSurfacePart( eclView, extractor.wellPathData() );
        if ( pipePart.notNull() ) parts.push_back( pipePart.p() );
    }
    else
    {
        std::cout << "NO ECLIPSE CASE FOUND!!" << std::endl;
    }

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

cvf::ref<cvf::Part> RivFractureModelPartMgr::createPipeSurfacePart( const RimEclipseView& activeView,
                                                                    const RigWellPath*    wellPath )
{
    auto displayCoordTransform = activeView.displayCoordTransform();
    if ( displayCoordTransform.isNull() ) return nullptr;

    const std::vector<cvf::Vec3d>& wellpathCenterLine = wellPath->m_wellPathPoints;

    if ( wellpathCenterLine.size() < 2 ) return nullptr;

    // TODO: improve this!!
    double wellPathRadius = 10.0; // this->wellPathRadius( characteristicCellSize, wellPathCollection );

    std::vector<cvf::Vec3d> clippedWellPathCenterLine;

    // Generate the well path geometry as a line and pipe structure

    RivPipeGeometryGenerator pipeGeomGenerator;

    pipeGeomGenerator.setRadius( wellPathRadius );
    // TODO: use well path collection??
    pipeGeomGenerator.setCrossSectionVertexCount( 20 ); // wellPathCollection->wellPathCrossSectionVertexCount() );

    double horizontalLengthAlongWellToClipPoint = 0.0;
    size_t idxToFirstVisibleSegment             = 0;
    // if ( wellPathCollection->wellPathClip )
    // {
    //     double maxZClipHeight     = wellPathClipBoundingBox.max().z() + wellPathCollection->wellPathClipZDistance;
    //     clippedWellPathCenterLine = RigWellPath::clipPolylineStartAboveZ( wellpathCenterLine,
    //                                                                       maxZClipHeight,
    //                                                                       &horizontalLengthAlongWellToClipPoint,
    //                                                                       &idxToFirstVisibleSegment );
    // }
    // else
    // {
    clippedWellPathCenterLine = wellpathCenterLine;
    // }

    if ( clippedWellPathCenterLine.size() < 2 ) return nullptr;

    cvf::ref<cvf::Vec3dArray> cvfCoords = new cvf::Vec3dArray( clippedWellPathCenterLine.size() );

    // Scale the centerline coordinates using the Z-scale transform of the grid and correct for the display offset.

    // if ( doFlatten )
    // {
    //     cvf::Vec3d              dummy;
    //     std::vector<cvf::Mat4d> flatningCSs =
    //         RivSectionFlattner::calculateFlatteningCSsForPolyline( clippedWellPathCenterLine,
    //                                                                cvf::Vec3d::Z_AXIS,
    //                                                                {horizontalLengthAlongWellToClipPoint,
    //                                                                 0.0,
    //                                                                 clippedWellPathCenterLine[0].z()},
    //                                                                &dummy );

    //     for ( size_t cIdx = 0; cIdx < cvfCoords->size(); ++cIdx )
    //     {
    //         auto clpoint         = clippedWellPathCenterLine[cIdx].getTransformedPoint( flatningCSs[cIdx] );
    //         ( *cvfCoords )[cIdx] = displayCoordTransform->scaleToDisplaySize( clpoint );
    //     }
    // }
    // else
    // {
    for ( size_t cIdx = 0; cIdx < cvfCoords->size(); ++cIdx )
    {
        ( *cvfCoords )[cIdx] = displayCoordTransform->transformToDisplayCoord( clippedWellPathCenterLine[cIdx] );
    }
    // }

    pipeGeomGenerator.setFirstVisibleSegmentIndex( idxToFirstVisibleSegment );
    pipeGeomGenerator.setPipeCenterCoords( cvfCoords.p() );
    cvf::ref<cvf::DrawableGeo> surfaceDrawable = pipeGeomGenerator.createPipeSurface();
    //    m_centerLineDrawable = pipeGeomGenerator.createCenterLine();

    if ( surfaceDrawable.notNull() )
    {
        cvf::ref<cvf::Part> surfacePart = new cvf::Part;
        surfacePart->setDrawable( surfaceDrawable.p() );

        // RivWellPathSourceInfo* sourceInfo = new RivWellPathSourceInfo( m_rimWellPath, m_pipeGeomGenerator.p() );
        // m_surfacePart->setSourceInfo( sourceInfo );

        caf::SurfaceEffectGenerator surfaceGen( cvf::Color4f( cvf::Color3::RED ), caf::PO_1 );
        cvf::ref<cvf::Effect>       eff = surfaceGen.generateCachedEffect();

        surfacePart->setEffect( eff.p() );
        return surfacePart;
    }

    return nullptr;
}
