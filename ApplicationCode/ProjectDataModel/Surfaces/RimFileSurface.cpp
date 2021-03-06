/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2020-     Equinor ASA
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

#include "RimFileSurface.h"

#include "RifSurfaceReader.h"
#include "RigSurface.h"
#include "RimSurfaceCollection.h"

#include <QFileInfo>

// TODO: Use the alias concept prototyped below when the alias concept for class is ready
// CAF_PDM_SOURCE_INIT( RimFileSurface, "FileSurface", "Surface" );
// CAF_PDM_SOURCE_INIT( <class>       ,  <keyword>   , <alias>);
CAF_PDM_SOURCE_INIT( RimFileSurface, "Surface" );

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFileSurface::RimFileSurface()
{
    CAF_PDM_InitObject( "Surface", ":/ReservoirSurface16x16.png", "", "" );

    CAF_PDM_InitFieldNoDefault( &m_surfaceDefinitionFilePath, "SurfaceFilePath", "File", "", "", "" );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFileSurface::~RimFileSurface()
{
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFileSurface::setSurfaceFilePath( const QString& filePath )
{
    m_surfaceDefinitionFilePath = filePath;
    if ( userDescription().isEmpty() )
    {
        setUserDescription( QFileInfo( filePath ).fileName() );
    }

    clearCachedNativeFileData();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QString RimFileSurface::surfaceFilePath()
{
    return m_surfaceDefinitionFilePath().path();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
bool RimFileSurface::onLoadData()
{
    return updateSurfaceDataFromFile();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFileSurface::fieldChangedByUi( const caf::PdmFieldHandle* changedField,
                                       const QVariant&            oldValue,
                                       const QVariant&            newValue )
{
    RimSurface::fieldChangedByUi( changedField, oldValue, newValue );

    if ( changedField == &m_surfaceDefinitionFilePath )
    {
        clearCachedNativeFileData();
        updateSurfaceDataFromFile();

        RimSurfaceCollection* surfColl;
        this->firstAncestorOrThisOfTypeAsserted( surfColl );
        surfColl->updateViews( {this} );
    }
}

//--------------------------------------------------------------------------------------------------
/// Returns false for fatal failure
//--------------------------------------------------------------------------------------------------
bool RimFileSurface::updateSurfaceDataFromFile()
{
    bool result = true;
    if ( m_vertices.empty() )
    {
        result = loadDataFromFile();
    }

    std::vector<cvf::Vec3d> vertices{m_vertices};
    std::vector<unsigned>   tringleIndices{m_tringleIndices};

    auto surface = new RigSurface;
    if ( !vertices.empty() && !tringleIndices.empty() )
    {
        RimSurface::applyDepthOffsetIfNeeded( &vertices );

        surface->setTriangleData( tringleIndices, vertices );
    }

    setSurfaceData( surface );

    return result;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFileSurface::clearCachedNativeFileData()
{
    m_vertices.clear();
    m_tringleIndices.clear();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
bool RimFileSurface::loadDataFromFile()
{
    std::vector<cvf::Vec3d> vertices;
    std::vector<unsigned>   tringleIndices;

    QString filePath = this->surfaceFilePath();
    if ( filePath.endsWith( "ptl", Qt::CaseInsensitive ) )
    {
        auto surface = RifSurfaceReader::readPetrelFile( filePath );

        vertices       = surface.first;
        tringleIndices = surface.second;
    }
    else if ( filePath.endsWith( "ts", Qt::CaseInsensitive ) )
    {
        auto surface = RifSurfaceReader::readGocadFile( filePath );

        vertices       = surface.first;
        tringleIndices = surface.second;
    }

    m_vertices       = vertices;
    m_tringleIndices = tringleIndices;

    if ( vertices.empty() || tringleIndices.empty() ) return false;

    return true;
}
