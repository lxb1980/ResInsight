/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2011-     Statoil ASA
//  Copyright (C) 2013-     Ceetron Solutions AS
//  Copyright (C) 2011-2012 Ceetron AS
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

#include "RimColorLegend.h"
#include "RimColorLegendItem.h"


CAF_PDM_SOURCE_INIT( RimColorLegend, "ColorLegend" );

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimColorLegend::RimColorLegend()
{
    CAF_PDM_InitObject("ColorLegend", ":/Legend.png", "", "");

    CAF_PDM_InitField(&m_colorLegendName, "ColorLegendName", QString(""), "Color Legend Name", "", "", "");

    CAF_PDM_InitFieldNoDefault( &m_colorLegendItems, "ColorLegendItems", "", "", "", "" );
    m_colorLegendItems.uiCapability()->setUiHidden( true );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimColorLegend::~RimColorLegend()
{
    m_colorLegendItems.deleteAllChildObjects();
}


//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimColorLegend::fieldChangedByUi( const caf::PdmFieldHandle* changedField,
                                            const QVariant&            oldValue,
                                            const QVariant&            newValue )
{
}

