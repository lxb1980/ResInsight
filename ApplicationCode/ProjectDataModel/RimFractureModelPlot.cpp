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
#include "RimFractureModelPlot.h"

#include "RiaDefines.h"
#include "RicfCommandObject.h"

// #include "RigFemPartResultsCollection.h"
// #include "RigFemResultAddress.h"
// #include "RigGeoMechCaseData.h"

#include "RimEclipseCase.h"
#include "RimFractureModel.h"
// #include "RimTools.h"
// #include "RimWbsParameters.h"
// #include "RimWellLogCurveCommonDataSource.h"
// #include "RimWellLogFile.h"

#include "cafPdmBase.h"
#include "cafPdmFieldIOScriptability.h"
#include "cafPdmObject.h"
#include "cafPdmUiComboBoxEditor.h"
#include "cafPdmUiGroup.h"

CAF_PDM_SOURCE_INIT( RimFractureModelPlot, "FractureModelPlot" );

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelPlot::RimFractureModelPlot()
{
    CAF_PDM_InitScriptableObject( "Fracture Model Plot", "", "", "A fracture model plot" );

    CAF_PDM_InitScriptableFieldWithIONoDefault( &m_eclipseCase, "EclipseCase", "Eclipse Case", "", "", "" );
    CAF_PDM_InitScriptableFieldWithIONoDefault( &m_fractureModel, "FractureModel", "Fracture Model", "", "", "" );
    CAF_PDM_InitScriptableFieldWithIONoDefault( &m_timeStep, "TimeStep", "Time Step", "", "", "" );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// void RimFractureModelPlot::applyWbsParametersToExtractor( RigGeoMechWellLogExtractor* extractor )
// {
//     m_wbsParameters->applyWbsParametersToExtractor( extractor );
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// double RimFractureModelPlot::userDefinedValue( const RigWbsParameter& parameter ) const
// {
//     return m_wbsParameters->userDefinedValue( parameter );
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// void RimFractureModelPlot::copyWbsParameters( const RimWbsParameters* wbsParameters )
// {
//     if ( wbsParameters )
//     {
//         *m_wbsParameters = *wbsParameters;
//     }
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::setFractureModel( RimFractureModel* fractureModel )
{
    m_fractureModel = fractureModel;
}

void RimFractureModelPlot::setEclipseCase( RimEclipseCase* eclipseCase )
{
    m_eclipseCase = eclipseCase;
}

void RimFractureModelPlot::setTimeStep( int timeStep )
{
    m_timeStep = timeStep;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::defineUiOrdering( QString uiConfigName, caf::PdmUiOrdering& uiOrdering )
{
    // m_commonDataSource->uiOrdering( RimWellLogCurveCommonDataSource::smoothingUiOrderinglabel(), uiOrdering );

    // caf::PdmUiGroup* parametersGroup = uiOrdering.addNewGroup( "Parameter Sources" );
    // m_wbsParameters->uiOrdering( uiConfigName, *parametersGroup );

    // caf::PdmUiGroup* depthGroup = uiOrdering.addNewGroup( "Depth Axis" );
    // RimWellLogPlot::uiOrderingForDepthAxis( uiConfigName, *depthGroup );

    // caf::PdmUiGroup* titleGroup = uiOrdering.addNewGroup( "Plot Title" );
    // RimWellLogPlot::uiOrderingForAutoName( uiConfigName, *titleGroup );

    // caf::PdmUiGroup* plotLayoutGroup = uiOrdering.addNewGroup( "Plot Layout" );
    // RimPlotWindow::uiOrderingForPlotLayout( uiConfigName, *plotLayoutGroup );

    uiOrdering.skipRemainingFields( true );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::onLoadDataAndUpdate()
{
    // m_wbsParameters->loadDataAndUpdate();
    // RimWellLogPlot::onLoadDataAndUpdate();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::childFieldChangedByUi( const caf::PdmFieldHandle* changedChildField )
{
    // if ( changedChildField == &m_commonDataSource )
    // {
    //     applyDataSource();
    // }
    // else if ( changedChildField == &m_wbsParameters )
    // {
    //     this->loadDataAndUpdate();
    // }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::initAfterRead()
{
    // updateCommonDataSource();
    // applyDataSource();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelPlot::applyDataSource()
{
    // m_wbsParameters->setGeoMechCase( dynamic_cast<RimGeoMechCase*>( m_commonDataSource->caseToApply() ) );
    // m_wbsParameters->setWellPath( m_commonDataSource->wellPathToApply() );
    // m_wbsParameters->setTimeStep( m_commonDataSource->timeStepToApply() );
    this->updateConnectedEditors();
}
