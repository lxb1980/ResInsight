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

#include "RimFractureModelCurve.h"

#include "RigWellLogCurveData.h"
#include "RigWellPath.h"

#include "RimProject.h"
#include "RimTools.h"
#include "RimWellLogFile.h"
#include "RimWellLogPlot.h"
#include "RimWellLogTrack.h"
#include "RimWellMeasurement.h"
#include "RimWellMeasurementCollection.h"
#include "RimWellMeasurementFilter.h"
#include "RimWellPath.h"
#include "RimWellPathCollection.h"
#include "RimWellPlotTools.h"
#include "RimWellRftPlot.h"

#include "RiuQwtPlotCurve.h"
#include "RiuQwtPlotWidget.h"

#include "RiaApplication.h"
#include "RiaPreferences.h"

#include "cafPdmUiTreeOrdering.h"

#include <QFileInfo>
#include <QMessageBox>

CAF_PDM_SOURCE_INIT( RimFractureModelCurve, "FractureModelCurve" );

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelCurve::RimFractureModelCurve()
{
    CAF_PDM_InitObject( "Fracture Model Curve", "", "", "" );

    CAF_PDM_InitFieldNoDefault( &m_wellPath, "CurveWellPath", "Well Path", "", "", "" );
    m_wellPath.uiCapability()->setUiTreeChildrenHidden( true );

    // CAF_PDM_InitFieldNoDefault( &m_measurementKind, "CurveMeasurementKind", "Measurement Kind", "", "", "" );
    // m_measurementKind.uiCapability()->setUiTreeChildrenHidden( true );
    // m_measurementKind.uiCapability()->setUiHidden( true );

    m_wellPath = nullptr;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelCurve::~RimFractureModelCurve()
{
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelCurve::onLoadDataAndUpdate( bool updateParentPlot )
{
    this->RimPlotCurve::updateCurvePresentation( updateParentPlot );

    if ( isCurveVisible() )
    {
        RimWellLogPlot* wellLogPlot;
        firstAncestorOrThisOfType( wellLogPlot );
        CVF_ASSERT( wellLogPlot );

        // RimWellPathCollection* wellPathCollection = RimTools::wellPathCollection();
        // if ( m_wellPath && !m_measurementKind().isEmpty() && wellPathCollection )
        // {

        // Extract the values for this measurement kind
        std::vector<double> values;
        std::vector<double> measuredDepthValues;

        // for ( auto& measurement : measurements )
        // {
        //     if ( measurement->kind() == measurementKind() )
        //     {
        //         values.push_back( measurement->value() );
        //         measuredDepthValues.push_back( measurement->MD() );
        //     }
        // }

        if ( values.size() == measuredDepthValues.size() )
        {
            // RigWellPath* rigWellPath = m_wellPath->wellPathGeometry();
            // if ( rigWellPath )
            // {
            //     std::vector<double> trueVerticalDepthValues;

            //     // for ( double measuredDepthValue : measuredDepthValues )
            //     // {
            //     //     trueVerticalDepthValues.push_back(
            //     //         -rigWellPath->interpolatedPointAlongWellPath( measuredDepthValue ).z() );
            //     // }

            //     this->setValuesWithMdAndTVD( values,
            //                                  measuredDepthValues,
            //                                  trueVerticalDepthValues,
            //                                  m_wellPath->wellPathGeometry()->rkbDiff(),
            //                                  RiaDefines::UNIT_METER,
            //                                  false );
            // }
            // else
            // {
            this->setValuesAndDepths( values, measuredDepthValues, RiaDefines::MEASURED_DEPTH, 0.0, RiaDefines::UNIT_METER, false );
        }

        if ( m_isUsingAutoName )
        {
            m_qwtPlotCurve->setTitle( createCurveAutoName() );
        }

        // setSymbol( getSymbolForMeasurementKind( m_measurementKind() ) );
        // setColor( getColorForMeasurementKind( measurementKind() ) );
        // setSymbolEdgeColor( getColorForMeasurementKind( measurementKind() ) );
        // setLineStyle( RiuQwtPlotCurve::STYLE_NONE );

        RiaDefines::DepthUnitType displayUnit = RiaDefines::UNIT_METER;
        if ( wellLogPlot )
        {
            displayUnit = wellLogPlot->depthUnit();
        }

        RiaDefines::DepthTypeEnum depthType = RiaDefines::MEASURED_DEPTH;
        if ( wellLogPlot && this->curveData()->availableDepthTypes().count( wellLogPlot->depthType() ) )
        {
            depthType = wellLogPlot->depthType();
        }

        m_qwtPlotCurve->setSamples( this->curveData()->xPlotValues().data(),
                                    this->curveData()->depthPlotValues( depthType, displayUnit ).data(),
                                    static_cast<int>( this->curveData()->xPlotValues().size() ) );
        m_qwtPlotCurve->setLineSegmentStartStopIndices( this->curveData()->polylineStartStopIndices() );
    }

    if ( updateParentPlot )
    {
        updateZoomInParentPlot();
    }

    if ( m_parentQwtPlot )
    {
        m_parentQwtPlot->replot();
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelCurve::setWellPath( RimWellPath* wellPath )
{
    m_wellPath = wellPath;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimWellPath* RimFractureModelCurve::wellPath() const
{
    return m_wellPath;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelCurve::fieldChangedByUi( const caf::PdmFieldHandle* changedField,
                                              const QVariant&            oldValue,
                                              const QVariant&            newValue )
{
    RimWellLogCurve::fieldChangedByUi( changedField, oldValue, newValue );

    // if ( changedField == &m_wellPath || changedField == &m_measurementKind )
    // {
    this->loadDataAndUpdate( true );
    //    }
    if ( m_parentQwtPlot ) m_parentQwtPlot->replot();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelCurve::defineUiOrdering( QString uiConfigName, caf::PdmUiOrdering& uiOrdering )
{
    RimPlotCurve::updateOptionSensitivity();

    caf::PdmUiGroup* curveDataGroup = uiOrdering.addNewGroup( "Curve Data" );
    curveDataGroup->add( &m_wellPath );

    caf::PdmUiGroup* appearanceGroup = uiOrdering.addNewGroup( "Appearance" );
    RimPlotCurve::appearanceUiOrdering( *appearanceGroup );

    caf::PdmUiGroup* nameGroup = uiOrdering.addNewGroup( "Curve Name" );
    nameGroup->add( &m_showLegend );
    RimPlotCurve::curveNameUiOrdering( *nameGroup );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RimFractureModelCurve::defineUiTreeOrdering( caf::PdmUiTreeOrdering& uiTreeOrdering, QString uiConfigName /*= ""*/ )
{
    uiTreeOrdering.skipRemainingChildren( true );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QList<caf::PdmOptionItemInfo>
    RimFractureModelCurve::calculateValueOptions( const caf::PdmFieldHandle* fieldNeedingOptions, bool* useOptionsOnly )
{
    QList<caf::PdmOptionItemInfo> options;

    options = RimWellLogCurve::calculateValueOptions( fieldNeedingOptions, useOptionsOnly );
    if ( options.size() > 0 ) return options;

    if ( fieldNeedingOptions == &m_wellPath )
    {
        RimTools::wellPathOptionItems( &options );
    }

    return options;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QString RimFractureModelCurve::createCurveAutoName()
{
    if ( m_wellPath )
    {
        // TODO: should take name from fracture?
        return "Fracture model";
    }

    return "Empty curve";
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QString RimFractureModelCurve::wellLogChannelUiName() const
{
    // Does not really make sense for measurement curve.
    return QString( "" );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QString RimFractureModelCurve::wellLogChannelUnits() const
{
    return RiaWellLogUnitTools::noUnitString();
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
QString RimFractureModelCurve::wellName() const
{
    if ( m_wellPath )
    {
        return m_wellPath->name();
    }

    return QString( "" );
}
