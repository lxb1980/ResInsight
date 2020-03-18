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

#include "RicNewFractureModelPlotFeature.h"

#include "RiaColorTables.h"
#include "RiaLogging.h"

//#include "RicNewWellLogCurveExtractionFeature.h"
// #include "RicNewWellLogFileCurveFeature.h"
#include "RicWellLogTools.h"
#include "WellLogCommands/RicNewWellLogPlotFeatureImpl.h"

// #include "RigFemPartResultsCollection.h"
// #include "RigFemResultAddress.h"
// #include "RigGeoMechCaseData.h"
// #include "RimGeoMechCase.h"
// #include "RimGeoMechView.h"
#include "RigWellPath.h"
#include "RimEclipseView.h"
#include "RimFractureModel.h"
#include "RimFractureModelPlot.h"
#include "RimProject.h"
// #include "RimWellLogExtractionCurve.h"
// #include "RimWellLogFile.h"
// #include "RimWellLogFileChannel.h"
// #include "RimWellLogFileCurve.h"
// #include "RimWellLogPlotCollection.h"
#include "RimWellLogTrack.h"
// #include "RimWellLogWbsCurve.h"
// #include "RimWellMeasurement.h"
// #include "RimWellMeasurementCollection.h"
// #include "RimWellMeasurementCurve.h"
#include "RimFractureModelPlotCollection.h"
#include "RimMainPlotCollection.h"
#include "RimWellPath.h"
#include "RimWellPathCollection.h"

#include "RiaGuiApplication.h"
#include "RicWellLogTools.h"
#include "RiuPlotMainWindowTools.h"

#include "RiaApplication.h"

#include "cafProgressInfo.h"
#include "cafSelectionManager.h"
#include "cvfAssert.h"
#include "cvfMath.h"

#include <QAction>
#include <QDateTime>
#include <QString>

#include <algorithm>

CAF_CMD_SOURCE_INIT( RicNewFractureModelPlotFeature, "RicNewFractureModelPlotFeature" );

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelPlot*
    RicNewFractureModelPlotFeature::createPlot( RimEclipseCase* eclipseCase, RimFractureModel* fractureModel, int timeStep )

{
    caf::ProgressInfo progInfo( 100, "Creating Fracture Model Plot" );

    RimFractureModelPlot* plot = createFractureModelPlot( true, "Fracture Model" );
    plot->setEclipseCase( eclipseCase );
    plot->setFractureModel( fractureModel );
    plot->setTimeStep( timeStep );

    {
        // auto task = progInfo.task( "Creating formation track", 2 );
        // createFormationTrack( plot, wellPath, eclipseCase );
    }

    {
        auto task = progInfo.task( "Creating parameters track", 15 );
        createParametersTrack( plot, fractureModel, eclipseCase, timeStep );
    }

    {
        auto task = progInfo.task( "Updating all tracks", 5 );

        plot->nameConfig()->setAutoNameTags( true, true, true, true, true );
        plot->setPlotTitleVisible( true );
        plot->setLegendsVisible( true );
        plot->setLegendsHorizontal( true );
        plot->setDepthType( RiaDefines::TRUE_VERTICAL_DEPTH_RKB );
        plot->setAutoScaleDepthEnabled( true );

        // RicNewWellLogPlotFeatureImpl::updateAfterCreation( plot );
        plot->loadDataAndUpdate();
    }

    RiuPlotMainWindowTools::selectAsCurrentItem( plot );

    // Make sure the summary plot window is visible
    RiuPlotMainWindowTools::showPlotMainWindow();

    return plot;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
bool RicNewFractureModelPlotFeature::isCommandEnabled()
{
    Rim3dView* view = RiaApplication::instance()->activeReservoirView();
    if ( !view ) return false;
    RimEclipseView* eclipseView = dynamic_cast<RimEclipseView*>( view );
    return eclipseView != nullptr;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RicNewFractureModelPlotFeature::onActionTriggered( bool isChecked )
{
    RimFractureModel* fractureModel = caf::SelectionManager::instance()->selectedItemAncestorOfType<RimFractureModel>();
    if ( !fractureModel ) return;

    Rim3dView* view = RiaApplication::instance()->activeReservoirView();
    if ( !view ) return;

    RimEclipseView* eclipseView = dynamic_cast<RimEclipseView*>( view );
    if ( !eclipseView ) return;

    RimEclipseCase* eclipseCase = eclipseView->eclipseCase();
    if ( !eclipseCase ) return;

    createPlot( eclipseCase, fractureModel, view->currentTimeStep() );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RicNewFractureModelPlotFeature::setupActionLook( QAction* actionToSetup )
{
    actionToSetup->setText( "New Fracture Model Plot" );
    // actionToSetup->setIcon( QIcon( ":/WellBoreStability16x16.png" ) );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// void RicNewFractureModelPlotFeature::createFormationTrack( RimFractureModelPlot* plot,
//                                                            RimFractureModel*          wellPath,
//                                                            RimEclipseCase*       geoMechCase )
// {
//     RimWellLogTrack* formationTrack = RicNewWellLogPlotFeatureImpl::createWellLogPlotTrack( false, "Formations", plot
//     ); formationTrack->setFormationWellPath( wellPath ); formationTrack->setFormationCase( geoMechCase );
//     formationTrack->setAnnotationType( RiuPlotAnnotationTool::FORMATION_ANNOTATIONS );
//     formationTrack->setVisibleXRange( 0.0, 0.0 );
//     formationTrack->setColSpan( RimPlot::ONE );
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RicNewFractureModelPlotFeature::createParametersTrack( RimFractureModelPlot* plot,
                                                            RimFractureModel*     fractureModel,
                                                            RimEclipseCase*       eclipseCase,
                                                            int                   timeStep )
{
    RimWellLogTrack* paramCurvesTrack =
        RicNewWellLogPlotFeatureImpl::createWellLogPlotTrack( false, "WBS Parameters", plot );
    paramCurvesTrack->setColSpan( RimPlot::TWO );
    paramCurvesTrack->setVisibleXRange( 0.0, 2.0 );
    paramCurvesTrack->setAutoScaleXEnabled( true );
    paramCurvesTrack->setTickIntervals( 1.0, 0.2 );
    paramCurvesTrack->setXAxisGridVisibility( RimWellLogPlot::AXIS_GRID_MAJOR_AND_MINOR );
    // paramCurvesTrack->setFormationWellPath( wellPath );
    // paramCurvesTrack->setFormationCase( geoMechCase );
    paramCurvesTrack->setAnnotationType( RiuPlotAnnotationTool::CURVE_ANNOTATIONS );
    paramCurvesTrack->setShowRegionLabels( true );
    paramCurvesTrack->setShowWindow( true );
    // std::set<RigWbsParameter> parameters = RigWbsParameter::allParameters();

    // caf::ColorTable                             colors     = RiaColorTables::contrastCategoryPaletteColors();
    // std::vector<RiuQwtPlotCurve::LineStyleEnum> lineStyles = {RiuQwtPlotCurve::STYLE_SOLID,
    //                                                           RiuQwtPlotCurve::STYLE_DASH,
    //                                                           RiuQwtPlotCurve::STYLE_DASH_DOT};

    // size_t i = 0;
    // for ( const RigWbsParameter& param : parameters )
    // {
    //     if ( !param.hasExternalSource() || param == RigWbsParameter::waterDensity() ) continue;

    //     RigFemResultAddress        resAddr( RIG_WELLPATH_DERIVED, param.name().toStdString(), "" );
    //     RimWellLogExtractionCurve* curve =
    //         RicWellLogTools::addWellLogExtractionCurve( paramCurvesTrack, geoMechCase, nullptr, wellPath, nullptr,
    //         -1, false, false );
    //     curve->setEclipseResultAddress( resAddr );
    //     curve->setCurrentTimeStep( timeStep );
    //     curve->setColor( colors.cycledColor3f( i ) );
    //     curve->setLineStyle( lineStyles[i % lineStyles.size()] );
    //     curve->setLineThickness( 2 );
    //     curve->loadDataAndUpdate( false );
    //     curve->setCustomName( param.name() );
    //     i++;
    // }
    paramCurvesTrack->setAutoScaleXEnabled( true );
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// void RicNewFractureModelPlotFeature::createStabilityCurvesTrack( RimFractureModelPlot* plot,
//                                                                  RimFractureModel*     wellPath,
//                                                                  RimEclipseCase*       geoMechCase,
//                                                                  int                   timeStep )
// {
//     RimWellLogTrack* stabilityCurvesTrack = RicNewWellLogPlotFeatureImpl::createWellLogPlotTrack( false,
//                                                                                                   "Stability Curves",

//                                                                                                   plot );
//     stabilityCurvesTrack->setVisibleXRange( 0.0, 2.5 );
//     stabilityCurvesTrack->setColSpan( RimPlot::THREE );
//     stabilityCurvesTrack->setAutoScaleXEnabled( true );
//     stabilityCurvesTrack->setTickIntervals( 1.0, 0.2 );
//     stabilityCurvesTrack->setXAxisGridVisibility( RimWellLogPlot::AXIS_GRID_MAJOR_AND_MINOR );
//     stabilityCurvesTrack->setFormationWellPath( wellPath );
//     stabilityCurvesTrack->setFormationCase( geoMechCase );
//     stabilityCurvesTrack->setAnnotationType( RiuPlotAnnotationTool::FORMATION_ANNOTATIONS );
//     stabilityCurvesTrack->setAnnotationDisplay( RiuPlotAnnotationTool::LIGHT_LINES );
//     stabilityCurvesTrack->setShowRegionLabels( false );

//     std::vector<QString> resultNames = RiaDefines::wbsDerivedResultNames();

//     std::vector<cvf::Color3f> colors = {cvf::Color3f::BLUE,
//                                         cvf::Color3f::BROWN,
//                                         cvf::Color3f::RED,
//                                         cvf::Color3f::PURPLE,
//                                         cvf::Color3f::DARK_GREEN,
//                                         cvf::Color3f::OLIVE};

//     std::vector<RiuQwtPlotCurve::LineStyleEnum> lineStyles( resultNames.size(), RiuQwtPlotCurve::STYLE_SOLID );
//     lineStyles.back() = RiuQwtPlotCurve::STYLE_DASH;

//     for ( size_t i = 0; i < resultNames.size(); ++i )
//     {
//         const QString&      resultName = resultNames[i];
//         RigFemResultAddress resAddr( RIG_WELLPATH_DERIVED, resultName.toStdString(), "" );
//         RimWellLogWbsCurve* curve =
//             RicWellLogTools::addWellLogWbsCurve( stabilityCurvesTrack, geoMechCase, nullptr, wellPath, -1, false,
//             false );
//         curve->setEclipseResultAddress( resAddr );
//         curve->setCurrentTimeStep( timeStep );
//         curve->setAutoNameComponents( false, true, false, false, false );
//         curve->setColor( colors[i % colors.size()] );
//         curve->setLineStyle( lineStyles[i] );
//         curve->setLineThickness( 2 );
//         curve->loadDataAndUpdate( false );
//         curve->setSmoothCurve( true );
//         curve->setSmoothingThreshold( 0.002 );
//     }

//     stabilityCurvesTrack->setAutoScaleXEnabled( true );
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
// void RicNewFractureModelPlotFeature::createAnglesTrack( RimFractureModelPlot* plot,
//                                                         RimFractureModel*              wellPath,
//                                                         RimEclipseCase*           geoMechCase,
//                                                         int                       timeStep )
// {
//     RimWellLogTrack* wellPathAnglesTrack =
//         RicNewWellLogPlotFeatureImpl::createWellLogPlotTrack( false, "Well Path Angles", plot );
//     double               minValue = 360.0, maxValue = 0.0;
//     const double         angleIncrement = 90.0;
//     std::vector<QString> resultNames    = RiaDefines::wbsAngleResultNames();

//     std::vector<cvf::Color3f> colors = {cvf::Color3f::GREEN, cvf::Color3f::DARK_ORANGE};

//     std::vector<RiuQwtPlotCurve::LineStyleEnum> lineStyles = {RiuQwtPlotCurve::STYLE_DASH,
//     RiuQwtPlotCurve::STYLE_SOLID};

//     for ( size_t i = 0; i < resultNames.size(); ++i )
//     {
//         const QString&             resultName = resultNames[i];
//         RigFemResultAddress        resAddr( RIG_WELLPATH_DERIVED, resultName.toStdString(), "" );
//         RimWellLogExtractionCurve* curve = RicWellLogTools::addWellLogExtractionCurve( wellPathAnglesTrack,
//                                                                                        geoMechCase,
//                                                                                        nullptr,
//                                                                                        wellPath,
//                                                                                        nullptr,
//                                                                                        -1,
//                                                                                        false,
//                                                                                        false );
//         curve->setEclipseResultAddress( resAddr );
//         curve->setCurrentTimeStep( timeStep );
//         curve->setCustomName( resultName );

//         curve->setColor( colors[i % colors.size()] );
//         curve->setLineStyle( lineStyles[i % lineStyles.size()] );
//         curve->setLineThickness( 2 );

//         curve->loadDataAndUpdate( false );

//         double actualMinValue = minValue, actualMaxValue = maxValue;
//         curve->xValueRangeInQwt( &actualMinValue, &actualMaxValue );
//         while ( maxValue < actualMaxValue )
//         {
//             maxValue += angleIncrement;
//         }
//         while ( minValue > actualMinValue )
//         {
//             minValue -= angleIncrement;
//         }
//         maxValue = cvf::Math::clamp( maxValue, angleIncrement, 720.0 );
//         minValue = cvf::Math::clamp( minValue, 0.0, maxValue - 90.0 );
//     }
//     wellPathAnglesTrack->setColSpan( RimPlot::TWO );
//     wellPathAnglesTrack->setVisibleXRange( minValue, maxValue );
//     wellPathAnglesTrack->setTickIntervals( 180.0, 45.0 );
//     wellPathAnglesTrack->setXAxisGridVisibility( RimWellLogPlot::AXIS_GRID_MAJOR_AND_MINOR );
//     wellPathAnglesTrack->setFormationWellPath( wellPath );
//     wellPathAnglesTrack->setFormationCase( geoMechCase );
//     wellPathAnglesTrack->setAnnotationType( RiuPlotAnnotationTool::FORMATION_ANNOTATIONS );
//     wellPathAnglesTrack->setAnnotationDisplay( RiuPlotAnnotationTool::LIGHT_LINES );
//     wellPathAnglesTrack->setShowRegionLabels( false );
// }

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelPlot* RicNewFractureModelPlotFeature::createFractureModelPlot( bool           showAfterCreation,
                                                                               const QString& plotDescription )

{
    RimFractureModelPlotCollection* fractureModelPlotColl = fractureModelPlotCollection();
    CVF_ASSERT( fractureModelPlotColl );

    // Make sure the summary plot window is created
    RiaGuiApplication::instance()->getOrCreateMainPlotWindow();

    RimFractureModelPlot* plot = new RimFractureModelPlot();
    plot->setAsPlotMdiWindow();

    fractureModelPlotColl->fractureModelPlots().push_back( plot );

    if ( !plotDescription.isEmpty() )
    {
        plot->nameConfig()->setCustomName( plotDescription );
    }
    else
    {
        plot->nameConfig()->setCustomName(
            QString( "Fracture Model Plot %1" ).arg( fractureModelPlotCollection()->fractureModelPlots.size() ) );
    }

    if ( showAfterCreation )
    {
        RiaGuiApplication::instance()->getOrCreateAndShowMainPlotWindow();
    }

    return plot;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimFractureModelPlotCollection* RicNewFractureModelPlotFeature::fractureModelPlotCollection()
{
    RimProject* project = RiaApplication::instance()->project();
    CVF_ASSERT( project );

    RimMainPlotCollection* mainPlotColl = project->mainPlotCollection();
    CVF_ASSERT( mainPlotColl );

    RimFractureModelPlotCollection* fractureModelPlotColl = mainPlotColl->fractureModelPlotCollection();
    CVF_ASSERT( fractureModelPlotColl );

    return mainPlotColl->fractureModelPlotCollection();
}
