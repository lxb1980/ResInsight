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

#pragma once

#include "RiaEclipseUnitTools.h"

#include "RimCheckableNamedObject.h"
#include "RimWellPathComponentInterface.h"

#include "cvfObject.h"
#include "cvfVector3.h"

#include "cafPdmChildField.h"
#include "cafPdmFieldCvfVec3d.h"
#include "cafPdmProxyValueField.h"
#include "cafPdmPtrField.h"

class RimEclipseCase;
class RimWellPath;
class RimModeledWellPath;

//==================================================================================================
///
///
//==================================================================================================
class RimFractureModel : public RimCheckableNamedObject, public RimWellPathComponentInterface
{
    CAF_PDM_HEADER_INIT;

public:
    enum class ExtractionType
    {
        TRUE_VERTICAL_THICKNESS,
        TRUE_STRATIGRAPHIC_THICKNESS,
    };

    RimFractureModel( void );
    ~RimFractureModel( void ) override;

    cvf::Vec3d anchorPosition() const;
    cvf::Vec3d thicknessDirection() const;

    void       fieldChangedByUi( const caf::PdmFieldHandle* changedField, const QVariant& oldValue, const QVariant& newValue ) override;
    cvf::Vec3d fracturePosition() const;

    // RimWellPathCompletionsInterface overrides.
    RiaDefines::WellPathComponentType componentType() const override;
    QString                           componentLabel() const override;
    QString                           componentTypeLabel() const override;
    cvf::Color3f                      defaultComponentColor() const override;
    double                            startMD() const override;
    double                            endMD() const override;
    bool                              isEnabled() const override;

    RimWellPath* wellPath() const;

    RimModeledWellPath* thicknessDirectionWellPath() const;
    void                setThicknessDirectionWellPath( RimModeledWellPath* thicknessDirectionWellPath );

protected:
    void defineUiOrdering( QString uiConfigName, caf::PdmUiOrdering& uiOrdering ) override;

private:
    void           updatePositionFromMeasuredDepth();
    void           updateThicknessDirection();
    cvf::Vec3d     calculateTSTDirection() const;
    void           findThicknessTargetPoints( cvf::Vec3d& topPosition, cvf::Vec3d& bottomPosition );
    static QString vecToString( const cvf::Vec3d& vec );
    void           updateThicknessDirectionWellPathName();

protected:
    caf::PdmField<double>                       m_MD;
    caf::PdmField<caf::AppEnum<ExtractionType>> m_extractionType;
    caf::PdmField<cvf::Vec3d>                   m_anchorPosition;
    caf::PdmField<cvf::Vec3d>                   m_thicknessDirection;
    caf::PdmField<double>                       m_boundingBoxVertical;
    caf::PdmField<double>                       m_boundingBoxHorizontal;
    caf::PdmPtrField<RimModeledWellPath*>       m_thicknessDirectionWellPath;
};
