/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) Statoil ASA
//  Copyright (C) Ceetron Solutions AS
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

#include "cvfObject.h"
#include "cvfVector3.h"

#include <vector>

class RigEclipseWellLogExtractor;
class RigEclipseCaseData;
class RigWellPath;
class RigResultAccessor;

namespace cvf
{
class BoundingBox;
} // namespace cvf

//==================================================================================================
///
//==================================================================================================
class RigFractureModelLogExtractor
{
public:
    RigFractureModelLogExtractor( const RigEclipseCaseData* aCase,
                                  const cvf::Vec3d&         position,
                                  const cvf::Vec3d&         direction,
                                  double                    measuredDepth,
                                  const cvf::BoundingBox&   geometryBoundingBox );

    void curveData( const RigResultAccessor* resultAccessor, std::vector<double>* values );

private:
    cvf::cref<RigEclipseCaseData>        m_caseData;
    cvf::ref<RigWellPath>                m_wellPath;
    cvf::ref<RigEclipseWellLogExtractor> m_wellLogExtractor;
};
