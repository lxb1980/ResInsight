/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2020     Equinor ASA
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

#include "cvfMatrix4.h"
#include "cvfObject.h"
#include "cvfVector3.h"

#include "cafPdmPointer.h"

#include <QString>
#include <vector>

namespace cvf
{
class ModelBasicList;
class DrawableGeo;
class Part;
class Color3f;
class ScalarMapper;
class Transform;
} // namespace cvf

namespace caf
{
class DisplayCoordTransform;
}

class RimFractureModel;
class RimEclipseView;

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
class RivFractureModelPartMgr : public cvf::Object
{
public:
    RivFractureModelPartMgr( RimFractureModel* fractureModel );
    ~RivFractureModelPartMgr() override;

    void appendGeometryPartsToModel( cvf::ModelBasicList* model, const RimEclipseView& eclView );

    const QString resultInfoText( const RimEclipseView& activeView, cvf::Vec3d domainIntersectionPoint ) const;

private:
    cvf::ref<cvf::Part> createEllipseSurfacePart( const RimEclipseView& activeView );

    static std::vector<cvf::Vec3f>
        transformToFractureDisplayCoords( const std::vector<cvf::Vec3f>&    polygon,
                                          cvf::Mat4d                        m,
                                          const caf::DisplayCoordTransform& displayCoordTransform );

    static cvf::ref<cvf::DrawableGeo> buildDrawableGeoFromTriangles( const std::vector<cvf::uint>&  triangleIndices,
                                                                     const std::vector<cvf::Vec3f>& nodeCoords );

    // static cvf::ref<cvf::Transform> createLocalTransformFromTranslation( const cvf::Vec3d& translation );
    // static void                     addPartAtPositiveAndNegativeTranslation( cvf::ModelBasicList* model,
    //                                                                          cvf::Part*           part,
    //                                                                          const cvf::Vec3d&    translation );

private:
    caf::PdmPointer<RimFractureModel> m_rimFractureModel;
};
