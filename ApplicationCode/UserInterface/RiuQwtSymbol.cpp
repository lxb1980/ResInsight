/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2015-     Equinor ASA
//  Copyright (C) 2015-     Ceetron Solutions AS
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

#include "RiuQwtSymbol.h"

#include <QPainter>

//--------------------------------------------------------------------------------------------------
/// Internal class to support labels on symbols
//--------------------------------------------------------------------------------------------------
RiuQwtSymbol::RiuQwtSymbol(PointSymbolEnum riuStyle, const QString& label, LabelPosition labelPosition)
    : QwtSymbol(QwtSymbol::NoSymbol), m_label(label), m_labelPosition(labelPosition)
{ 
    QwtSymbol::Style style = QwtSymbol::NoSymbol;

    switch (riuStyle)
    {
    case SYMBOL_ELLIPSE:
        style = QwtSymbol::Ellipse;
        break;
    case SYMBOL_RECT:
        style = QwtSymbol::Rect;
        break;
    case SYMBOL_DIAMOND:
        style = QwtSymbol::Diamond;
        break;
    case SYMBOL_TRIANGLE:
        style = QwtSymbol::Triangle;
        break;
    case SYMBOL_CROSS:
        style = QwtSymbol::Cross;
        break;
    case SYMBOL_XCROSS:
        style = QwtSymbol::XCross;
        break;
    case SYMBOL_DOWN_TRIANGLE:
        style = QwtSymbol::DTriangle;
        break;
    case SYMBOL_LEFT_TRIANGLE:
        style = QwtSymbol::Path;
        {
            QPainterPath path;
            path.moveTo(0, 0);
            path.lineTo(-10, 0);
            path.lineTo(0, -10);
            path.lineTo(0, 0);
            setPath(path);
            setPinPoint(QPointF(0, 0));
        }
        break;
    case SYMBOL_RIGHT_TRIANGLE:
        style = QwtSymbol::Path;
        {
            QPainterPath path;
            path.moveTo(0, 0);
            path.lineTo(10, 0);
            path.lineTo(0, -10);
            path.lineTo(0, 0);
            setPath(path);
            setPinPoint(QPointF(0, 0));
        }
        break;
    default:
        break;
    }
    setStyle(style);
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RiuQwtSymbol::renderSymbols(QPainter *painter, const QPointF *points, int numPoints) const
{
    QwtSymbol::renderSymbols(painter, points, numPoints);

    if (!m_label.isEmpty())
    {
        for (int i = 0; i < numPoints; i++)
        {
            auto position = points[i];
            renderSymbolLabel(painter, position);
        }
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RiuQwtSymbol::renderSymbolLabel(QPainter *painter, const QPointF& position) const
{
    int symbolWidth = this->size().width();
    int labelWidth = painter->fontMetrics().width(m_label);
    if (m_labelPosition == LabelAboveSymbol)
    {
        painter->drawText(position.x() - labelWidth / 2, position.y() - 5, m_label);
    }
    else if (m_labelPosition == LabelRightOfSymbol)
    {
        painter->drawText(position.x() + symbolWidth + 3, position.y(), m_label);
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RiuQwtSymbol::setLabelPosition(LabelPosition labelPosition)
{
    m_labelPosition = labelPosition;
}

