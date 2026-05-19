#include "visualizerGeometry.h"
#include "../mathUtils.h"

#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MFnMatrixData.h>
#include <maya/MDrawContext.h>

#include <algorithm> // std::min, std::max
#include <cmath>     // std::sin, std::cos

namespace
{
    void appendLine(MPointArray& lineList, const MPoint& a, const MPoint& b, const MMatrix& worldMatrix)
    {
        lineList.append(a * worldMatrix);
        lineList.append(b * worldMatrix);
    }
}

namespace visualizerGeometry
{
    VisualizerDrawData::VisualizerDrawData() : MUserData(false),
        color(1.0f, 1.0f, 1.0f),
        depthPriority(MHWRender::MRenderItem::sDormantWireDepthPriority)
    {}

    bool getBoolPlug(const MObject& node, const MObject& attribute, bool defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull())
        {
            return defaultValue;
        }

        bool value = defaultValue;
        if (!plug.getValue(value))
        {
            return defaultValue;
        }

        return value;
    }

    /*int getIntPlug(const MObject& node, const MObject& attribute, int defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull())
        {
            return defaultValue;
        }

        int value = defaultValue;
        if (!plug.getValue(value))
        {
            return defaultValue;
        }

        return value;
    }*/

    double getDoublePlug(const MObject& node, const MObject& attribute, double defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull())
        {
            return defaultValue;
        }

        double value = defaultValue;
        if (!plug.getValue(value))
        {
            return defaultValue;
        }

        return value;
    }

    MMatrix getMatrixPlug(const MObject& node, const MObject& attribute, const MMatrix& defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull())
        {
            return defaultValue;
        }

        MObject matrixObject;
        if (!plug.getValue(matrixObject))
        {
            return defaultValue;
        }

        MStatus status;
        MFnMatrixData matrixData(matrixObject, &status);

        if (!status)
        {
            return defaultValue;
        }

        return matrixData.matrix(&status);
    }
    
    void appendRadiusSphere(MPointArray& lineList, double radius, const MMatrix& sphereWorldMatrix, int segments)
    {
        radius = std::max(0.0, radius);
        segments = std::max(4, segments);

        const double pi = mathUtils::kPi;
        const double doubleSegments = static_cast<double>(segments);

        for (int i = 0; i < segments; ++i)
        {
            const double theta1 = 2.0 * pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = 2.0 * pi * static_cast<double>(i + 1) / doubleSegments;

            const double a = radius * std::cos(theta1);
            const double b = radius * std::sin(theta1);
            const double c = radius * std::cos(theta2);
            const double d = radius * std::sin(theta2);

            // XY
            appendLine(lineList, MPoint(a, b, 0.0), MPoint(c, d, 0.0), sphereWorldMatrix);

            // XZ
            appendLine(lineList, MPoint(a, 0.0, b), MPoint(c, 0.0, d), sphereWorldMatrix);
            
            // YZ
            appendLine(lineList, MPoint(0.0, a, b), MPoint(0.0, c, d), sphereWorldMatrix);
        }
    }

    void appendAngleLimitCone(MPointArray& lineList, double angleLimitDegrees, double length, const MMatrix& coneWorldMatrix, int segments)
    {
        length = std::max(0.001, length);
        angleLimitDegrees = std::max(0.0, std::min(360.0, angleLimitDegrees));

        const double pi = mathUtils::kPi;
        const double doubleSegments = static_cast<double>(segments);

        const double halfAngleRadians = angleLimitDegrees * 0.5 * (pi / 180.0);
        const double rawRadius = std::tan(halfAngleRadians) * length;
        const double coneRadius = std::min(std::abs(rawRadius), length * 100.0);

        // circle
        for (int i = 0; i < segments; ++i)
        {
            const double theta1 = 2.0 * pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = 2.0 * pi * static_cast<double>(i + 1) / doubleSegments;

            const double y1 = coneRadius * std::cos(theta1);
            const double z1 = coneRadius * std::sin(theta1);

            const double y2 = coneRadius * std::cos(theta2);
            const double z2 = coneRadius * std::sin(theta2);

            appendLine(lineList, MPoint(length, y1, z1), MPoint(length, y2, z2), coneWorldMatrix);
            //appendLine(lineList, MPoint(0.0, 0.0, 0.0), MPoint(length, y1, z1), coneWorldMatrix); // line
        }

        // cone line
        for (int i = 0; i < 8; ++i)
        {
            const double theta = 2.0 * pi * static_cast<double>(i) / 8.0;
            
            const double y = coneRadius * std::cos(theta);
            const double z = coneRadius * std::sin(theta);

            appendLine(lineList, MPoint(0.0, 0.0, 0.0), MPoint(length, y, z), coneWorldMatrix);
        }
    }
}