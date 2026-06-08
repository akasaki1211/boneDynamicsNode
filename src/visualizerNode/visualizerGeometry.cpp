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

    MVector getVectorPlug(const MObject& node, const MObject& attribute, const MVector& defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull() || plug.numChildren() < 3)
        {
            return defaultValue;
        }

        double x = defaultValue.x;
        double y = defaultValue.y;
        double z = defaultValue.z;

        if (!plug.child(0).getValue(x))
        {
            x = defaultValue.x;
        }

        if (!plug.child(1).getValue(y))
        {
            y = defaultValue.y;
        }

        if (!plug.child(2).getValue(z))
        {
            z = defaultValue.z;
        }

        return MVector(x, y, z);
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
    
    void appendSphere(MPointArray& lineList, double radius, const MMatrix& sphereWorldMatrix, int segments)
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

    void appendCapsule(MPointArray& lineList, double radiusA, double radiusB, double height, const MMatrix& capsuleWorldMatrix, int segments)
    {
        radiusA = std::max(0.0, radiusA);
        radiusB = std::max(0.0, radiusB);
        height = std::max(0.0, height);
        segments = std::max(4, segments);

        const double pi = mathUtils::kPi;
        const double halfHeight = height * 0.5;
        const double doubleSegments = static_cast<double>(segments);

        // body lines
        for (int i = 0; i < 4; ++i)
        {
            const double theta = 2.0 * pi * static_cast<double>(i) / 4.0;
            const double y = radiusA * std::cos(theta);
            const double z = radiusA * std::sin(theta);

            appendLine(lineList, MPoint(-halfHeight, y, z), MPoint(halfHeight, y, z), capsuleWorldMatrix);
        }

        // circle lines
        for (int i = 0; i < segments; ++i)
        {
            const double theta1 = 2.0 * pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = 2.0 * pi * static_cast<double>(i + 1) / doubleSegments;

            const double a = radiusA * std::cos(theta1);
            const double b = radiusA * std::sin(theta1);
            const double c = radiusA * std::cos(theta2);
            const double d = radiusA * std::sin(theta2);

            appendLine(lineList, MPoint(halfHeight, a, b), MPoint(halfHeight, c, d), capsuleWorldMatrix);
            appendLine(lineList, MPoint(-halfHeight, a, b), MPoint(-halfHeight, c, d), capsuleWorldMatrix);
        }

        // hemisphere
        for (int i = 0; i < segments / 2; ++i)
        {
            const double theta1 = 2.0 * pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = 2.0 * pi * static_cast<double>(i + 1) / doubleSegments;

            const double a = radiusA * std::cos(theta1);
            const double b = radiusA * std::sin(theta1);
            const double c = radiusA * std::cos(theta2);
            const double d = radiusA * std::sin(theta2);

            // XY
            appendLine(lineList, MPoint(b + halfHeight, a, 0.0), MPoint(d + halfHeight, c, 0.0), capsuleWorldMatrix);

            // XZ
            appendLine(lineList, MPoint(b + halfHeight, 0.0, a), MPoint(d + halfHeight, 0.0, c), capsuleWorldMatrix);

            // XY
            appendLine(lineList, MPoint(-b - halfHeight, a, 0.0), MPoint(-d - halfHeight, c, 0.0), capsuleWorldMatrix);

            // XZ
            appendLine(lineList, MPoint(-b - halfHeight, 0.0, a), MPoint(-d - halfHeight, 0.0, c), capsuleWorldMatrix);
            
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