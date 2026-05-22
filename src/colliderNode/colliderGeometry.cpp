#include "colliderGeometry.h"
#include "../mathUtils.h"

#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MDrawContext.h>

#include <algorithm> // std::min, std::max
#include <cmath>     // std::sin, std::cos

namespace colliderGeometry
{
    ColliderDrawData::ColliderDrawData() : MUserData(false), 
        color(1.0f, 1.0f, 1.0f), 
        depthPriority(MHWRender::MRenderItem::sDormantWireDepthPriority)
    {}

    double getDistancePlugAsCentimeters(const MObject& node, const MObject& attribute, double defaultValue)
    {
        MPlug plug(node, attribute);
        if (plug.isNull())
        {
            return defaultValue;
        }

        MDistance value;
        MStatus status = plug.getValue(value);
        if (!status)
        {
            return defaultValue;
        }

        return value.asCentimeters();
    }

    MBoundingBox makeSphereBoundingBox(double radius)
    {
        radius = std::max(0.0, radius);

        MPoint corner1(-radius, -radius, -radius);
        MPoint corner2(radius, radius, radius);

        return MBoundingBox(corner1, corner2);
    }

    MBoundingBox makeCapsuleBoundingBox(double radiusA, double radiusB, double height)
    {
        radiusA = std::max(0.0, radiusA);
        radiusB = std::max(0.0, radiusB);
        height = std::max(0.0, height);

        const double halfHeight = height * 0.5;
        const double maxRadius = std::max(radiusA, radiusB);

        MPoint corner1(-maxRadius, -halfHeight - radiusB, -maxRadius);
        MPoint corner2(maxRadius, halfHeight + radiusA, maxRadius);

        return MBoundingBox(corner1, corner2);
    }
    
    MBoundingBox makePlaneBoundingBox(double size)
    {
        size = std::max(0.0, size);

        MPoint corner1(-size, 0.0, -size);
        MPoint corner2(size, size, size);

        return MBoundingBox(corner1, corner2);
    }

    void appendWireSphere(MPointArray& lineList, double radius, int segments)
    {
        radius = std::max(0.0, radius);
        segments = std::max(4, segments);

        const double pi = mathUtils::kPi;
        const double doubleSegments = static_cast<double>(segments);

        for (int i = 0; i < segments; ++i)
        {
            const double theta1 = pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = pi * static_cast<double>(i + 1) / doubleSegments;

            for (int j = 0; j < segments; ++j)
            {
                const double phi1 = 2.0 * pi * static_cast<double>(j) / doubleSegments;
                const double phi2 = 2.0 * pi * static_cast<double>(j + 1) / doubleSegments;

                const double x1 = radius * std::sin(theta1) * std::cos(phi1);
                const double y1 = radius * std::cos(theta1);
                const double z1 = radius * std::sin(theta1) * std::sin(phi1);

                const double x2 = radius * std::sin(theta1) * std::cos(phi2);
                const double y2 = radius * std::cos(theta1);
                const double z2 = radius * std::sin(theta1) * std::sin(phi2);

                const double x3 = radius * std::sin(theta2) * std::cos(phi1);
                const double y3 = radius * std::cos(theta2);
                const double z3 = radius * std::sin(theta2) * std::sin(phi1);

                lineList.append(MPoint(x1, y1, z1));
                lineList.append(MPoint(x2, y2, z2));
                
                lineList.append(MPoint(x1, y1, z1));
                lineList.append(MPoint(x3, y3, z3));
            }
        }
    }

    void appendWireCapsule(MPointArray& lineList, double radiusA, double radiusB, double height, int segments)
    {
        radiusA = std::max(0.0, radiusA);
        radiusB = std::max(0.0, radiusB);
        height = std::max(0.0, height);
        segments = std::max(4, segments);

        const double pi = mathUtils::kPi;
        const double halfHeight = height * 0.5;
        const double doubleSegments = static_cast<double>(segments);

        // body lines
        for (int i = 0; i < segments; ++i)
        {
            const double phi = 2.0 * pi * static_cast<double>(i) / doubleSegments;

            const double x1 = radiusA * std::cos(phi);
            const double z1 = radiusA * std::sin(phi);

            const double x2 = radiusB * std::cos(phi);
            const double z2 = radiusB * std::sin(phi);

            lineList.append(MPoint(x1, halfHeight, z1));
            lineList.append(MPoint(x2, -halfHeight, z2));
        }

        // upper hemisphere
        for (int i = 0; i < segments / 2; ++i)
        {
            const double theta1 = pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = pi * static_cast<double>(i + 1) / doubleSegments;

            for (int j = 0; j < segments; ++j)
            {
                const double phi1 = 2.0 * pi * static_cast<double>(j) / doubleSegments;
                const double phi2 = 2.0 * pi * static_cast<double>(j + 1) / doubleSegments;

                const double x1 = radiusA * std::cos(theta1) * std::cos(phi1);
                const double y1 = radiusA * std::sin(theta1);
                const double z1 = radiusA * std::cos(theta1) * std::sin(phi1);

                const double x2 = radiusA * std::cos(theta1) * std::cos(phi2);
                const double y2 = radiusA * std::sin(theta1);
                const double z2 = radiusA * std::cos(theta1) * std::sin(phi2);

                const double x3 = radiusA * std::cos(theta2) * std::cos(phi1);
                const double y3 = radiusA * std::sin(theta2);
                const double z3 = radiusA * std::cos(theta2) * std::sin(phi1);

                lineList.append(MPoint(x1, y1 + halfHeight, z1));
                lineList.append(MPoint(x2, y2 + halfHeight, z2));

                lineList.append(MPoint(x1, y1 + halfHeight, z1));
                lineList.append(MPoint(x3, y3 + halfHeight, z3));
            }
        }

        // lower hemisphere
        for (int i = 0; i < segments / 2; ++i)
        {
            const double theta1 = pi * static_cast<double>(i) / doubleSegments;
            const double theta2 = pi * static_cast<double>(i + 1) / doubleSegments;

            for (int j = 0; j < segments; ++j)
            {
                const double phi1 = 2.0 * pi * static_cast<double>(j) / doubleSegments;
                const double phi2 = 2.0 * pi * static_cast<double>(j + 1) / doubleSegments;

                const double x1 = radiusB * std::cos(theta1) * std::cos(phi1);
                const double y1 = -radiusB * std::sin(theta1);
                const double z1 = radiusB * std::cos(theta1) * std::sin(phi1);

                const double x2 = radiusB * std::cos(theta1) * std::cos(phi2);
                const double y2 = -radiusB * std::sin(theta1);
                const double z2 = radiusB * std::cos(theta1) * std::sin(phi2);

                const double x3 = radiusB * std::cos(theta2) * std::cos(phi1);
                const double y3 = -radiusB * std::sin(theta2);
                const double z3 = radiusB * std::cos(theta2) * std::sin(phi1);

                lineList.append(MPoint(x1, y1 - halfHeight, z1));
                lineList.append(MPoint(x2, y2 - halfHeight, z2));

                lineList.append(MPoint(x1, y1 - halfHeight, z1));
                lineList.append(MPoint(x3, y3 - halfHeight, z3));
            }
        }
    }

    void appendWirePlane(MPointArray& lineList, double size)
    {
        size = std::max(0.0, size);
        
        const MPoint a = MPoint(-size, 0.0, -size);
        const MPoint b = MPoint(size, 0.0, -size);
        const MPoint c = MPoint(size, 0.0, size);
        const MPoint d = MPoint(-size, 0.0, size);
        const MPoint e = MPoint(0.0, size, 0.0);

        lineList.append(a);
        lineList.append(b);

        lineList.append(b);
        lineList.append(c);

        lineList.append(c);
        lineList.append(d);
        
        lineList.append(d);
        lineList.append(a);

        lineList.append(a);
        lineList.append(c);

        lineList.append(b);
        lineList.append(d);

        lineList.append(MPoint(0.0, 0.0, 0.0));
        lineList.append(e);
    }
}