#include "colliderGeometry.h"
#include "../mathUtils.h"

#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MDrawContext.h>

#include <algorithm>
#include <cmath>

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

    void appendWireSphere(MPointArray& lineList, double radius, int segments)
    {
        radius = std::max(0.0, radius);
        segments = std::max(4, segments);

        const double pi = mathUtils::kPi;

        for (int i = 0; i < segments; ++i)
        {
            const double theta1 = pi * static_cast<double>(i) / static_cast<double>(segments);
            const double theta2 = pi * static_cast<double>(i + 1) / static_cast<double>(segments);

            for (int j = 0; j < segments; ++j)
            {
                const double phi1 = 2.0 * pi * static_cast<double>(j) / static_cast<double>(segments);
                const double phi2 = 2.0 * pi * static_cast<double>(j + 1) / static_cast<double>(segments);

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
}
