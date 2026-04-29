#include "colliderGeometry.h"
#include "../mathUtils.h"

#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MDrawContext.h>

#include <algorithm>

namespace colliderGeometry
{
    ColliderDrawData::ColliderDrawData() : MUserData(false), 
        color(1.0f, 1.0f, 1.0f), 
        depthPriority(MHWRender::MRenderItem::sDormantWireDepthPriority)
    {}

    float getDistancePlugAsCentimeters(const MObject& node, const MObject& attribute, float defaultValue)
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

        return static_cast<float>(value.asCentimeters());
    }

    MBoundingBox makeSphereBoundingBox(float radius)
    {
        radius = std::max(0.0f, radius);

        MPoint corner1(-radius, -radius, -radius);
        MPoint corner2(radius, radius, radius);

        return MBoundingBox(corner1, corner2);
    }

    void appendWireSphere(MPointArray& lineList, float radius, int segments)
    {
        radius = std::max(0.0f, radius);
        segments = std::max(4, segments);

        const float pi = static_cast<float>(mathUtils::kPi);

        for (int i = 0; i < segments; ++i)
        {
            const float theta1 = pi * static_cast<float>(i) / static_cast<float>(segments);
            const float theta2 = pi * static_cast<float>(i + 1) / static_cast<float>(segments);

            for (int j = 0; j < segments; ++j)
            {
                const float phi1 = 2.0f * pi * static_cast<float>(j) / static_cast<float>(segments);
                const float phi2 = 2.0f * pi * static_cast<float>(j + 1) / static_cast<float>(segments);

                const float x1 = radius * std::sin(theta1) * std::cos(phi1);
                const float y1 = radius * std::cos(theta1);
                const float z1 = radius * std::sin(theta1) * std::sin(phi1);

                const float x2 = radius * std::sin(theta1) * std::cos(phi2);
                const float y2 = radius * std::cos(theta1);
                const float z2 = radius * std::sin(theta1) * std::sin(phi2);

                const float x3 = radius * std::sin(theta2) * std::cos(phi1);
                const float y3 = radius * std::cos(theta2);
                const float z3 = radius * std::sin(theta2) * std::sin(phi1);

                lineList.append(MPoint(x1, y1, z1));
                lineList.append(MPoint(x2, y2, z2));
                
                lineList.append(MPoint(x1, y1, z1));
                lineList.append(MPoint(x3, y3, z3));
            }
        }
    }
}