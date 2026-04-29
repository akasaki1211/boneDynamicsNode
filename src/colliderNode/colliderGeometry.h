#pragma once

#include <maya/MPointArray.h>
#include <maya/MColor.h>
#include <maya/MObject.h>
#include <maya/MBoundingBox.h>
#include <maya/MVector.h>
#include <maya/MUserData.h>

namespace colliderGeometry
{
    constexpr int kDefaultSegments = 16;
    
    class ColliderDrawData : public MUserData
    {
    public:
        ColliderDrawData();
        virtual ~ColliderDrawData() {}

        MPointArray lineList;
        MColor color;
        unsigned int depthPriority;
    };

    double getDistancePlugAsCentimeters(const MObject& node, const MObject& attribute, double defaultValue);

    MBoundingBox makeSphereBoundingBox(double radius);

    void appendWireSphere(MPointArray& lineList, double radius, int segments = kDefaultSegments);
}
