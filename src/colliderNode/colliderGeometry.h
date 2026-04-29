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

    float getDistancePlugAsCentimeters(const MObject& node, const MObject& attribute, float defaultValue);

    MBoundingBox makeSphereBoundingBox(float radius);

    void appendWireSphere(MPointArray& lineList, float radius, int segments = kDefaultSegments);
}