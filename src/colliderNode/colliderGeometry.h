#pragma once

// Disabled min/max macros in Windows API
#define NOMINMAX

#include <maya/MPointArray.h>
#include <maya/MColor.h>
#include <maya/MObject.h>
#include <maya/MBoundingBox.h>
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
    MBoundingBox makeCapsuleBoundingBox(double radiusA, double radiusB, double height);
    MBoundingBox makePlaneBoundingBox(double size = 1.0);

    void appendWireSphere(MPointArray& lineList, double radius, int segments = kDefaultSegments);
    void appendWireCapsule(MPointArray& lineList, double radiusA, double radiusB, double height, int segments = kDefaultSegments);
    void appendWirePlane(MPointArray& lineList, double size = 1.0);
}