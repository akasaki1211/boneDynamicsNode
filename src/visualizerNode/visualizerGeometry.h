#pragma once

// Disabled min/max macros in Windows API
#define NOMINMAX

#include <maya/MPointArray.h>
#include <maya/MColor.h>
#include <maya/MObject.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MUserData.h>

namespace visualizerGeometry
{
    constexpr int kDefaultSegments = 32;

    class VisualizerDrawData : public MUserData
    {
    public:
        VisualizerDrawData();
        virtual ~VisualizerDrawData() {}

        MPointArray radiusSphereLines;
        MPointArray angleLimitLines;
        MColor color;
        unsigned int depthPriority;
    };

    bool getBoolPlug(const MObject& node, const MObject& attribute, bool defaultValue = false);
    //int getIntPlug(const MObject& node, const MObject& attribute, int defaultValue);
    double getDoublePlug(const MObject& node, const MObject& attribute, double defaultValue = 0.0);
    MVector getVectorPlug(const MObject& node, const MObject& attribute, const MVector& defaultValue = MVector::zero);
    MMatrix getMatrixPlug(const MObject& node, const MObject& attribute, const MMatrix& defaultValue = MMatrix::identity);

    void appendRadiusSphere(
        MPointArray& lineList, 
        double radius, 
        const MMatrix& sphereWorldMatrix, 
        int segments = kDefaultSegments
    );

    void appendAngleLimitCone(
        MPointArray& lineList, 
        double angleLimitDegrees, 
        double length, 
        const MMatrix& coneWorldMatrix, 
        int segments = kDefaultSegments
    );
}