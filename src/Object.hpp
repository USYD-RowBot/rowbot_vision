#ifndef OBJECT
#define OBJECT

#include <iostream>
#include <vector>

class Object {
public:
    std::vector<std::string> m_types;
    std::vector<float> m_confidences;
private:
    float m_bearing; // angle based on pixel position on screen
    float m_range;

public:
    Object(std::vector<std::string> types, std::vector<float> confidences)
    : m_types{types}, m_confidences{confidences}
    {};
    void setBearing(float bearing);
    void setRange(float range);
    void print() const;
    float getMaxConfidence() const;
    float getBearing() const;
    float getRange() const;
};

#endif
