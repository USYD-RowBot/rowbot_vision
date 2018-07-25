#ifndef OBJECT
#define OBJECT

#include <iostream>
#include <vector>

class Object {
private:
    std::vector<std::string> m_types;
    std::vector<float> m_confidences;
    float m_bearing; // angle based on pixel position on screen
    float m_range;

public:
    Object(std::vector<std::string> types, std::vector<float> confidences)
    : m_types{types}, m_confidences{confidences}
    {};
    void setBearing(float bearing);
    void setRange(float range);
    void print() const;
};

#endif
