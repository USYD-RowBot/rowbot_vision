#include "Object.hpp"

void Object::setBearing(float bearing) {
    m_bearing = bearing;
}

void Object::setRange(float range) {
    m_range = range;
}

void Object::print() const {
    std::cout << "Object detected at bearing of " << m_bearing << " degrees";
    std::cout << " and range of " << m_range;
    std::cout << ". Possible classifications:" << std::endl;
    for (int i = 0; i < m_types.size(); ++i) {
        std::cout << "    " << m_types[i] << " with ";
        std::cout << 100*m_confidences[i] << "% confidence" << std::endl;
    }
}

float Object::getMaxConfidence() const {
    float maxConfidence = -1;
    for (int i = 0; i < m_confidences.size(); ++i) {
        if (m_confidences[i] > maxConfidence) {
            maxConfidence = m_confidences[i];
        }
    }
    return maxConfidence; // -1 if empty confidences
}

float getBearing() const {
    return m_bearing;
}

float getRange() const {
    return m_range;
}
