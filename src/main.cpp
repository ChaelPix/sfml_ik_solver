/********************************************
 * IK solver by using circles method.
 * Readapted from : https://github.com/tushartalukder/Automated-Realtime-Robotic-Arm
 * 
 * @ChaelPix
*********************************************/
#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <functional>
#include <numeric>

class Vector2D {
public:
    float x, y;

    Vector2D(float x = 0, float y = 0) : x(x), y(y) {}

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    Vector2D normalized() const {
        float len = length();
        return Vector2D(x / len, y / len);
    }

    float sin() const {
        return y / length();
    }

    float cos() const {
        return x / length();
    }

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    bool operator<(const Vector2D& other) const {
        return length() < other.length();
    }
};

void drawCircle(sf::RenderWindow& window, Vector2D center, float radius, sf::Color color) {
    sf::CircleShape circle(radius);
    circle.setFillColor(sf::Color::Transparent);
    circle.setOutlineThickness(1);
    circle.setOutlineColor(color);
    circle.setOrigin(radius, radius);
    circle.setPosition(center.x, center.y);
    window.draw(circle);
}

bool checkTriangleValidity(float a, float b, float c) {
    return a + b >= c && a + c >= b && b + c >= a;
}

std::pair<Vector2D, Vector2D> getIntersections(Vector2D position1, float radius1, Vector2D position2, float radius2) {
    Vector2D distanceVector = position2 - position1;
    float distance = distanceVector.length();
    float a = std::max((radius1 * radius1 - radius2 * radius2 + distance * distance) / (2 * distance), 0.0f);
    float height = std::sqrt(std::max(radius1 * radius1 - a * a, 0.0f));
    Vector2D heightVector(-height * distanceVector.sin(), height * distanceVector.cos());

    Vector2D point = position1 + distanceVector.normalized() * a;
    Vector2D intersection1 = point + heightVector;
    Vector2D intersection2 = point - heightVector;

    return std::make_pair(intersection1, intersection2);
}

float findSide(float minimalLength, float maximalLength, float side1, float side2) {
    for (float side = maximalLength; side >= minimalLength; side -= 0.5f) {
        if (checkTriangleValidity(side, side1, side2)) {
            return side;
        }
    }
    return 0;
}

std::vector<Vector2D> resolveIK(const std::vector<float>& chain, std::vector<Vector2D>& vectors, Vector2D endEffector, float maximalDistance, Vector2D pole) {
    std::vector<Vector2D> newVectors;
    if (endEffector.length() > maximalDistance) {
        endEffector = endEffector.normalized() * maximalDistance;
    }
    Vector2D currentSideVector = endEffector;

    for (int i = chain.size() - 1; i > 0; --i) {
        float currentSide = currentSideVector.length();
        float newSide = findSide(0, std::accumulate(chain.begin(), chain.begin() + i, 0.0f), chain[i], currentSide);

        auto intersections = getIntersections(currentSideVector, chain[i], Vector2D(0, 0), newSide);
        Vector2D intersection = (i > 0 && (intersections.first - vectors[i - 1]).length() <
                                (intersections.second - vectors[i - 1]).length())
                                ? intersections.first
                                : intersections.second;

        newVectors.insert(newVectors.begin(), currentSideVector - intersection);
        currentSideVector = intersection;
        float threshold = 1e-2;
        if (i > 0 && (intersections.first - vectors[i - 1]).length() + threshold <
                    (intersections.second - vectors[i - 1]).length()) {
            intersection = intersections.first;
        } else {
            intersection = intersections.second;
        }

    }

    newVectors.insert(newVectors.begin(), currentSideVector);
    return newVectors;
}

void drawVectorsChain(sf::RenderWindow& window, Vector2D position, const std::vector<Vector2D>& chain, sf::Color color, float width = 1, bool drawCircles = false, float radius = 1, sf::Color circleColor = sf::Color::White) {
    sf::CircleShape circle(radius);
    circle.setFillColor(circleColor);

    for (const auto& vector : chain) {
        Vector2D newVector = position + Vector2D(vector.x, -vector.y);
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(position.x, position.y), color),
            sf::Vertex(sf::Vector2f(newVector.x, newVector.y), color)
        };
        window.draw(line, 2, sf::Lines);

        if (drawCircles) {
            circle.setPosition(position.x - radius, position.y - radius);
            window.draw(circle);
            circle.setPosition(newVector.x - radius, newVector.y - radius);
            window.draw(circle);
        }

        position = newVector;
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode(960, 540), "Pixx Inverse Kinematics");

    std::vector<float> chain = {30, 20,30, 20, 20, 30};                                       //CHANGE CHAIN HERE
    std::vector<Vector2D> vectors(chain.size(), Vector2D(0, 0));

    Vector2D endEffector(0, 0);
    Vector2D pole(0, 0);
    float maximalDistance = std::accumulate(chain.begin(), chain.end(), 0.0f);

    Vector2D screenMiddlePosition(window.getSize().x / 2, window.getSize().y / 2);
    Vector2D poleGlobal(0, 0);
    Vector2D endEffectorGlobal(0, 0);

    sf::Clock clock;
    const int fps = 60;
    bool running = true;

    while (running) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                running = false;
            }
        }

        window.clear(sf::Color(255, 215, 0));

        if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            sf::Vector2i mousePos = sf::Mouse::getPosition(window);
            endEffectorGlobal = Vector2D(mousePos.x, mousePos.y);
            endEffector = endEffectorGlobal - screenMiddlePosition;
            endEffector.y *= -1;
            vectors = resolveIK(chain, vectors, endEffector, maximalDistance, pole);
        }

        if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
            sf::Vector2i mousePos = sf::Mouse::getPosition(window);
            poleGlobal = Vector2D(mousePos.x, mousePos.y);
            pole = poleGlobal - screenMiddlePosition;
            pole.y *= -1;
        }

        sf::CircleShape poleCircle(5);
        poleCircle.setFillColor(sf::Color(0, 242, 255));
        poleCircle.setPosition(poleGlobal.x - 5, poleGlobal.y - 5);
        window.draw(poleCircle);

        sf::CircleShape endEffectorCircle(5);
        endEffectorCircle.setFillColor(sf::Color(15, 153, 113));
        endEffectorCircle.setPosition(endEffectorGlobal.x - 5, endEffectorGlobal.y - 5);
        window.draw(endEffectorCircle);

        // Dessiner le vecteur principal (ligne)
        drawVectorsChain(window, screenMiddlePosition, vectors, sf::Color::White, 7, true, 5, sf::Color(55, 59, 68));

        // Afficher les cercles autour des articulations
        Vector2D currentPosition = screenMiddlePosition;
        for (size_t i = 0; i < vectors.size(); ++i) {
            currentPosition = currentPosition + Vector2D(vectors[i].x, -vectors[i].y);
            drawCircle(window, currentPosition, chain[i], sf::Color(255, 0, 0));
        }


        sf::sleep(sf::milliseconds(1000 / fps));
        window.display();
    }

    return 0;
}
