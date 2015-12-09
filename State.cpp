#include "State.h"
#include "Collider.h"
#include "Const.h"
#include "Debug.h"
#include "Map.h"
#include "math3d.h"

#include <algorithm>
#include <exception>
#include <memory>
#include <sstream>
#include <iostream>

using namespace std;

const double EPSILON = 1e-7;

// We consider that we pick up a bonus if the rectangle this much smaller than the bonus intersects our rectangle.
const double BONUS_SIZE_RATIO = 1.25;

CarPosition carById(const World *world, long long id) {
    for (auto& car : world->getCars()) {
        if (car.getId() == id) {
            return CarPosition(world, &car);
        }
    }
    throw invalid_argument(string("no car found by id ") + to_string(id));
}

vector<BonusPosition> convertBonuses(const vector<Bonus>& bonuses) {
    vector<BonusPosition> result;
    result.reserve(bonuses.size());
    for (auto& bonus : bonuses) {
        result.emplace_back(&bonus);
    }
    sort(result.begin(), result.end(), [](const BonusPosition& b1, const BonusPosition& b2) {
        if (b1.location.x < b2.location.x) return true;
        if (b1.location.x > b2.location.x) return false;
        return b1.location.y < b2.location.y;
    });
    return result;
}

vector<double> computeBonusX(const vector<BonusPosition>& bonuses) {
    vector<double> result;
    result.reserve(bonuses.size());
    for (auto& bonus : bonuses) {
        result.push_back(bonus.location.x);
    }
    return result;
}

State::State(const World *world, long long id) : original(world),
    car(carById(world, id)),
    bonuses(convertBonuses(world->getBonuses())),
    bonusX(computeBonusX(bonuses))
{ }

double updateEnginePower(double carEnginePower, double moveEnginePower) {
    static double maxChange = Const::getGame().getCarEnginePowerChangePerTick();
    auto delta = moveEnginePower - carEnginePower;
    return carEnginePower + min(max(delta, -maxChange), maxChange);
}

double updateWheelTurn(double carWheelTurn, WheelTurnDirection direction) {
    static double maxChange = Const::getGame().getCarWheelTurnChangePerTick();
    switch (direction) {
        case WheelTurnDirection::TURN_LEFT: return max(carWheelTurn - maxChange, -1.0);
        case WheelTurnDirection::KEEP: return carWheelTurn;
        case WheelTurnDirection::TURN_RIGHT: return min(carWheelTurn + maxChange, 1.0);
        default: return carWheelTurn;
    }
}

Vec3D toVec3D(const Vec& vec) {
    return Vec3D(vec.x, vec.y, 0.0);
}

Vec3D toVec3D(const Point& p1, const Point& p2) {
    return toVec3D(Vec(p1, p2));
}

void resolveImpact(
        const CollisionInfo& collision, CarPosition& car, const Vec3D& normal, const Vec3D& vecBC, const Vec3D& relativeVelocity
) {
    static const double invertedBuggyMass = 1.0 / Const::getGame().getBuggyMass();
    static const double invertedJeepMass = 1.0 / Const::getGame().getJeepMass();
    static const double momentumTransferFactor = 0.3; // ?!

    double invertedCarMass = car.isBuggy() ? invertedBuggyMass : invertedJeepMass;

    double invertedCarAngularMass = car.angularSpeed * invertedCarMass / car.velocity.length();

    auto part = ((vecBC ^ normal) * invertedCarAngularMass) ^ vecBC;
    auto denominator = invertedCarMass + normal * part;

    auto impulseChange = -1.0 * (1.0 + momentumTransferFactor) * (relativeVelocity * normal) / denominator;
    // if (D) cout << "impact impulse-change " << impulseChange << endl;
    if (abs(impulseChange) < EPSILON) return;

    auto velocityChange = normal * (impulseChange * invertedCarMass);
    auto newVelocity = toVec3D(car.velocity) - velocityChange;
    // if (D) cout << "impact previous-velocity " << car.velocity.toString() << endl;
    car.velocity = Vec(newVelocity.x, newVelocity.y);

    // if (D) cout << "impact velocity-change " << velocityChange.x << " " << velocityChange.y << " new-velocity " << car.velocity.toString() << endl;

    auto angularSpeedChange = (vecBC ^ (normal * impulseChange)) * invertedCarAngularMass;
    car.angularSpeed -= angularSpeedChange.z;
    // if (D) cout << "impact angular-speed-change " << angularSpeedChange.z << " new-angular-speed " << car.angularSpeed << endl;
}

void resolveSurfaceFriction(
        const CollisionInfo& collision, CarPosition& car, const Vec3D& normal, const Vec3D& vecBC, const Vec3D& relativeVelocity
) {
    static const double invertedBuggyMass = 1.0 / Const::getGame().getBuggyMass();
    static const double invertedJeepMass = 1.0 / Const::getGame().getJeepMass();
    static const double surfaceFrictionFactor = 0.015; // ?!

    double invertedCarMass = car.isBuggy() ? invertedBuggyMass : invertedJeepMass;

    double invertedCarAngularMass = car.angularSpeed * invertedCarMass / car.velocity.length();

    auto tangent = relativeVelocity - normal * (relativeVelocity * normal);
    if (tangent.normSquare() < EPSILON * EPSILON) return;

    tangent = tangent.normalize();
    auto surfaceFriction = mySqrt(2 * surfaceFrictionFactor) * abs(relativeVelocity * normal) / relativeVelocity.norm();
    if (surfaceFriction < EPSILON) return;

    auto part = ((vecBC ^ tangent) * invertedCarAngularMass) ^ vecBC;
    auto denominator = invertedCarMass + tangent * part;

    auto impulseChange = -1.0 * surfaceFriction * (relativeVelocity * tangent) / denominator;
    // if (D) cout << "surface-friction impulse-change " << impulseChange << endl;
    if (abs(impulseChange) < EPSILON) return;

    auto velocityChange = tangent * (impulseChange * invertedCarMass);
    auto newVelocity = toVec3D(car.velocity) - velocityChange;
    // if (D) cout << "surface-friction previous-velocity " << car.velocity.toString() << endl;
    car.velocity = Vec(newVelocity.x, newVelocity.y);

    // if (D) cout << "surface-friction velocity-change " << velocityChange.x << " " << velocityChange.y << " new-velocity " << car.velocity.toString() << endl;

    auto angularSpeedChange = (vecBC ^ (tangent * impulseChange)) * invertedCarAngularMass;
    car.angularSpeed -= angularSpeedChange.z;
    // if (D) cout << "surface-friction angular-speed-change " << angularSpeedChange.z << " new-angular-speed " << car.angularSpeed << endl;
}

void resolveWallCollision(const CollisionInfo& collision, CarPosition& car) {
    // if (D) cout << "  velocity " << car.velocity.length() << endl;
    car.health = max(car.health - car.velocity.length() / 200.0, 0.0); // ?!

    // TODO: turn on
    /*
    auto normal = toVec3D(collision.normal);
    auto vecBC = toVec3D(car.location, collision.point);
    auto angularSpeedBC = Vec3D(0.0, 0.0, car.angularSpeed) ^ vecBC;
    auto velocityBC = toVec3D(car.velocity) + angularSpeedBC;
    auto relativeVelocity = -velocityBC;
    // if (D) cout << "vec-bc " << vecBC.toString() << " relative-velocity " << relativeVelocity.toString() << endl;
    if (relativeVelocity * normal < EPSILON) {
        // TODO: optimize
        resolveImpact(collision, car, normal, vecBC, relativeVelocity);
        resolveSurfaceFriction(collision, car, normal, vecBC, relativeVelocity);
    }
    */

    // TODO: dirty hack
    auto speed = car.velocity.length();
    if (speed > EPSILON) {
        car.velocity *= (car.velocity * collision.normal) / speed;
    }

    if (collision.depth > EPSILON) {
        car.location -= collision.normal * (collision.depth + EPSILON);
    }
}

struct Walls {
    vector<Segment> segments;
    vector<Circle> corners;
    unsigned long height;

    Walls(const vector<Segment>& segments, const vector<Circle>& corners) :
        segments(segments), corners(corners), height(Map::getMap().height) { }

    const Segment& segment(unsigned long tx, unsigned long ty, int d) const {
        return segments[((tx * height) + ty) * 4 + d];
    }

    const Circle& corner(unsigned long tx, unsigned long ty, int d) const {
        return corners[((tx * height) + ty) * 4 + d];
    }

    static Walls *instance;
};

Walls *Walls::instance = nullptr;

Walls *computeWalls() {
    static Game& game = Const::getGame();
    static const unsigned long mapWidth = Map::getMap().width;
    static const unsigned long mapHeight = Map::getMap().height;
    static const double tileSize = game.getTrackTileSize();

    // To avoid driving right against walls and corners
    static const double segmentMargin = game.getTrackTileMargin() * 1.01;
    static const double circleRadius = game.getTrackTileMargin() * 1.05;

    auto segments = vector<Segment>(mapWidth * mapHeight * 4, Segment { Point(), Point() });
    auto circles = vector<Circle>(mapWidth * mapHeight * 4, Circle { Point(), double() });

    static const int dx[] = {1, 0, -1, 0};
    static const int dy[] = {0, 1, 0, -1};

    for (unsigned long tx = 0; tx < mapWidth; tx++) {
        for (unsigned long ty = 0; ty < mapHeight; ty++) {
            for (int d = 0; d < 4; d++) {
                auto p1 = Point(
                        (tx + max(dx[d] + dy[d], 0)) * tileSize - dx[d] * segmentMargin,
                        (ty + max(dy[d] - dx[d], 0)) * tileSize - dy[d] * segmentMargin
                );
                auto p2 = Point(
                        (tx + max(dx[d] - dy[d], 0)) * tileSize - dx[d] * segmentMargin,
                        (ty + max(dx[d] + dy[d], 0)) * tileSize - dy[d] * segmentMargin
                );
                auto segment = Segment(p1, p2);

                // Reorder p1 and p2 so that if p1.x == p2.x then p1.y < p2.y, and if p1.y == p2.y then p1.x < p2.x
                if (!(d & 1) && segment.p1.y > segment.p2.y) swap(segment.p1, segment.p2);
                if ((d & 1) && segment.p1.x > segment.p2.x) swap(segment.p1, segment.p2);

                segments[((tx * mapHeight) + ty) * 4 + d] = segment;

                auto p = Point(
                        (tx + (dx[d] - dy[d] + 1.) / 2) * tileSize,
                        (ty + (dx[d] + dy[d] + 1.) / 2) * tileSize
                );
                circles[((tx * mapHeight) + ty) * 4 + d] = Circle(p, circleRadius);
            }
        }
    }

    Walls::instance = new Walls(segments, circles);

    return Walls::instance;
}

void collideCarWithSegmentWall(CarPosition& car, const Segment& wall, int d) {
    static const double carRadius = myHypot(Const::getGame().getCarHeight() / 2, Const::getGame().getCarWidth() / 2) + EPSILON;

    auto& location = car.location;
    if (!(d & 1)) {
        if (abs(location.x - wall.p1.x) > carRadius) return;
        if (location.y < wall.p1.y - carRadius) return;
        if (location.y > wall.p2.y + carRadius) return;
    } else {
        if (abs(location.y - wall.p1.y) > carRadius) return;
        if (location.x < wall.p1.x - carRadius) return;
        if (location.x > wall.p2.x + carRadius) return;
    }

    CollisionInfo collision;
    if (collideRectAndSegment(car.rectangle, wall, collision)) {
        /*
        if (Debug::debug) {
            cout << "segment collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
            cout << "  " << car.toString() << endl;
            cout << "  (with segment " << wall.toString() << ")" << endl;
        }
        */
        resolveWallCollision(collision, car);
    }
}

void collideCarWithCornerWall(CarPosition& car, const Circle& corner) {
    static const double carRadius = myHypot(Const::getGame().getCarHeight() / 2, Const::getGame().getCarWidth() / 2) + EPSILON;

    auto& location = car.location;
    auto& center = corner.center;
    double relevantDistance = corner.radius + carRadius;
    if (location.distanceTo(center) > relevantDistance) return;
    if (car.rectangle.distanceFrom(center) > corner.radius + EPSILON) return;

    CollisionInfo collision;
    if (collideCircleAndRect(car.rectangle, corner, collision)) {
        /*
        if (Debug::debug) {
            cout << "corner collision at " << collision.point.toString() << " normal " << collision.normal.toString() << " depth " << collision.depth << endl;
            cout << "  " << car.toString() << endl;
            cout << "  (with corner at " << corner.toString() << ")" << endl;
        }
        */
        resolveWallCollision(collision, car);
    }
}

void determineTileBounds(
        const Rect& rectangle, unsigned long& txBegin, unsigned long& txEnd, unsigned long& tyBegin, unsigned long& tyEnd
) {
    static const double tileSize = Const::getGame().getTrackTileSize();

    auto& p = rectangle.points();

    auto x0 = p[0].x, x1 = p[1].x, x2 = p[2].x, x3 = p[3].x;
    auto y0 = p[0].y, y1 = p[1].y, y2 = p[2].y, y3 = p[3].y;
    if (x0 > x2) swap(x0, x2); if (x1 > x3) swap(x1, x3); if (x0 > x1) swap(x0, x1); if (x2 > x3) swap(x2, x3);
    if (y0 > y2) swap(y0, y2); if (y1 > y3) swap(y1, y3); if (y0 > y1) swap(y0, y1); if (y2 > y3) swap(y2, y3);

    txBegin = static_cast<unsigned long>(x0 / tileSize);
    txEnd = static_cast<unsigned long>(x3 / tileSize);
    tyBegin = static_cast<unsigned long>(y0 / tileSize);
    tyEnd = static_cast<unsigned long>(y3 / tileSize);
}

void collideCarWithWalls(CarPosition& car) {
    static const double trackTileSize = Const::getGame().getTrackTileSize();

    static Map& map = Map::getMap();

    static Walls *allWalls = computeWalls();

    unsigned long txBegin, txEnd, tyBegin, tyEnd;
    determineTileBounds(car.rectangle, txBegin, txEnd, tyBegin, tyEnd);

    // TODO: this shouldn't happen and is only here to prevent segfaults
    if (txBegin > map.width || txEnd > map.width || tyBegin > map.height || tyEnd > map.height) return;

    for (auto tx = txBegin; tx <= txEnd; tx++) {
        for (auto ty = tyBegin; ty <= tyEnd; ty++) {
            int tile = map.get(tx, ty);

            for (int d = 0; d < 4; d++) {
                if (!(tile & (1 << d))) {
                    collideCarWithSegmentWall(car, allWalls->segment(tx, ty, d), d);
                }
            }
        }
    }

    auto cornerX = static_cast<unsigned long>(car.location.x / trackTileSize + 0.5);
    if (cornerX < map.width) {
        auto cornerY = static_cast<unsigned long>(car.location.y / trackTileSize + 0.5);
        if (cornerY < map.height) {
            collideCarWithCornerWall(car, allWalls->corner(cornerX, cornerY, 2));
        }
    }
}

void applyBonus(CarPosition& car, BonusPosition& bonus) {
    bonus.isAlive = false;
    switch (bonus.original->getType()) {
        case model::REPAIR_KIT: car.medicines++; car.health = 1.0; break;
        case model::AMMO_CRATE: car.projectiles++; break;
        case model::NITRO_BOOST: car.nitroCharges++; break;
        case model::OIL_CANISTER: car.oilCanisters++; break;
        case model::PURE_SCORE: car.pureScore++; break;
        default: break;
    }
}

void collectBonuses(CarPosition& car, vector<BonusPosition>& bonuses, const vector<double>& bonusX) {
    static const double maximumRelevantDistance = myHypot(
            (Const::getGame().getCarHeight() + Const::getGame().getBonusSize()) / 2,
            (Const::getGame().getCarWidth() + Const::getGame().getBonusSize()) / 2
    ) + EPSILON;

    auto& location = car.location;
    auto& points = car.rectangle.points();
    
    auto bonusBegin = lower_bound(bonusX.begin(), bonusX.end(), location.x - maximumRelevantDistance) - bonusX.begin();
    auto bonusEnd = upper_bound(bonusX.begin(), bonusX.end(), location.x + maximumRelevantDistance) - bonusX.begin();

    Point unused;
    for (auto bonusIndex = bonusBegin; bonusIndex < bonusEnd; bonusIndex++) {
        auto& bonus = bonuses[bonusIndex];
        // if (Debug::debug) cout << "  bonus at " << bonus.location.toString() << " " << bonus.isAlive << " distance " << location.distanceTo(bonus.location) << endl;
        if (!bonus.isAlive) continue;
        auto& bonusLocation = bonus.location;
        auto dx = location.x - bonusLocation.x;
        if (dx < -maximumRelevantDistance || dx > maximumRelevantDistance) continue;
        auto dy = location.y - bonusLocation.y;
        if (dy < -maximumRelevantDistance || dy > maximumRelevantDistance) continue;
        if (location.distanceTo(bonusLocation) > maximumRelevantDistance) continue;

        bool signedDistancesNegative = true;
        bool shouldApply = false;
        for (unsigned long i = 0, size = points.size(); i < size; i++) {
            auto side = Segment(points[i], points[i + 1 == size ? 0 : i + 1]);
            if (side.intersects(bonus.innerRectangle, unused)) {
                shouldApply = true;
                break;
            }
            signedDistancesNegative &= Line(side.p1, side.p2).signedDistanceFrom(bonusLocation) < 0;
        }

        if (shouldApply || signedDistancesNegative) {
            applyBonus(car, bonus);
        }
    }
}

void State::apply(const Go& move) {
    car.advance(move);
    collideCarWithWalls(car);
    collectBonuses(car, bonuses, bonusX);
}

Rect rectangleByUnit(double angle, double x, double y, double width, double height) {
    Vec dir = Vec(angle);
    Vec forward = dir * (width / 2);
    Vec sideways = Vec(-dir.y, dir.x) * (height / 2);
    Point lpf = Point(x + forward.x, y + forward.y);
    Point lmf = Point(x - forward.x, y - forward.y);
    return Rect({ lpf - sideways, lpf + sideways, lmf + sideways, lmf - sideways });
}

void CarPosition::advance(const Go& move) {
    static auto& game = Const::getGame();

    static const double halfCarWidth = game.getCarWidth() / 2;
    static const double halfCarHeight = game.getCarHeight() / 2;

    static const double carAngularSpeedFactor = game.getCarAngularSpeedFactor();
    static const int nitroDurationTicks = game.getNitroDurationTicks();
    static const double nitroEnginePowerFactor = game.getNitroEnginePowerFactor();

    static const double jeepEngineRearAcceleration = game.getJeepEngineRearPower() / game.getJeepMass();
    static const double jeepEngineForwardAcceleration = game.getJeepEngineForwardPower() / game.getJeepMass();

    static const double buggyEngineRearAcceleration = game.getBuggyEngineRearPower() / game.getBuggyMass();
    static const double buggyEngineForwardAcceleration = game.getBuggyEngineForwardPower() / game.getBuggyMass();

    static const double airFriction = 1.0 - game.getCarMovementAirFrictionFactor();
    static const double lengthwiseVelocityChangeBase = game.getCarLengthwiseMovementFrictionFactor();
    static const double crosswiseVelocityChange = game.getCarCrosswiseMovementFrictionFactor();

    auto dir = direction();
    auto side = Vec(-dir.y, dir.x);

    // Engine & wheels

    if (move.useNitro && nitroCharges > 0 && nitroCooldown == 0) {
        nitroCharges--;
        nitroCooldown = nitroDurationTicks;
    }

    if (nitroCooldown > 0) {
        nitroCooldown--;
        enginePower = nitroEnginePowerFactor;
    } else {
        enginePower = updateEnginePower(enginePower == nitroEnginePowerFactor ? 1.0 : enginePower, move.enginePower);
    }

    wheelTurn = updateWheelTurn(wheelTurn, move.wheelTurn);
    angularSpeed = wheelTurn * carAngularSpeedFactor * (velocity * dir);

    // Location

    location += velocity;

    {
        // See rectangleByUnit()
        auto& points = rectangle.points();
        auto x = location.x, y = location.y;
        auto forwardX = dir.x * halfCarWidth, forwardY = dir.y * halfCarWidth;
        auto sideX = side.x * halfCarHeight, sideY = side.y * halfCarHeight;
        points[0].x = x + forwardX - sideX; points[0].y = y + forwardY - sideY;
        points[1].x = x + forwardX + sideX; points[1].y = y + forwardY + sideY;
        points[2].x = x - forwardX + sideX; points[2].y = y - forwardY + sideY;
        points[3].x = x - forwardX - sideX; points[3].y = y - forwardY - sideY;
    }

    if (!move.brake) {
        auto acceleration = enginePower * (enginePower < 0 ?
                (isBuggy() ? buggyEngineRearAcceleration : jeepEngineRearAcceleration):
                (isBuggy() ? buggyEngineForwardAcceleration : jeepEngineForwardAcceleration)
        );
        velocity += dir * acceleration;
    }

    // Air friction

    velocity *= airFriction;

    // Movement friction

    auto lengthwiseVelocityChange = move.brake ? crosswiseVelocityChange : lengthwiseVelocityChangeBase;
    auto lengthwiseVelocityPart = velocity * dir;
    lengthwiseVelocityPart =
            lengthwiseVelocityPart >= 0.0
            ? max(lengthwiseVelocityPart - lengthwiseVelocityChange, 0.0)
            : min(lengthwiseVelocityPart + lengthwiseVelocityChange, 0.0);

    auto crosswiseVelocityPart = velocity * side;
    crosswiseVelocityPart =
            crosswiseVelocityPart >= 0.0
            ? max(crosswiseVelocityPart - crosswiseVelocityChange, 0.0)
            : min(crosswiseVelocityPart + crosswiseVelocityChange, 0.0);

    velocity = dir * lengthwiseVelocityPart + side * crosswiseVelocityPart;

    // Angle

    angle = normalizeAngle(angle + angularSpeed);
}

CarPosition::CarPosition(const World *world, const Car *car) : original(car),
        originalWorld(world),
        location(Point(car->getX(), car->getY())),
        velocity(Vec(car->getSpeedX(), car->getSpeedY())),
        angle(car->getAngle()),
        angularSpeed(car->getAngularSpeed()),
        enginePower(car->getEnginePower()),
        wheelTurn(car->getWheelTurn()),
        health(car->getDurability()),
        projectiles(car->getProjectileCount()),
        nitroCharges(car->getNitroChargeCount()),
        oilCanisters(car->getOilCanisterCount()),
        nitroCooldown(car->getRemainingNitroCooldownTicks()),
        rectangle(rectangleByUnit(car->getAngle(), car->getX(), car->getY(), car->getWidth(), car->getHeight())),
        medicines(0),
        pureScore(0) { }

Point CarPosition::bumperCenter() const {
    // return location + direction() * (Const::getGame().getCarWidth() / 2);
    return Point::between(rectangle.points()[0], rectangle.points()[1]);
}

Tile CarPosition::tile() const {
    return Tile(bumperCenter());
};

bool CarPosition::isBuggy() const {
    return original->getType() == model::CarType::BUGGY;
}

DirectedTile CarPosition::directedTile() const {
    double positiveAngle = normalizeAngle(angle);
    if (positiveAngle < 0) positiveAngle += M_PI + M_PI;
    int direction = static_cast<int>(positiveAngle * 4.0 / M_PI + 0.5) % 8;
    return DirectedTile(tile(), direction);
}

string CarPosition::toString() const {
    ostringstream ss;
    ss.precision(3);
    ss << fixed << "car at " << location.toString() <<
            " going " << velocity.toString() <<
            " angle " << angle <<
            " angular " << angularSpeed <<
            " engine " << enginePower <<
            " wheel " << wheelTurn <<
            " health " << health <<
            " ammo " << projectiles <<
            " nitros " << nitroCharges <<
            " oil " << oilCanisters <<
            " score " << pureScore <<
            " nitro-cooldown " << nitroCooldown;
    return ss.str();
}

BonusPosition::BonusPosition(const Bonus *bonus) : original(bonus),
        location(Point(bonus->getX(), bonus->getY())),
        innerRectangle(rectangleByUnit(bonus->getAngle(), bonus->getX(), bonus->getY(),
                    bonus->getWidth() / BONUS_SIZE_RATIO, bonus->getHeight() / BONUS_SIZE_RATIO)),
        isAlive(true) { }
