#pragma once

#include <vector>
#include <core/SkPathBuilder.h>
#include <core/SkPoint.h>

SkVector rotate(SkVector v, float angle);

class Path {
public:
  void addPoint(float x, float y, float r);

  const SkPath &getSkPath();

private:
  void computePath();

  struct Point {
    SkPoint point;
    float r;

    SkRect bounds() const;

    bool contains(const Point &o) const;
  };

  static float tangentAngle(const Point &p, const Point &q);

  struct LineSegment {
    SkPoint p, q;

    float angle() const;
  };

  static bool computeIntersection(const LineSegment &s1, const LineSegment &s2, SkPoint &i);

  std::vector<Point> points;
  SkPath path;
  bool dirty{};
};