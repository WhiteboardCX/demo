#include "Path.h"
#include <core/SkPoint.h>
#include "Util.h"

constexpr float EPS = 1e-3f;

void Path::addPoint(float x, float y, float r) {
  Point p{{x, y}, r};
  if (!points.empty()) {
    //check if previous point contains the new point
    //replace if the new point is larger
    if (points.back().contains(p)) {
      bool replace{};
      while (!points.empty() && points.back().r < r && points.back().contains(p)) {
        replace = true;
        points.pop_back();
      }
      if (!replace) return;
    }
  }
  points.push_back(p);
  dirty = true;
}

const SkPath &Path::getSkPath() {
  if (dirty) {
    computePath();
  }
  return path;
}

void Path::computePath() {
  dirty = false;
  path.rewind();

  const auto n = points.size();
  if (n == 1) {
    auto &p = points[0];
    path.addCircle(p.point.fX, p.point.fY, p.r);
  } else if (n > 1) {
    std::vector<LineSegment> segsF, segsB;
    for (size_t i = 0; i < n - 1; ++i) {
      auto &p = points[i];
      auto &q = points[i + 1];
      auto v = p.point - q.point;
      v.normalize();
      auto angle = tangentAngle(p, q);
      auto wF = rotate(v, angle);
      auto wB = rotate(v, -angle);
      segsF.push_back({p.point + wF * p.r, q.point + wF * q.r});
      segsB.push_back({q.point + wB * q.r, p.point + wB * p.r});
    }

    auto *seg0 = &segsB[0];
    path.moveTo(seg0->q);
    //printf("p.moveTo({%f, %f});\n", seg0->q.fX, seg0->q.fY);
    for (size_t i = 0; i < 2 * segsF.size(); ++i) {
      LineSegment *seg1;
      Point *p;
      if (i < segsF.size()) {
        seg1 = &segsF[i];
        p = &points[i];
      } else {
        auto j = i - segsF.size();
        seg1 = &segsB[segsB.size() - 1 - j];
        p = &points[n - 1 - j];
      }
      if (SkPoint intersect; i > 0 && computeIntersection(*seg0, *seg1, intersect)) {
        path.lineTo(intersect);
        //printf("p.lineTo({%f, %f});\n", intersect.fX, intersect.fY);
      } else {
        if (i > 0) path.lineTo(seg0->q);
        auto a0 = seg0->angle();
        auto a1 = seg1->angle() + 180;
        path.arcTo(p->bounds(), a0 - 90, 180 - (a0 - a1), false);
        //printf("p.arcTo({%f, %f, %f, %f}, %f, %f, %d);\n",
        //       p->bounds().fLeft, p->bounds().fTop, p->bounds().fRight, p->bounds().fBottom,
        //       a0 - 90, 180 - (a0 - a1), false);
      }
      seg0 = seg1;
    }
    path.close();
  }

}

float Path::tangentAngle(const Path::Point &p, const Path::Point &q) {
  auto d = (q.point - p.point).length();
  auto r = q.r - p.r; // d > r
  return std::acos(r / d);
}

bool Path::computeIntersection(const LineSegment &s1, const LineSegment &s2, SkPoint &i) {
  //https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  const auto p0_x = s1.p.fX, p0_y = s1.p.fY;
  const auto p1_x = s1.q.fX, p1_y = s1.q.fY;
  const auto p2_x = s2.p.fX, p2_y = s2.p.fY;
  const auto p3_x = s2.q.fX, p3_y = s2.q.fY;

  const auto s1_x = p1_x - p0_x;
  const auto s1_y = p1_y - p0_y;
  const auto s2_x = p3_x - p2_x;
  const auto s2_y = p3_y - p2_y;

  const auto s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
  const auto t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    // Collision detected
    i = {p0_x + (t * s1_x), p0_y + (t * s1_y)};
    return true;
  }

  return false;
}

SkVector rotate(SkVector v, float angle) {
  auto cs = std::cos(angle);
  auto sn = std::sin(angle);
  return {
      v.fX * cs - v.fY * sn,
      v.fX * sn + v.fY * cs
  };
}

SkRect Path::Point::bounds() const {
  return SkRect::MakeXYWH(point.fX - r, point.fY - r, 2 * r, 2 * r);
}

bool Path::Point::contains(const Path::Point &o) const {
  return (point - o.point).length() < std::abs(r - o.r) + EPS;
}

float Path::LineSegment::angle() const {
  auto v = q - p;
  return rad2deg(std::atan2(v.fY, v.fX));
}
