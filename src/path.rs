use std::f64::consts::{FRAC_PI_2, PI, TAU};

use vello::kurbo::{Arc, BezPath, Circle, Line, Point, Shape, Vec2};

const EPS: f64 = 1e-3;

fn rotate(v: Vec2, angle: f64) -> Vec2 {
    let cs = angle.cos();
    let sn = angle.sin();
    Vec2 {
        x: v.x * cs - v.y * sn,
        y: v.x * sn + v.y * cs,
    }
}

#[derive(Default)]
pub struct PointPath {
    pub points: Vec<PointData>,
}

impl PointPath {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    pub fn add_point(&mut self, x: f64, y: f64, r: f64) {
        let p = PointData {
            point: Point { x, y },
            r,
        };
        while !self.points.is_empty() && p.contains(self.points.last().unwrap()) {
            self.points.pop();
        }
        if !self.points.is_empty() && self.points.last().unwrap().contains(&p) {
            return;
        }
        self.points.push(p);
    }

    // TODO does not look 100% right yet...
    pub fn bez_path(&mut self) -> BezPath {
        let n = self.points.len();
        if n == 0 {
            return BezPath::new();
        } else if n == 1 {
            let p = &self.points[0];
            let c = Circle::new(p.point, p.r);
            return c.to_path(0.1);
        }

        let mut segs_f = Vec::new();
        let mut segs_b = Vec::new();
        for i in 0..n - 1 {
            let p = &self.points[i];
            let q = &self.points[i + 1];
            let v = (p.point - q.point).normalize();
            let angle = tangent_angle(p, q);
            let w_f = rotate(v, angle);
            let w_b = rotate(v, -angle);
            segs_f.push(Line {
                p0: p.point + w_f * p.r,
                p1: q.point + w_f * q.r,
            });
            segs_b.push(Line {
                p0: q.point + w_b * q.r,
                p1: p.point + w_b * p.r,
            });
        }

        let mut seg0 = segs_b.first().unwrap();
        let mut path = BezPath::new();
        path.move_to(seg0.p1);
        for i in 0..2 * segs_f.len() {
            let (seg1, p) = if i < segs_f.len() {
                (&segs_f[i], &self.points[i])
            } else {
                let j = i - segs_f.len();
                (&segs_b[segs_b.len() - 1 - j], &self.points[n - 1 - j])
            };
            let mut is_intersection = false;
            if i > 0 {
                if let Some(intersect) = compute_intersection(seg0, seg1) {
                    path.line_to(intersect);
                    is_intersection = true;
                } else {
                    path.line_to(seg0.p1);
                }
            }
            if !is_intersection {
                let a0 = line_angle(seg0);
                let a1 = line_angle(seg1) + PI;
                let mut sweep = PI - (a0 - a1);
                if sweep >= PI {
                    sweep -= TAU;
                }
                let arc = Arc::new(p.point, (p.r, p.r), a0 - FRAC_PI_2, sweep, 0.);
                for b in arc.append_iter(0.1) {
                    path.push(b);
                }
            }
            seg0 = seg1;
        }
        path.close_path();
        path
    }

    pub fn reset(&mut self) {
        self.points.clear();
    }
}

#[derive(Debug)]
pub struct PointData {
    pub point: Point,
    pub r: f64,
}

impl PointData {
    fn contains(&self, other: &PointData) -> bool {
        let d = self.point.distance(other.point);
        d + other.r <= self.r + EPS
    }
}

fn tangent_angle(p: &PointData, q: &PointData) -> f64 {
    let d = p.point.distance(q.point);
    let r = q.r - p.r; // d > r
    (r / d).acos()
}

fn compute_intersection(s1: &Line, s2: &Line) -> Option<Point> {
    // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
    let p0 = s1.p0;
    let p1 = s1.p1;
    let p2 = s2.p0;
    let p3 = s2.p1;

    let s1_x = p1.x - p0.x;
    let s1_y = p1.y - p0.y;
    let s2_x = p3.x - p2.x;
    let s2_y = p3.y - p2.y;

    let s = (-s1_y * (p0.x - p2.x) + s1_x * (p0.y - p2.y)) / (-s2_x * s1_y + s1_x * s2_y);
    let t = (s2_x * (p0.y - p2.y) - s2_y * (p0.x - p2.x)) / (-s2_x * s1_y + s1_x * s2_y);

    if s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0 {
        let intersection: Point = Point {
            x: p0.x + (t * s1_x),
            y: p0.y + (t * s1_y),
        };
        Some(intersection)
    } else {
        None
    }
}

fn line_angle(l: &Line) -> f64 {
    let v = l.p1 - l.p0;
    v.y.atan2(v.x)
}
