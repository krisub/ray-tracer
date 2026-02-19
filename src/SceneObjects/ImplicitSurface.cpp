#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iostream>

#include "ImplicitSurface.h"
#include "../scene/ray.h"

namespace {
constexpr double THRESHOLD = 1e-5;
constexpr double EPSILON = 1e-8;
constexpr double NORMAL_EPS = 1e-6;
constexpr int MAX_STEPS = 1024;

double wrapAngle(double theta) {
    theta = fmod(theta, 2.0 * M_PI);
    if (theta < 0.0) theta += 2.0 * M_PI;
    return theta;
}

// evaluate the k-th branch of Im[z^(n/d)] - p.z
// phi_k(x,y,z) = r^alpha * sin(alpha*(theta + 2pi*k)) - z
double evalRiemannBranch(const glm::dvec3 &p, int k, int n, int d) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    double theta = std::atan2(p.y, p.x);
    if (theta < 0.0) theta += 2.0 * M_PI;

    double theta_k = theta + 2.0 * M_PI * k;
    double alpha = (double)n / (double)d;

    return std::pow(r, alpha) * std::sin(alpha * theta_k) - p.z;
}

// gradient of phi_k wrt (x,y,z).
// phi_k = r^alpha * sin(alpha*theta_k) - z, where theta_k = atan2(y,x) + 2pi*k
// dphi/dx = alpha*r^(alpha-2)*( x*sin(alpha*theta_k) - y*cos(alpha*theta_k) )
// dphi/dy = alpha*r^(alpha-2)*( y*sin(alpha*theta_k) + x*cos(alpha*theta_k) )
// dphi/dz = -1
glm::dvec3 gradientRiemannBranch(const glm::dvec3 &p, int k, int n, int d) {
    const double r2 = p.x * p.x + p.y * p.y;
    const double r  = std::sqrt(r2);
    if (r < 1e-12) {
        const double h = 1e-5;
        double gx = evalRiemannBranch({p.x+h,p.y,p.z},k,n,d)
                  - evalRiemannBranch({p.x-h,p.y,p.z},k,n,d);
        double gy = evalRiemannBranch({p.x,p.y+h,p.z},k,n,d)
                  - evalRiemannBranch({p.x,p.y-h,p.z},k,n,d);
        return glm::dvec3(gx, gy, -2.0*h) / (2.0*h);
    }

    double theta = std::atan2(p.y, p.x);
    if (theta < 0.0) theta += 2.0 * M_PI;
    double theta_k = theta + 2.0 * M_PI * k;
    double alpha = (double)n / (double)d;

    double s = std::sin(alpha * theta_k);
    double c = std::cos(alpha * theta_k);
    double scale = alpha * std::pow(r, alpha - 2.0);

    double gx = scale * ( p.x * s - p.y * c);
    double gy = scale * ( p.y * s + p.x * c);
    double gz = -1.0;
    return glm::dvec3(gx, gy, gz);
}

bool refineRiemannHit(const glm::dvec3 &o, const glm::dvec3 &dVec,
                      double &ta, double &tb, int k, int n, int d) {
    double fa = evalRiemannBranch(o + ta * dVec, k, n, d);

    for (int it = 0; it < 32; ++it) {
        const double tm = 0.5 * (ta + tb);
        const double fm = evalRiemannBranch(o + tm * dVec, k, n, d);

        if (std::abs(fm) < 1e-8) { ta = tm; return true; }

        if (fa * fm < 0.0) { tb = tm; }
        else               { ta = tm; fa = fm; }
    }
    ta = 0.5 * (ta + tb);
    return true;
}

} // namespace

//  harnack tracing for Riemann surface height fields
//  algorithm 1 from Gillespie et al. 2024, Section 4.6
bool ImplicitSurface::harnackTrace(ray &r, isect &i, double tMin, double tMax) const {
    double t = std::max(tMin, 1e-3);
    const glm::dvec3 o    = r.getPosition();
    const glm::dvec3 dVec = r.getDirection();   // assumed unit length

    const int    n     = riemann_n;
    const int    d     = riemann_d;
    const double alpha = (double)n / (double)d;

    // stopping tolerance epsilon  (gradient-normalised)
    constexpr double EPS_STOP = 1e-5;
    // ZERO level set of phi_k(x,y,z) = Im[z^(n/d)]_k − z_3d
    constexpr double F_STAR = 0.0;

    for (int step = 0; step < 4000 && t < tMax; ++step) {

        const glm::dvec3 p    = o + t * dVec;
        const double     rCyl = std::sqrt(p.x*p.x + p.y*p.y);

        // R(x): distance to the nearest singularity (the z-axis)
        // paper says "we set R(x) to the distance from x to the closest singularity"
        const double R_t = std::max(1e-6, rCyl);

        // c(x): valid lower bound on phi_k throughout B_R(p)
        //
        // within B_R(p), any point y satisfies:
        //   r'  le rCyl + R_t       (cylindrical radius)
        //   |y.z| le |p.z| + R_t
        //
        // so:  phi_k(y) = r'^alpha · sin(alpha*theta'_k) − y.z
        //             ge  −(rCyl + R_t)^alpha − (|p.z| + R_t)
        //
        // tighter than the paper's constant c = −4
        const double c_t = -(std::pow(rCyl + R_t, alpha) + std::abs(p.z) + R_t);

        // stopping check |phi_k| / len(change in phi_k) < eps
        int hitBranch = -1;
        for (int k = 0; k < d; ++k) {
            double phi_k = evalRiemannBranch(p, k, n, d);
            glm::dvec3 g = gradientRiemannBranch(p, k, n, d);
            double gLen  = glm::length(g);

            double stopVal = (gLen > 1e-10) ? std::abs(phi_k) / gLen
                                            : std::abs(phi_k);
            if (stopVal < EPS_STOP) {
                hitBranch = k;
                break;
            }
        }

        if (hitBranch >= 0) {
            if (rCyl > 1.5) {
                t += 0.1;
                continue;
            }

            glm::dvec3 g = gradientRiemannBranch(p, hitBranch, n, d);
            if (glm::length(g) > 1e-9) g = glm::normalize(g);
            else                        g = glm::dvec3(0, 0, 1);
            if (glm::dot(g, dVec) > 0)  g = -g;

            i.setObject(this);
            i.setMaterial(this->getMaterial());
            i.setT(t);
            i.setN(g);
            i.setUVCoordinates(glm::dvec2(0.5, 0.5));
            return true;
        }

        // harnack step size algorithm 1
        //
        // for each branch k, the shifted target is F_STAR − c_t  (> 0 since
        // F_STAR = 0 and c_t < 0).  eqn7: a = (f_t − c_t) / (f* − c_t)
        // p_k = R/2 · |a + 2 − sqrt(a^2 + 8a)|

        double minRho = R_t;

        for (int k = 0; k < d; ++k) {
            double f_t = evalRiemannBranch(p, k, n, d);

            if (F_STAR <= c_t) {
                continue;
            }

            if (f_t <= c_t) {
                minRho = std::min(minRho, 0.001 * R_t);
                continue;
            }

            double a    = (f_t - c_t) / (F_STAR - c_t);
            double rho_k = 0.5 * R_t * std::abs(a + 2.0 - std::sqrt(a*a + 8.0*a));

            minRho = std::min(minRho, rho_k);
        }

        minRho = std::clamp(minRho, 1e-7, R_t);
        t += minRho;
    }

    return false;   // no intersection within tMax
}

ImplicitSurface::ImplicitSurface(Scene *scene, Material *mat, Type type)
    : SceneObject(scene, mat), type_(type) {}

double ImplicitSurface::eval(glm::dvec3 p) const {
    const double x = p.x, y = p.y, z = p.z;
    switch (type_) {
    case Type::Sphere:
        return glm::length(p) - radius_;
    case Type::Torus:
        return glm::length(glm::dvec2(glm::length(glm::dvec2(x,y)) - major_radius_, z)) - minor_radius_;
    case Type::Goursat: {
        double r2 = x*x + y*y + z*z;
        return x*x*x*x + y*y*y*y + z*z*z*z - goursat_a_*r2*r2 + goursat_b_*r2 - goursat_c_;
    }
    case Type::Blob: {
        const double b1_2 = blob_radius1_*blob_radius1_;
        const double b2_2 = blob_radius2_*blob_radius2_;
        auto soft = [](double r2, double b2){
            if (r2 >= b2) return 0.0;
            double t = 1.0 - r2/b2; return t*t*t;
        };
        double r1_2 = glm::dot(p - blob_c1_, p - blob_c1_);
        double r2_2 = glm::dot(p - blob_c2_, p - blob_c2_);
        return soft(r1_2,b1_2) + soft(r2_2,b2_2) - blob_threshold_;
    }
    case Type::Riemann:
        return evalRiemannBranch(p, 0, riemann_n, riemann_d);
    }
    return 0;
}

glm::dvec3 ImplicitSurface::gradient(glm::dvec3 p) const {
    const double x = p.x, y = p.y, z = p.z;
    switch (type_) {
    case Type::Sphere: {
        double len = glm::length(p);
        return len > 1e-10 ? p/len : glm::dvec3(1,0,0);
    }
    case Type::Torus: {
        double xy_len = glm::length(glm::dvec2(x,y));
        if (xy_len < 1e-12) return glm::dvec3(0,0,z>0?1:-1);
        double qx = xy_len - major_radius_;
        glm::dvec2 q(qx,z);
        double q_len = glm::length(q);
        if (q_len < 1e-12) return glm::dvec3(1,0,0);
        double scale = (1.0 - major_radius_/xy_len)/q_len;
        return glm::normalize(glm::dvec3(scale*x, scale*y, z/q_len));
    }
    case Type::Goursat: {
        double r2 = x*x + y*y + z*z;
        double factor = -2.0*goursat_a_*r2 + goursat_b_;
        return glm::dvec3(4.0*x*x*x + 2.0*x*factor,
                          4.0*y*y*y + 2.0*y*factor,
                          4.0*z*z*z + 2.0*z*factor);
    }
    default: {
        const double h = NORMAL_EPS;
        return glm::dvec3(
            (eval(p+glm::dvec3(h,0,0)) - eval(p-glm::dvec3(h,0,0))) / (2.0*h),
            (eval(p+glm::dvec3(0,h,0)) - eval(p-glm::dvec3(0,h,0))) / (2.0*h),
            (eval(p+glm::dvec3(0,0,h)) - eval(p-glm::dvec3(0,0,h))) / (2.0*h));
    }
    }
}

double ImplicitSurface::lipschitzConstant() const {
    if (lipschitz_override_ > 0.0) return lipschitz_override_;
    switch (type_) {
    case Type::Sphere:   return 2.5;
    case Type::Torus:    return 1.0;
    case Type::Goursat:  return 14.0;
    case Type::Blob: {
        double minR = std::min(blob_radius1_, blob_radius2_);
        return std::max(4.0, 1.0/(minR*minR));
    }
    case Type::Riemann:  return 8.0;
    }
    return 4.0;
}

bool ImplicitSurface::sphereTrace(ray &r, isect &i, double tMin, double tMax) const {
    const double L = lipschitzConstant();
    double t = tMin;
    const glm::dvec3 o = r.getPosition();
    const glm::dvec3 d = r.getDirection();

    for (int step = 0; step < MAX_STEPS && t < tMax; ++step) {
        glm::dvec3 p = o + t*d;
        double f = eval(p);

        if (std::abs(f) < EPSILON) {
            glm::dvec3 g = gradient(p);
            double gLen = glm::length(g);
            glm::dvec3 n = gLen > 1e-10 ? g/gLen : glm::dvec3(1,0,0);
            if (glm::dot(n,d) > 0) n = -n;

            i.setObject(this);
            i.setMaterial(this->getMaterial());
            i.setT(t);
            i.setN(n);
            i.setUVCoordinates(glm::dvec2(0.5, 0.5));
            return true;
        }

        t += std::abs(f) / L;
    }
    return false;
}

bool ImplicitSurface::intersectLocal(ray &r, isect &i) const {
    r.setDirection(glm::normalize(r.getDirection()));

    double tMin, tMax;
    BoundingBox localBox = getLocalBoundingBoxImpl();
    if (!localBox.intersect(r, tMin, tMax)) return false;
    if (tMin < RAY_EPSILON) tMin = RAY_EPSILON;

    if (type_ == Type::Riemann)
        return harnackTrace(r, i, tMin, tMax);

    return sphereTrace(r, i, tMin, tMax);
}

BoundingBox ImplicitSurface::getLocalBoundingBoxImpl() const {
    BoundingBox b;
    switch (type_) {
    case Type::Sphere:
        b.setMin(glm::dvec3(-radius_,-radius_,-radius_));
        b.setMax(glm::dvec3( radius_, radius_, radius_));
        break;
    case Type::Torus: {
        double extent = major_radius_ + minor_radius_;
        b.setMin(glm::dvec3(-extent,-extent,-minor_radius_));
        b.setMax(glm::dvec3( extent, extent, minor_radius_));
        break;
    }
    case Type::Goursat: {
        double extent = 1.5;
        b.setMin(glm::dvec3(-extent,-extent,-extent));
        b.setMax(glm::dvec3( extent, extent, extent));
        break;
    }
    case Type::Blob: {
        double r = std::max(blob_radius1_, blob_radius2_) + 0.1;
        glm::dvec3 lo = glm::min(blob_c1_, blob_c2_) - glm::dvec3(r);
        glm::dvec3 hi = glm::max(blob_c1_, blob_c2_) + glm::dvec3(r);
        b.setMin(lo); b.setMax(hi);
        break;
    }
    case Type::Riemann: {
        double extent = 4.0;
        b.setMin(glm::dvec3(-extent,-extent,-extent));
        b.setMax(glm::dvec3( extent, extent, extent));
        break;
    }
    }
    return b;
}

BoundingBox ImplicitSurface::ComputeLocalBoundingBox() {
    return getLocalBoundingBoxImpl();
}

void ImplicitSurface::glDrawLocal(int, bool, bool) const {}