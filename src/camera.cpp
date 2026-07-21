#define _USE_MATH_DEFINES
#include "camera.hpp"
#include <cmath>
#include <algorithm>

const double RADIAN_CONVERSION = M_PI / 180.0;

Camera::Camera(double p_sens, double s_sens, double w, double h) : pan_sensitivity(p_sens), scroll_sensitivity(s_sens), width(w), height(h) {}

void Camera::camera_pan_handler(double xcurrent, double ycurrent) {

    if (active_pan) {
        
        double deltax = xcurrent - mouse_x;
        double deltay = ycurrent - mouse_y;

        theta = theta + ((pan_sensitivity * 2 / width) * deltax);
        phi = phi + ((pan_sensitivity * 2 / height) * deltay);
        phi = std::max(-89.0 * RADIAN_CONVERSION, std::min(89.0 * RADIAN_CONVERSION, phi));
    }
}

void Camera::camera_scroll_handler(double yoffset) {
    r = r - (scroll_sensitivity * yoffset);
}

void Camera::mouse_pos(double x, double y) {
    mouse_x = x;
    mouse_y = y;
}

void Camera::activate_pan() {
    active_pan = true;
}

void Camera::deactivate_pan() {
    active_pan = false;
}

double Camera::get_r() const {
    return r;
}

double Camera::get_theta() const {
    return theta;
}

double Camera::get_phi() const {
    return phi;
}