#pragma once

struct Camera {
private:
    double mouse_x;
    double mouse_y;
    double theta = 0.0;
    double phi = 0.0;
    double r = 100.0;
    double pan_sensitivity;
    double scroll_sensitivity;

    double width;
    double height;

    bool active_pan = false;
public:
    explicit Camera(double p_sens, double s_sens, double w, double h);
    void camera_pan_handler(double xcurrent, double ycurrent);
    void camera_scroll_handler(double yoffset);

    void mouse_pos(double x, double y);

    void activate_pan();
    void deactivate_pan();

    double get_r() const;
    double get_theta() const;
    double get_phi() const;
};