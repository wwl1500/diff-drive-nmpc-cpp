#pragma once

struct State {
    double x;
    double y;
    double theta;
};

struct Control {
    double v;
    double omega;
};

struct Reference {
    double x;
    double y;
    double theta;
};
