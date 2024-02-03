#ifndef MODEL_H
#define MODEL_H

struct PIDModel {
    double proportional{0};
    double integral{0};
    double derivative{0};
    double system_constrained_total{0};
};

enum class DiffSide{
    LEFT, RIGHT
};
#endif