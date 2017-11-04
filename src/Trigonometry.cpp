//
// Created by Andreas Ntalakas on 04/11/2017.
//

#include "Trigonometry.h"

Trigonometry::Trigonometry() {
}

Trigonometry::~Trigonometry() {

}

double Trigonometry::deg2rad(double x) { return x * pi() / 180; }

double Trigonometry::rad2deg(double x) { return x * 180 / pi(); }