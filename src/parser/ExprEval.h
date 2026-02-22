#pragma once

#include <optional>
#include <string>

/**
 * any numeric field in the scene JSON can be given
 * as a string expression, "sin(t)", "2*t", "1 + 0.5*cos(t)".
 *
 * numbers, t, +, -, *, /, sin, cos, tan, sqrt, pow, pi, ().
 */
namespace ExprEval {

std::optional<double> eval(const std::string &expr, double t);

bool isExpression(const std::string &s);

} // namespace ExprEval
