#pragma once

#include <optional>
#include <string>

/**
 * Evaluates mathematical expressions in the variable t (time).
 * Used for animation: any numeric field in the scene JSON can be given
 * as a string expression, e.g. "sin(t)", "2*t", "1 + 0.5*cos(t)".
 *
 * Supported: numbers, t, +, -, *, /, sin, cos, tan, sqrt, pow, pi, ().
 * Unary minus is supported.
 */
namespace ExprEval {

/** Evaluate expression with given time t. Returns nullopt on parse error. */
std::optional<double> eval(const std::string &expr, double t);

/** Returns true if the string looks like an expression (would be evaluated). */
bool isExpression(const std::string &s);

} // namespace ExprEval
