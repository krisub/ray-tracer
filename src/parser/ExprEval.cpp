#include "ExprEval.h"
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

namespace ExprEval {

namespace {

const double PI = 3.14159265358979323846;

class Parser {
public:
  Parser(const std::string &input, double t_val) : s(input), t(t_val), pos(0) {}

  std::optional<double> parse() {
    skipSpace();
    auto v = parseExpr();
    if (!v.has_value())
      return std::nullopt;
    skipSpace();
    if (pos != (int)s.size())
      return std::nullopt; // trailing junk
    return v;
  }

private:
  const std::string &s;
  double t;
  int pos;

  void skipSpace() {
    while (pos < (int)s.size() && std::isspace((unsigned char)s[pos]))
      ++pos;
  }

  bool peek(const char *token) {
    skipSpace();
    int len = 0;
    while (token[len] && pos + len < (int)s.size() && s[pos + len] == token[len])
      ++len;
    if (!token[len])
      return true;
    return false;
  }

  bool consume(const char *token) {
    if (!peek(token))
      return false;
    while (*token)
      ++pos, ++token;
    return true;
  }

  bool isIdentifierChar(char c) {
    return std::isalnum((unsigned char)c) || c == '_';
  }

  std::optional<double> parseExpr() {
    auto left = parseTerm();
    if (!left.has_value())
      return std::nullopt;
    for (;;) {
      skipSpace();
      if (pos >= (int)s.size())
        return left;
      if (s[pos] == '+') {
        ++pos;
        auto right = parseTerm();
        if (!right.has_value())
          return std::nullopt;
        *left = *left + *right;
      } else if (s[pos] == '-') {
        ++pos;
        auto right = parseTerm();
        if (!right.has_value())
          return std::nullopt;
        *left = *left - *right;
      } else
        return left;
    }
  }

  std::optional<double> parseTerm() {
    auto left = parseFactor();
    if (!left.has_value())
      return std::nullopt;
    for (;;) {
      skipSpace();
      if (pos >= (int)s.size())
        return left;
      if (s[pos] == '*') {
        ++pos;
        auto right = parseFactor();
        if (!right.has_value())
          return std::nullopt;
        *left = *left * *right;
      } else if (s[pos] == '/') {
        ++pos;
        auto right = parseFactor();
        if (!right.has_value())
          return std::nullopt;
        if (*right == 0.0)
          return std::nullopt;
        *left = *left / *right;
      } else
        return left;
    }
  }

  std::optional<double> parseFactor() {
    skipSpace();
    if (pos >= (int)s.size())
      return std::nullopt;
    if (s[pos] == '-') {
      ++pos;
      auto v = parseFactor();
      if (!v.has_value())
        return std::nullopt;
      return -(*v);
    }
    if (s[pos] == '+') {
      ++pos;
      return parseFactor();
    }
    return parsePrimary();
  }

  std::optional<double> parsePrimary() {
    skipSpace();
    if (pos >= (int)s.size())
      return std::nullopt;
    if (s[pos] == '(') {
      ++pos;
      auto v = parseExpr();
      if (!v.has_value())
        return std::nullopt;
      skipSpace();
      if (pos >= (int)s.size() || s[pos] != ')')
        return std::nullopt;
      ++pos;
      return v;
    }
    if (std::isdigit((unsigned char)s[pos]) || (s[pos] == '.' && pos + 1 < (int)s.size() && std::isdigit((unsigned char)s[pos + 1]))) {
      char *end = nullptr;
      double v = std::strtod(s.c_str() + pos, &end);
      if (end == s.c_str() + pos)
        return std::nullopt;
      pos = (int)(end - s.c_str());
      return v;
    }
    if (consume("t") || consume("T")) {
      if (pos < (int)s.size() && isIdentifierChar(s[pos]))
        return std::nullopt;
      return t;
    }
    if (consume("pi") || consume("PI")) {
      if (pos < (int)s.size() && isIdentifierChar(s[pos]))
        return std::nullopt;
      return PI;
    }
    if (consume("sin")) {
      if (!consume("("))
        return std::nullopt;
      auto v = parseExpr();
      if (!v.has_value() || !consume(")"))
        return std::nullopt;
      return std::sin(*v);
    }
    if (consume("cos")) {
      if (!consume("("))
        return std::nullopt;
      auto v = parseExpr();
      if (!v.has_value() || !consume(")"))
        return std::nullopt;
      return std::cos(*v);
    }
    if (consume("tan")) {
      if (!consume("("))
        return std::nullopt;
      auto v = parseExpr();
      if (!v.has_value() || !consume(")"))
        return std::nullopt;
      return std::tan(*v);
    }
    if (consume("sqrt")) {
      if (!consume("("))
        return std::nullopt;
      auto v = parseExpr();
      if (!v.has_value() || !consume(")"))
        return std::nullopt;
      if (*v < 0.0)
        return std::nullopt;
      return std::sqrt(*v);
    }
    if (consume("pow")) {
      if (!consume("("))
        return std::nullopt;
      auto a = parseExpr();
      if (!a.has_value())
        return std::nullopt;
      skipSpace();
      if (pos >= (int)s.size() || s[pos] != ',')
        return std::nullopt;
      ++pos;
      auto b = parseExpr();
      if (!b.has_value() || !consume(")"))
        return std::nullopt;
      return std::pow(*a, *b);
    }
    if (consume("abs")) {
      if (!consume("("))
        return std::nullopt;
      auto v = parseExpr();
      if (!v.has_value() || !consume(")"))
        return std::nullopt;
      return std::abs(*v);
    }
    return std::nullopt;
  }
};

} // namespace

std::optional<double> eval(const std::string &expr, double t) {
  if (expr.empty())
    return std::nullopt;
  Parser p(expr, t);
  return p.parse();
}

bool isExpression(const std::string &s) {
  if (s.empty())
    return false;
  for (char c : s) {
    if (std::isalnum((unsigned char)c) || c == '.' || c == '+' || c == '-' ||
        c == '*' || c == '/' || c == '(' || c == ')' || c == ',' ||
        c == ' ' || c == '\t' || c == '_')
      continue;
    return false;
  }
  return true;
}

} // namespace ExprEval
