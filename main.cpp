#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class ASTNode {
 public:
  virtual ~ASTNode() {}
  virtual void print(std::ostream &out) const = 0;
};

class ASTNodeList : public ASTNode {
 public:
  virtual ~ASTNodeList() {
    for (auto node : nodes_) {
      delete node;
    }
  }

  void add(ASTNode *node) { nodes_.push_back(node); }

  virtual void print(std::ostream &out) const {
    for (auto node : nodes_) {
      node->print(out);
    }
  }

 private:
  std::vector<ASTNode *> nodes_;
};

class ASTType : public ASTNode {
 public:
  ASTType(const std::string &name) : name_(name) {}

  virtual void print(std::ostream &out) const { out << name_; }

 private:
  std::string name_;
};
class ASTPointer : public ASTNode {
 public:
  ASTPointer(ASTNode *type) : type_(type) {}

  virtual void print(std::ostream &out) const {
    out << "POINTER(";
    type_->print(out);
    out << ")";
  }

 private:
  ASTNode *type_;
};
class ASTArray : public ASTNode {
 public:
  ASTArray(ASTNode *type, ASTNode *size) : type_(type), size_(size) {}

  virtual void print(std::ostream &out) const {
    out << "ARRAY(";
    size_->print(out);
    out << ", ";
    type_->print(out);
    out << ")";
  }

 private:
  ASTNode *type_;
  ASTNode *size_;
};
class ASTTemplate : public ASTNode {
 public:
  ASTTemplate(ASTNode *type, const std::string &name)
      : type_(type), name_(name) {}

  virtual void print(std::ostream &out) const {
    out << "<";
    out << name_;
    out << ">";
    type_->print(out);
  }

 private:
  ASTNode *type_;
  std::string name_;
};
class ASTFunctionArg : public ASTNode {
 public:
  ASTFunctionArg(ASTNode *type, const std::string &name)
      : type_(type), name_(name) {}

  virtual void print(std::ostream &out) const {
    out << name_;
    out << ":";
    type_->print(out);
  }

 private:
  ASTNode *type_;
  std::string name_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(int value) : value_(value) {}

  virtual void print(std::ostream &out) const { out << value_; }

 private:
  int value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value) : value_(value) {}

  virtual void print(std::ostream &out) const { out << '"' << value_ << '"'; }

 private:
  std::string value_;
};

class ASTBinaryOp : public ASTNode {
 public:
  ASTBinaryOp(std::string op, ASTNode *left, ASTNode *right)
      : op_(op), left_(left), right_(right) {}

  virtual ~ASTBinaryOp() {
    delete left_;
    delete right_;
  }

  virtual void print(std::ostream &out) const {
    out << "(";
    left_->print(out);
    out << " " << op_ << " ";
    right_->print(out);
    out << ")";
  }

 private:
  std::string op_;
  ASTNode *left_;
  ASTNode *right_;
};

class ASTLambda : public ASTNode {
 public:
  ASTLambda(ASTNode *args, ASTNode *body) : args_(args), body_(body) {}

  virtual ~ASTLambda() { delete body_; }

  virtual void print(std::ostream &out) const {
    out << "%(";
    args_->print(out);
    out << "){";
    body_->print(out);
    out << "}";
  }

 private:
  ASTNode *body_;
  ASTNode *args_;
};
class ASTVariableDecl : public ASTNode {
 public:
  ASTVariableDecl(const std::string &name, ASTNode *type, ASTNode *value)
      : name_(name), type_(type), value_(value) {}

  virtual ~ASTVariableDecl() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out) const {
    type_->print(out);
    out << " " << name_;
    out << " = ";
    value_->print(out);
    out << ";";
  }

 private:
  std::string name_;
  ASTNode *type_;
  ASTNode *value_;
};
class ASTVariable : public ASTNode {
 public:
  ASTVariable(const std::string &name) : name_(name) {}

  virtual void print(std::ostream &out) const { out << name_; }

 private:
  std::string name_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(const std::string &name, ASTNode *args)
      : name_(name), args_(args) {}

  virtual ~ASTFunctionCall() { delete args_; }

  virtual void print(std::ostream &out) const {
    out << name_;
    out << "(";
    args_->print(out);
    out << ")";
  }

 private:
  std::string name_;
  ASTNode *args_;
};

class Parser {
 public:
  Parser(std::istream &in) : in_(in) {}

  ASTNode *parse() {
    auto result = parseGlobalBody();
    if (in_.peek() != EOF) {
      throw std::runtime_error("Unexpected token");
    }
    return result;
  }

 private:
  void skipWhitespace() {
    while (isspace(in_.peek())) {
      in_.get();
    }
  }
  bool isValidIdentifierChar(char i, int pos = 0) {
    if (pos == 0) {
      return isalpha(i) || i == '_';
    } else {
      return isalnum(i) || i == '_';
    }
  }
  bool isValidOperatorChar(char i, int pos = 0) {
    return i == '+' || i == '-' || i == '*' || i == '/' || i == '%' ||
           i == '^' || i == '&' || i == '|' || i == '!' || i == '~' ||
           i == '<' || i == '>' || i == '=' || i == '?' || i == ':';
  }
  std::string parseIdentifier() {
    skipWhitespace();
    std::string result;
    int _p = 0;
    while (isValidIdentifierChar(in_.peek(), _p)) {
      result += in_.get();
      _p++;
    }
    skipWhitespace();
    return result;
  }
  std::string parseOperator() {
    skipWhitespace();
    std::string result;
    int _p = 0;
    while (isValidOperatorChar(in_.peek(), _p)) {
      result += in_.get();
      _p++;
    }
    skipWhitespace();
    return result;
  }
  std::string readLine() {
    std::string result;
    while (in_.peek() != ';') {
      result += in_.get();
    }
    in_.get();
    return result;
  }

  ASTNode *parseVariableDecl() {
    ASTNode *type = parseType();
    std::string name = parseIdentifier();
    ASTNode *value;
    if (in_.peek() == '=') {
      in_.get();
      skipWhitespace();
      value = parseExpression();
      skipWhitespace();
    } else {
      value = new ASTNumber(0);
    }
    if (in_.get() != ';') {
      throw std::runtime_error("Expected ';'");
    }
    return new ASTVariableDecl(name, type, value);
  }

  ASTNode *parseType() {
    skipWhitespace();
    if (in_.peek() == EOF) {
      throw std::runtime_error("Unexpected EOF");
    }
    if (in_.peek() == '*') {
      in_.get();
      return new ASTPointer(parseType());
    }
    if (in_.peek() == '[') {
      in_.get();
      if (in_.peek() == ']') {
        in_.get();
        return new ASTArray(parseType(), new ASTNumber(0));
      }
      ASTNode *size = parseExpression();
      if (in_.get() != ']') {
        throw std::runtime_error("Expected ']'");
      }
      return new ASTArray(parseType(), size);
    }
    if (in_.peek() == '<') {
      in_.get();
      std::string name;
      while (in_.peek() != '>') {
        name += in_.get();
      }
      in_.get();
      return new ASTTemplate(parseType(), name);
    }
    std::string name;
    int _p = 0;
    while (isValidIdentifierChar(in_.peek(), _p)) {
      name += in_.get();
      _p++;
    }
    return new ASTType(name);
  }

  ASTNode *parseGlobalBody() {
    auto result = new ASTNodeList();
    while (in_.peek() != EOF) {
      skipWhitespace();
      result->add(parseVariableDecl());
      skipWhitespace();
    }
    return result;
  }

  ASTNode *parseBody() {
    ASTNodeList *result = new ASTNodeList();
    while (in_.peek() != '}') {
      if (in_.peek() == EOF) {
        throw std::runtime_error("Unexpected EOF");
      }
      skipWhitespace();

      // debug
      std::streampos oldpos = in_.tellg();
      std::string line = readLine();
      std::cout << "line: " << line << std::endl;
      in_.seekg(oldpos);
      // end debug

      result->add(parseExpression());
      if (in_.peek() != ';') {
        throw std::runtime_error("Expected ';'");
      }
      in_.get();

      skipWhitespace();
    }
    return result;
  }

  ASTNode *parseFunctionArg() {
    std::string name;
    int _p = 0;
    while (isValidIdentifierChar(in_.peek(), _p)) {
      name += in_.get();
      _p++;
    }
    skipWhitespace();
    if (in_.peek() != ':') {
      throw std::runtime_error("Expected ':'");
    }
    in_.get();
    skipWhitespace();
    ASTNode *type = parseType();
    skipWhitespace();
    return new ASTFunctionArg(type, name);
  }

  ASTNode *parseFunctionArgs() {
    ASTNodeList *result = new ASTNodeList();
    while (in_.peek() != '{') {
      result->add(parseFunctionArg());
      skipWhitespace();

      if (in_.peek() == ',') {
        in_.get();
      } else {
      }
    }
    skipWhitespace();
    return result;
  }

  ASTNode *parseLambda() {
    if (in_.get() != '%') {
      throw std::runtime_error("Expected '%'");
    }
    skipWhitespace();
    ASTNode *args = parseFunctionArgs();
    if (in_.get() != '{') {
      throw std::runtime_error("Expected '{'");
    }
    ASTNode *body = parseBody();
    if (in_.get() != '}') {
      throw std::runtime_error("Expected '}'");
    }
    return new ASTLambda(args, body);
  }

  ASTNode *parseExpression() {
    ASTNode *result;

    if (in_.peek() == '%') {
      skipWhitespace();
      result = parseLambda();
      skipWhitespace();
    } else if (isValidIdentifierChar(in_.peek())) {
      std::string name = parseIdentifier();
      skipWhitespace();
      if (in_.peek() == '(') {
        in_.get();
        ASTNodeList *args = new ASTNodeList();
        while (in_.peek() != ')') {
          skipWhitespace();
          args->add(parseExpression());
          skipWhitespace();
          if (in_.peek() == ',') {
            in_.get();
          } else {
          }
        }
        in_.get();

        result = new ASTFunctionCall(name, args);
      } else {
        result = new ASTVariable(name);
      }
      skipWhitespace();
    } else {
      result = parseTerm();
    }
    skipWhitespace();
    while (isValidOperatorChar(in_.peek())) {
      std::string op = parseOperator();
      skipWhitespace();
      auto right = parseExpression();
      result = new ASTBinaryOp(op, result, right);
    }
    return result;
  }

  ASTNode *parseTerm() {
    auto result = parseFactor();
    return result;
  }

  ASTNode *parseFactor() {
    if (in_.peek() == '(') {
      in_.get();
      auto result = parseExpression();
      if (in_.get() != ')') {
        throw std::runtime_error("Expected ')'");
      }
      return result;
    } else if (isdigit(in_.peek())) {
      int value = 0;
      while (isdigit(in_.peek())) {
        value = value * 10 + in_.get() - '0';
      }
      return new ASTNumber(value);
    } else if (in_.peek() == '"') {
      in_.get();
      std::string value;
      while (in_.peek() != '"') {
        value += in_.get();
      }
      in_.get();
      return new ASTString(value);
    } else {
      throw std::runtime_error("Unexpected token");
    }
  }

  std::istream &in_;
};

int main() {
  std::ifstream file("tests/initial.wha");
  std::istream &code = static_cast<std::istream &>(file);

  Parser parser(code);
  auto ast = parser.parse();

  ast->print(std::cout);
  std::cout << std::endl;
}