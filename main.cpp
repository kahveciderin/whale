#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class Runner;

class ASTNode {
 public:
  virtual ~ASTNode() {}
  virtual void print(std::ostream &out) const = 0;
  virtual void run(Runner *runner, void *out = nullptr) const = 0;
};

class Runner {
 public:
  std::map<std::string, std::pair<ASTNode *, void *>> variables;
  Runner(ASTNode *ast) : ast_(ast) {}

  void run(void *out) { ast_->run(this, out); }
  void *alloc(int size) { return malloc(size); }

 private:
  ASTNode *ast_;
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

  virtual void run(Runner *runner, void *out) const {
    for (auto node : nodes_) {
      node->run(runner);
    }
  }

  std::vector<ASTNode *> nodes_;

 private:
};

class ASTType : public ASTNode {
 public:
  ASTType(const std::string &name) : name_(name) {}

  virtual void print(std::ostream &out) const { out << name_; }

  virtual void run(Runner *runner, void *out) const {
    if (name_ == "i32") {
      *(int *)out = sizeof(int);
    } else if (name_ == "i64") {
      *(int *)out = sizeof(long int);
    } else if (name_ == "float") {
      *(int *)out = sizeof(float);
    } else if (name_ == "double") {
      *(int *)out = sizeof(double);
    } else if (name_ == "void") {
      *(int *)out = sizeof(char);
    } else if (name_ == "bool") {
      *(int *)out = sizeof(bool);
    } else if (name_ == "char") {
      *(int *)out = sizeof(char);
    } else {
      throw std::runtime_error("Unknown type: " + name_);
    }
  }

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

  virtual void run(Runner *runner, void *out) const {
    *(int *)out = sizeof(void *);
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

  virtual void run(Runner *runner, void *out) const {
    int sizeOutput, typeOutput;
    size_->run(runner, &sizeOutput);
    type_->run(runner, &typeOutput);
    *(int *)out = sizeOutput * typeOutput;
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
    out << "TEMPLATE(";
    out << name_;
    out << ", ";
    type_->print(out);
    out << ")";
  }

  virtual void run(Runner *runner, void *out) const {
    *(int *)out = sizeof(void *);  // templates always have a pointer size
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

  virtual void run(Runner *runner, void *out) const {
    int bytes;
    type_->run(runner, &bytes);
    void *ptr = runner->alloc(bytes);
    std::cout << "Allocated " << bytes << " bytes for " << name_ << " on "
              << ptr << std::endl;
    runner->variables[name_] = std::make_pair(type_, ptr);

    *(void **)out = ptr;
  }

 private:
  ASTNode *type_;
  std::string name_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(int value) : value_(value) {}

  virtual void print(std::ostream &out) const { out << value_; }

  virtual void run(Runner *runner, void *out) const { *(int *)out = value_; }

 private:
  int value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value) : value_(value) {}

  virtual void print(std::ostream &out) const { out << '"' << value_ << '"'; }

  virtual void run(Runner *runner, void *out) const {}

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

  virtual void run(Runner *runner, void *out) const {
    int leftOutput, rightOutput;
    left_->run(runner, &leftOutput);
    right_->run(runner, &rightOutput);
    if (op_ == "+") {
      *(int *)out = leftOutput + rightOutput;
    } else if (op_ == "-") {
      *(int *)out = leftOutput - rightOutput;
    } else if (op_ == "*") {
      *(int *)out = leftOutput * rightOutput;
    } else if (op_ == "/") {
      *(int *)out = leftOutput / rightOutput;
    } else if (op_ == "%") {
      *(int *)out = leftOutput % rightOutput;
    } else if (op_ == "<<") {
      *(int *)out = leftOutput << rightOutput;
    } else if (op_ == ">>") {
      *(int *)out = leftOutput >> rightOutput;
    } else if (op_ == "&") {
      *(int *)out = leftOutput & rightOutput;
    } else if (op_ == "|") {
      *(int *)out = leftOutput | rightOutput;
    } else if (op_ == "^") {
      *(int *)out = leftOutput ^ rightOutput;
    } else if (op_ == "&&") {
      *(int *)out = leftOutput && rightOutput;
    } else if (op_ == "||") {
      *(int *)out = leftOutput || rightOutput;
    } else if (op_ == "==") {
      *(int *)out = leftOutput == rightOutput;
    } else if (op_ == "!=") {
      *(int *)out = leftOutput != rightOutput;
    } else if (op_ == "<") {
      *(int *)out = leftOutput < rightOutput;
    } else if (op_ == ">") {
      *(int *)out = leftOutput > rightOutput;
    } else if (op_ == "<=") {
      *(int *)out = leftOutput <= rightOutput;
    } else if (op_ == ">=") {
      *(int *)out = leftOutput >= rightOutput;
    } else {
      throw std::runtime_error("Unknown operator: " + op_);
    }
  }

 private:
  std::string op_;
  ASTNode *left_;
  ASTNode *right_;
};

class ASTLambda : public ASTNode {
 public:
  ASTLambda(ASTNodeList *args, ASTNode *body) : args_(args), body_(body) {}

  virtual ~ASTLambda() { delete body_; }

  virtual void print(std::ostream &out) const {
    out << "%(";
    args_->print(out);
    out << "){";
    body_->print(out);
    out << "}";
  }

  virtual void run(Runner *runner, void *out) const {
    std::cout << "LAMBDA address: " << this << std::endl;
    *(const ASTLambda **)out = this;
  }

  ASTNodeList *args_;
  ASTNode *body_;

 private:
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

  virtual void run(Runner *runner, void *out) const {
    int bytes;
    type_->run(runner, &bytes);
    void *ptr = runner->alloc(bytes);
    std::cout << "Allocated " << bytes << " bytes for " << name_ << " on "
              << ptr << std::endl;
    runner->variables[name_] = std::make_pair(type_, ptr);

    value_->run(runner, ptr);
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

  virtual void run(Runner *runner, void *out) const {}

 private:
  std::string name_;
};
class ASTVariableAssign : public ASTNode {
 public:
  ASTVariableAssign(const std::string &name, ASTNode *value)
      : name_(name), value_(value) {}

  virtual ~ASTVariableAssign() { delete value_; }

  virtual void print(std::ostream &out) const {
    out << name_;
    out << " = ";
    value_->print(out);
    out << ";";
  }

  virtual void run(Runner *runner, void *out) const {}

 private:
  std::string name_;
  ASTNode *value_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(const std::string &name, ASTNodeList *args)
      : name_(name), args_(args) {}

  virtual ~ASTFunctionCall() { delete args_; }

  virtual void print(std::ostream &out) const {
    out << name_;
    out << "(";
    args_->print(out);
    out << ")";
  }

  virtual void run(Runner *runner, void *out) const {
    void *lambdaAddress = std::get<void *>(runner->variables[name_]);
    std::cout << "Calling " << name_ << " @ " << lambdaAddress << std::endl;

  if(lambdaAddress == nullptr) {
    throw std::runtime_error("Unknown function: " + name_);
  }

    ASTLambda *lambda = *(ASTLambda **)lambdaAddress;

    int i = 0;
    for (auto arg : lambda->args_->nodes_) {
      void *variableAddress;
      arg->run(runner, &variableAddress);
      std::cout << "ARG: " << variableAddress << std::endl;
      i++;
    }

    lambda->body_->run(runner, out);
  }

 private:
  std::string name_;
  ASTNodeList *args_;
};

class Parser {
 public:
  Parser(std::istream &in) : in_(in) {}

  ASTNode *parse(bool standalone = true) {
    ASTNodeList *result = parseGlobalBody();
    if (in_.peek() != EOF) {
      throw std::runtime_error("Unexpected token");
    }
    result->add((new ASTFunctionCall("main", new ASTNodeList())));
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
    return new ASTVariableDecl(name, type, value);
  }

  ASTNode *parseType() {
    skipWhitespace();
    if (in_.peek() == EOF) {
      throw std::runtime_error("Unexpected EOF");
    }
    std::streampos oldpos = in_.tellg();
    std::string name = parseIdentifier();
    skipWhitespace();
    if (in_.peek() == '<') {
      in_.get();
      skipWhitespace();
      ASTNode *type = parseType();
      skipWhitespace();
      if (in_.peek() != '>') {
        throw std::runtime_error("Expected >");
      }
      in_.get();
      skipWhitespace();
      return new ASTTemplate(type, name);
    }
    std::streampos newpos = in_.tellg();
    in_.seekg(oldpos);

    if (in_.peek() == '*') {
      in_.get();
      return new ASTPointer(parseType());
    } else if (in_.peek() == '[') {
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
    in_.seekg(newpos);
    return new ASTType(name);
  }

  ASTNodeList *parseGlobalBody() {
    auto result = parseBody(false);
    return result;
  }

  ASTNodeList *parseBody(bool errorOnEOF = true) {
    ASTNodeList *result = new ASTNodeList();
    while (in_.peek() != '}') {
      if (in_.peek() == EOF) {
        if (errorOnEOF) {
          throw std::runtime_error("Unexpected EOF");
        } else {
          break;
        }
      }
      skipWhitespace();

      std::streampos oldpos = in_.tellg();

      enum {
        VAR_DECLARATION = 1,
        VAR_ASSIGNMENT = 2,
        EXPRESSION = 3,
      } expressionType = EXPRESSION;

      int _p = 0;
      while (in_.peek() != ';') {
        skipWhitespace();
        try {
          parseExpression();
        } catch (std::runtime_error &e) {
          expressionType = VAR_DECLARATION;
          break;
        }
        skipWhitespace();
        if (in_.peek() == '=') {
          expressionType = VAR_DECLARATION;
          break;
        }
        _p++;
      }
      if (_p == 0 && expressionType == EXPRESSION)
        expressionType = VAR_ASSIGNMENT;
      in_.seekg(oldpos);

      if (expressionType == VAR_DECLARATION) {
        result->add(parseVariableDecl());
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';'");
        }
      } else if (expressionType == VAR_ASSIGNMENT) {
        std::string name = parseIdentifier();
        if (in_.get() != '=') {
          throw std::runtime_error("Expected '='");
        }
        skipWhitespace();
        ASTNode *value = parseExpression();
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';'");
        }
        result->add(new ASTVariableAssign(name, value));
      } else {
        result->add(parseExpression());
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';'");
        }
      }

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

  ASTNodeList *parseFunctionArgs() {
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
    ASTNodeList *args = parseFunctionArgs();
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
      std::streampos oldpos = in_.tellg();
      std::string op = parseOperator();
      if (op != "=") {
        skipWhitespace();
        auto right = parseExpression();
        result = new ASTBinaryOp(op, result, right);
      } else {
        in_.seekg(oldpos);
        break;
      }
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

  int exitCode = 0;

  Runner runner(ast);
  runner.run(&exitCode);

  return exitCode;
}