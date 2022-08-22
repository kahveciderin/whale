#include <cctype>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

class ASTNodeList;
class Runner;
class RunnerStackFrame;

class ASTNode {
 public:
  virtual ~ASTNode() {}
  virtual void print(std::ostream &out, int level = 0) const = 0;
  std::string indent(int level) const {
    std::string s;
    for (int i = 0; i < level; ++i) {
      s += "  ";
    }
    return s;
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out = nullptr) const = 0;
};

class ASTType : public ASTNode {
 public:
  ASTType(const std::string &name) : name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Type " << name_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    if (name_ == "i32") {
      *(int *)out = sizeof(int);
    } else if (name_ == "i64") {
      *(int *)out = sizeof(long int);
    } else if (name_ == "fun") {
      *(int *)out = sizeof(void *);
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

class Runner {
 public:
  bool _return = false;
  Runner(ASTNode *ast);

  void run(void *out);
  void *alloc(int size);

  void generateFunction(std::string name, ASTNodeList *args,
                        void (*body)(Runner *, RunnerStackFrame *),
                        ASTNode *ret = new ASTType("void"));

  void gc();

  void enterFrame();
  void exitFrame();

 private:
  ASTNode *ast_;
  RunnerStackFrame *stackFrame_;
};

class RunnerStackFrame {
 public:
  RunnerStackFrame(RunnerStackFrame *parent = nullptr) : parent_(parent) {}
  ~RunnerStackFrame() {
    for (auto &kv : variables_) {
      free(kv.second);
    }
  }

  void *getVariable(const std::string &name) {
    if (this->variables_[name] != nullptr) return this->variables_[name];
    if (this->parent_ != nullptr) return this->parent_->getVariable(name);
    return nullptr;
  };
  template <typename T>
  T getVariable(const std::string &name) {
    return *(T *)this->getVariable(name);
  };
  void *allocVariable(const std::string &name, ASTNode *type, Runner *runner) {
    int size;
    type->run(runner, this, &size);
    void *value = runner->alloc(size);
    this->variables_[name] = value;
    return value;
  };

  RunnerStackFrame *parent_;
  std::map<std::string, void *> variables_;  // name -> value
};

class ASTNodeList : public ASTNode {
 public:
  ASTNodeList(const std::vector<ASTNode *> &nodes) : nodes_(nodes) {}
  ASTNodeList() {}

  virtual ~ASTNodeList() {
    for (auto node : nodes_) {
      delete node;
    }
  }

  void add(ASTNode *node) { nodes_.push_back(node); }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "NodeList:\n";
    for (auto node : nodes_) {
      node->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    RunnerStackFrame newStackFrame(stackFrame);

    unsigned long trash_ = 0;
    for (auto node : nodes_) {
      node->run(runner, &newStackFrame, &trash_);
      if (runner->_return) {
        *(unsigned long *)out = trash_;
        break;
      }
    }
    runner->gc();
  }

  std::vector<ASTNode *> nodes_;

 private:
};

class ASTTemplate : public ASTNode {
 public:
  ASTTemplate(ASTNode *type, const std::string &name)
      : type_(type), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Template: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    *(int *)out = sizeof(void *);  // templates always have a pointer size
  }

 private:
  ASTNode *type_;
  std::string name_;
};
class ASTLambda : public ASTNode {
 public:
  ASTLambda(ASTNodeList *args, ASTNode *body, ASTNode *type)
      : args_(args), body_(body), type_(type) {}

  virtual ~ASTLambda() { delete body_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Lambda: " << std::endl;
    args_->print(out, level + 1);
    body_->print(out, level + 1);
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    *(const ASTLambda **)out = this;
  }

  ASTNodeList *args_;
  ASTNode *body_;
  ASTNode *type_;

 private:
};

class ASTNativeFunction : public ASTNode {
 public:
  ASTNativeFunction(void (*function)(Runner *, RunnerStackFrame *)) : function_(function) {}

  virtual ~ASTNativeFunction() {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "native @ " << function_;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const { function_(runner, stackFrame); }

 private:
  void (*function_)(Runner *, RunnerStackFrame *);
};

Runner::Runner(ASTNode *ast) : ast_(ast) {
  this->stackFrame_ = new RunnerStackFrame();
}
void Runner::run(void *out) { ast_->run(this, this->stackFrame_, out); }
void *Runner::alloc(int size) { return malloc(size); }

void Runner::generateFunction(std::string name, ASTNodeList *args,
                              void (*body)(Runner *, RunnerStackFrame *), ASTNode *ret) {
  ASTLambda *newLambda =
      new ASTLambda(args, new ASTNodeList({new ASTNativeFunction(body)}), ret);
  ASTType *newTemplate = new ASTType("fun");
  void *ptr = this->stackFrame_->allocVariable(name, newTemplate, this);
  newLambda->run(this, stackFrame_, ptr);
}
void Runner::gc() {}

class ASTPointer : public ASTNode {
 public:
  ASTPointer(ASTNode *type) : type_(type) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Pointer: " << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    *(int *)out = sizeof(void *);
  }

 private:
  ASTNode *type_;
};
class ASTArray : public ASTNode {
 public:
  ASTArray(ASTNode *type, ASTNode *size) : type_(type), size_(size) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Array: " << std::endl;
    type_->print(out, level + 1);
    size_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    int sizeOutput, typeOutput;
    size_->run(runner, stackFrame, &sizeOutput);
    type_->run(runner, stackFrame, &typeOutput);
    *(int *)out = sizeOutput * typeOutput;
  }

 private:
  ASTNode *type_;
  ASTNode *size_;
};
class ASTFunctionArg : public ASTNode {
 public:
  ASTFunctionArg(ASTNode *type, const std::string &name)
      : type_(type), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Argument: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    void *ptr = stackFrame->allocVariable(name_, type_, runner);
    *(void **)out = ptr;
  }

 private:
  ASTNode *type_;
  std::string name_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(int value) : value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Number: " << value_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const { *(int *)out = value_; }

 private:
  int value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value) : value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "String: \"" << value_ << "\"" << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    *(const char **)out = value_.c_str();
  }

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

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "BinaryOp: " << op_ << std::endl;
    left_->print(out, level + 1);
    right_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    int leftOutput, rightOutput;
    left_->run(runner, stackFrame, &leftOutput);
    right_->run(runner, stackFrame, &rightOutput);
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

class ASTVariableDecl : public ASTNode {
 public:
  ASTVariableDecl(const std::string &name, ASTNode *type, ASTNode *value)
      : name_(name), type_(type), value_(value) {}

  virtual ~ASTVariableDecl() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "VariableDecl: " << name_ << std::endl;
    type_->print(out, level + 1);
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    void *ptr = stackFrame->allocVariable(name_, type_, runner);
    value_->run(runner, stackFrame, ptr);
  }

 private:
  std::string name_;
  ASTNode *type_;
  ASTNode *value_;
};
class ASTVariable : public ASTNode {
 public:
  ASTVariable(const std::string &name) : name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Variable: " << name_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    auto it = stackFrame->getVariable(name_);

    if (out != nullptr) {
      *(unsigned long *)out = (unsigned long)*(unsigned long int **)it;
    }
  }

 private:
  std::string name_;
};
class ASTVariableAssign : public ASTNode {
 public:
  ASTVariableAssign(const std::string &name, ASTNode *value)
      : name_(name), value_(value) {}

  virtual ~ASTVariableAssign() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "VariableAssign: " << name_ << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    auto it = stackFrame->getVariable(name_);
    value_->run(runner, stackFrame, it);
  }

 private:
  std::string name_;
  ASTNode *value_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(const std::string &name, ASTNodeList *args)
      : name_(name), args_(args) {}

  virtual ~ASTFunctionCall() { delete args_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "FunctionCall: " << name_ << std::endl;
    args_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    void *lambdaAddress = stackFrame->getVariable(name_);

    ASTLambda *lambda = *(ASTLambda **)lambdaAddress;


    RunnerStackFrame newStackFrame(stackFrame);
    int i = 0;
    for (auto arg : lambda->args_->nodes_) {
      void *variableAddress;
      arg->run(runner, &newStackFrame, &variableAddress);
      this->args_->nodes_[i]->run(runner, stackFrame, variableAddress);
      i++;
    }

    int bytes;
    lambda->type_->run(runner, &newStackFrame, &bytes);
    void *returnAddress = runner->alloc(bytes);

    lambda->body_->run(runner, &newStackFrame, returnAddress);

    if (out != nullptr) {
      *(unsigned long *)out = *(unsigned long *)returnAddress;
    }

    runner->_return = false;

    runner->gc();
  }

 private:
  std::string name_;
  ASTNodeList *args_;
};
class ASTIf : public ASTNode {
 public:
  ASTIf(ASTNode *condition, ASTNode *body, ASTNode *elseBody)
      : condition_(condition), body_(body), elseBody_(elseBody) {}

  virtual ~ASTIf() {
    delete condition_;
    delete body_;
    delete elseBody_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "If: " << std::endl;
    condition_->print(out, level + 1);
    body_->print(out, level + 1);
    if (elseBody_ != nullptr) {
      elseBody_->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    if (conditionOutput) {
      body_->run(runner, stackFrame, out);
    } else if (elseBody_ != nullptr) {
      elseBody_->run(runner, stackFrame, out);
    }
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
  ASTNode *elseBody_;
};
class ASTWhile : public ASTNode {
 public:
  ASTWhile(ASTNode *condition, ASTNode *body)
      : condition_(condition), body_(body) {}

  virtual ~ASTWhile() {
    delete condition_;
    delete body_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "While: " << std::endl;
    condition_->print(out, level + 1);
    body_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    while (conditionOutput) {
      body_->run(runner, stackFrame, out);
      condition_->run(runner, stackFrame, &conditionOutput);
    }
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
};
class ASTReturn : public ASTNode {
 public:
  ASTReturn(ASTNode *value) : value_(value) {}

  virtual ~ASTReturn() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Return: " << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame, void *out) const {
    value_->run(runner, stackFrame, out);
    runner->_return = true;
  }

 private:
  ASTNode *value_;
};

class Parser {
 public:
  Parser(std::istream &in) : in_(in) {}

  ASTNode *parse(bool standalone = true) {
    ASTNodeList *result = parseGlobalBody();
    if (in_.peek() != EOF) {
      throw std::runtime_error("Unexpected token : EOF expected");
    }
    if (standalone) {
      result->add(
          new ASTReturn(new ASTFunctionCall("main", new ASTNodeList())));
    }
    return result;
  }

 private:
  void skipComment() {
    while (in_.peek() != '\n' && in_.peek() != EOF) {
      in_.get();
    }
  }
  void skipWhitespace() {
    while (isspace(in_.peek()) || in_.peek() == '#') {
      char c_ = in_.get();
      if (c_ == '#') {
        skipComment();
      }
    }
  }
  bool isValidIdentifierChar(char i, int pos = 0) {
    if (pos == 0) {
      return isalpha(i) || i == '_';
    } else {
      return isalnum(i) || i == '_';
    }
  }
  bool isValidOperatorChar(char i, int pos = 0,
                           bool shouldIncludeSemicolon = false) {
    return i == '+' || i == '-' || i == '*' || i == '/' || i == '%' ||
           i == '^' || i == '&' || i == '|' || i == '!' || i == '~' ||
           i == '<' || i == '>' || i == '=' || i == '?' || i == ':' ||
           (shouldIncludeSemicolon && i == ';');
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
  std::string parseOperator(bool shouldIncludeSemicolon = false) {
    skipWhitespace();
    std::string result;
    int _p = 0;
    while (isValidOperatorChar(in_.peek(), _p, shouldIncludeSemicolon)) {
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
    skipWhitespace();
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

      std::string line = "";
      while (in_.peek() != ';') {
        line += in_.get();
      }
      in_.seekg(oldpos);

      enum {
        VAR_DECLARATION = 1,
        VAR_ASSIGNMENT = 2,
        EXPRESSION = 3,
      } expressionType = EXPRESSION;

      try {
        parseType();
        skipWhitespace();
        std::string identifier = parseIdentifier();
        skipWhitespace();
        std::string nextOperator = parseOperator();
        if ((nextOperator == "=" || nextOperator == ";") && identifier != "") {
          expressionType = VAR_DECLARATION;
        }
      } catch (std::runtime_error &e) {
      }

      in_.seekg(oldpos);

      if (expressionType == EXPRESSION) {
        try {
          parseIdentifier();
          skipWhitespace();
          std::string nextOperator = parseOperator();
          if (nextOperator == "=") {
            expressionType = VAR_ASSIGNMENT;
          }
        } catch (std::runtime_error &e) {
        }
      }

      in_.seekg(oldpos);

      if (expressionType == VAR_DECLARATION) {
        result->add(parseVariableDecl());
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after variable declaration");
        }
      } else if (expressionType == VAR_ASSIGNMENT) {
        std::string name = parseIdentifier();
        if (in_.get() != '=') {
          throw std::runtime_error("Expected '='");
        }
        skipWhitespace();
        ASTNode *value = parseExpression();
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after variable assignment");
        }
        result->add(new ASTVariableAssign(name, value));
      } else {
        ASTNode *value = parseExpression();
        result->add(value);
        if (in_.get() != ';') {
          throw std::runtime_error("Expected ';' after expression");
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
    ASTNodeList *body = parseBody();
    if (in_.get() != '}') {
      throw std::runtime_error("Expected '}'");
    }
    skipWhitespace();
    if (in_.peek() == '-') {
      std::string op = parseOperator();
      if (op == "->") {
        skipWhitespace();
        ASTNode *type = parseType();
        return new ASTLambda(args, body, type);
      }
    }
    body->add(new ASTReturn(new ASTNumber(0)));
    return new ASTLambda(args, body, new ASTType("void"));
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
      // special identifiers
      if (name == "if") {
        ASTNode *condition = parseExpression();
        skipWhitespace();
        if (in_.get() != '{') {
          throw std::runtime_error("Expected '{'");
        }
        ASTNode *ifBody = parseBody();
        skipWhitespace();
        if (in_.get() != '}') {
          throw std::runtime_error("Expected '}'");
        }
        skipWhitespace();
        ASTNode *elseBody = nullptr;
        if (in_.peek() == 'e') {
          std::string elseName = parseIdentifier();
          if (elseName != "else") {
            throw std::runtime_error("Expected 'else'");
          }
          skipWhitespace();
          if (in_.get() != '{') {
            throw std::runtime_error("Expected '{'");
          }
          skipWhitespace();
          elseBody = parseBody();
          if (in_.get() != '}') {
            throw std::runtime_error("Expected '}'");
          }
          skipWhitespace();
        }
        result = new ASTIf(condition, ifBody, elseBody);
      } else if (name == "while") {
        ASTNode *condition = parseExpression();
        skipWhitespace();
        if (in_.get() != '{') {
          throw std::runtime_error("Expected '{'");
        }
        skipWhitespace();
        ASTNode *body = parseBody();
        if (in_.get() != '}') {
          throw std::runtime_error("Expected '}'");
        }
        skipWhitespace();
        result = new ASTWhile(condition, body);
      } else if (name == "return" || name == "ret") {
        ASTNode *value = parseExpression();
        result = new ASTReturn(value);
      } else {
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
      }
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
      throw std::runtime_error("Unexpected token: ");
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

  int exitCode = 0;

  Runner runner(ast);

  runner.generateFunction("print",
                          new ASTNodeList({new ASTFunctionArg(
                              new ASTPointer(new ASTType("char")), "str")}),
                          [](Runner *runner, RunnerStackFrame *stackFrame) {
                            char *str = stackFrame->getVariable<char *>("str");

                            std::cout << str << std::endl;
                          });
  runner.generateFunction(
      "printintl",
      new ASTNodeList(
          {new ASTFunctionArg(new ASTPointer(new ASTType("char")), "str"),
           new ASTFunctionArg(new ASTPointer(new ASTType("i32")), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        char *str = stackFrame->getVariable<char *>("str");
        int number = stackFrame->getVariable<int>("number");

        std::cout << str << number << std::endl;
      });
  runner.generateFunction("printint",
                          new ASTNodeList({new ASTFunctionArg(
                              new ASTPointer(new ASTType("i32")), "number")}),
                          [](Runner *runner, RunnerStackFrame *stackFrame) {
                            int number = stackFrame->getVariable<int>("number");

                            std::cout << number << std::endl;
                          });

  runner.run(&exitCode);

  return exitCode;
}