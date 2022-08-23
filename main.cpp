#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

enum class ValueType {
  i32,
  i64,
  double_,
  float_,
  char_,
  void_,
  fun,
  bool_,
  pointer,
  array
};
ValueType parseType(std::string type) {
  type = type.substr(0, type.find(':'));
  if (type == "i32") {
    return ValueType::i32;
  } else if (type == "i64") {
    return ValueType::i64;
  } else if (type == "double") {
    return ValueType::double_;
  } else if (type == "float") {
    return ValueType::float_;
  } else if (type == "char") {
    return ValueType::char_;
  } else if (type == "void") {
    return ValueType::void_;
  } else if (type == "fun") {
    return ValueType::fun;
  } else if (type == "bool") {
    return ValueType::bool_;
  } else if (type == "pointer") {
    return ValueType::pointer;
  } else if (type == "array") {
    return ValueType::array;
  } else {
    throw std::runtime_error("Unknown type: " + type);
  }
}

class ASTNodeList;
class Runner;
class RunnerStackFrame;

class ASTNode {
 public:
  ASTNode(unsigned long pos) : pos_(pos) {}
  virtual ~ASTNode() {}
  virtual void print(std::ostream &out, int level = 0) const = 0;
  std::string indent(int level) const {
    std::string s;
    for (int i = 0; i < level; ++i) {
      s += "  ";
    }
    return s;
  }
  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out = nullptr) const = 0;

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const = 0;
  unsigned long pos_ = 0;
};
class ASTType : public ASTNode {
 public:
  ASTType(unsigned long pos) : ASTNode(pos) {}
};

class ASTBaseType : public ASTType {
 public:
  ASTBaseType(const std::string &name, unsigned long pos = 0)
      : ASTType(pos), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Type: " << name_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    ValueType type = parseType(name_);
    switch (type) {
      case ValueType::i32:
        *(int *)out = sizeof(int);
        break;
      case ValueType::i64:
        *(int *)out = sizeof(long int);
        break;
      case ValueType::fun:
        *(int *)out = sizeof(void *);
        break;
      case ValueType::float_:
        *(int *)out = sizeof(float);
        break;
      case ValueType::double_:
        *(int *)out = sizeof(double);
        break;
      case ValueType::void_:
        *(int *)out = sizeof(char);
        break;
      case ValueType::bool_:
        *(int *)out = sizeof(bool);
        break;
      case ValueType::char_:
        *(int *)out = sizeof(char);
        break;
      case ValueType::pointer:
        *(int *)out = sizeof(void *);
        break;
      default:
        throw std::runtime_error("Unknown type: " + name_);
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return name_;
  }

 private:
  std::string name_;
};

class Runner {
 public:
  bool _return = false;
  ASTNode *error = nullptr;
  Runner(ASTNode *ast, std::istream &code);

  void run(void *out);
  void *alloc(int size);

  void generateFunction(std::string name, ASTNodeList *args,
                        void (*body)(Runner *, RunnerStackFrame *),
                        ASTType *ret = new ASTBaseType("void"));

  void gc();

  void enterFrame();
  void exitFrame();

  void errorAt(const ASTNode *node, std::string msg);

 private:
  ASTNode *ast_;
  RunnerStackFrame *stackFrame_;
  std::istream &code_;
};

class RunnerStackFrame {
 public:
  RunnerStackFrame(RunnerStackFrame *parent = nullptr) : parent_(parent) {}
  ~RunnerStackFrame() {
    for (auto &kv : variables_) {
      free(kv.second);
    }
  }

  class Variable {
   public:
    Variable(void *ptr, ASTType *type) : ptr_(ptr), type_(type) {}
    void *ptr_;
    ASTType *type_;
  };

  Variable *getVariable(const std::string &name) {
    if (this->variables_[name] != nullptr) return this->variables_[name];
    if (this->parent_ != nullptr) return this->parent_->getVariable(name);
    throw std::runtime_error("Variable not found: " + name);
  };
  template <typename T>
  T getVariable(const std::string &name) {
    return *(T *)this->getVariable(name)->ptr_;
  };
  void *allocVariable(const std::string &name, ASTType *type, Runner *runner) {
    int size;
    type->run(runner, this, &size);
    void *value = runner->alloc(size);
    this->variables_[name] = new Variable(value, type);
    return value;
  };

  RunnerStackFrame *parent_;
  std::map<std::string, Variable *> variables_;
};

class ASTNodeList : public ASTNode {
 public:
  ASTNodeList(const std::vector<ASTNode *> &nodes, unsigned long pos = 0)
      : ASTNode(pos), nodes_(nodes) {}
  ASTNodeList(unsigned long pos = 0) : ASTNode(pos) {}

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

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
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

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

  std::vector<ASTNode *> nodes_;

 private:
};

class ASTTemplate : public ASTType {
 public:
  ASTTemplate(ASTType *type, const std::string &name, unsigned long pos = 0)
      : ASTType(pos), type_(type), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Template: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(int *)out = sizeof(void *);  // templates always have a pointer size
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "template-" + name_ + ":" + type_->returnType(runner, stack);
  }

 private:
  ASTType *type_;
  std::string name_;
};
class ASTLambda : public ASTNode {
 public:
  ASTLambda(ASTNodeList *args, ASTNode *body, ASTType *type,
            unsigned long pos = 0)
      : ASTNode(pos), args_(args), body_(body), type_(type) {}

  virtual ~ASTLambda() { delete body_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Lambda: " << std::endl;
    args_->print(out, level + 1);
    body_->print(out, level + 1);
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(const ASTLambda **)out = this;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "fun";
  }

  ASTNodeList *args_;
  ASTNode *body_;
  ASTType *type_;

 private:
};

class ASTNativeFunction : public ASTNode {
 public:
  ASTNativeFunction(void (*function)(Runner *, RunnerStackFrame *),
                    unsigned long pos = 0)
      : ASTNode(pos), function_(function) {}

  virtual ~ASTNativeFunction() {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "native @ " << function_;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    function_(runner, stackFrame);
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  void (*function_)(Runner *, RunnerStackFrame *);
};

void errorWithMessage(const std::string &msg, std::istream &code_,
                      unsigned long pos) {
  int line = 1;
  int column = 0;
  std::string lineString;
  code_.clear();
  code_.seekg(0);
  for (unsigned long i = 0; i < pos; i++) {
    if (code_.peek() == EOF) {
      throw std::runtime_error("Error at end of file: " + msg);
    }
    if (code_.peek() == '\n') {
      line++;
      column = 0;
    } else {
      column++;
    }
    code_.get();
  }
  code_.clear();
  code_.seekg(0);
  for (int i = 0; i < line; i++) {
    std::getline(code_, lineString);
  }

  std::string lineNoString = std::to_string(line);

  std::cerr << "Error at " << line << ":" << column << "\t" << msg << "\n\n"
            << std::endl;
  std::cerr << std::string(5 - std::min<int>(5, lineNoString.length()), ' ') +
                   lineNoString
            << " | " << lineString << std::endl;
  std::cerr << "      | " << std::string(std::max(column - 1, 0), '~') << "^"
            << "\n\n"
            << std::endl;
  exit(1);
}

Runner::Runner(ASTNode *ast, std::istream &code) : ast_(ast), code_(code) {
  code_.clear();
  code_.seekg(0);
  this->stackFrame_ = new RunnerStackFrame();
}
void Runner::run(void *out) { ast_->run(this, this->stackFrame_, out); }
void *Runner::alloc(int size) { return malloc(size); }

void Runner::generateFunction(std::string name, ASTNodeList *args,
                              void (*body)(Runner *, RunnerStackFrame *),
                              ASTType *ret) {
  ASTLambda *newLambda =
      new ASTLambda(args, new ASTNodeList({new ASTNativeFunction(body)}), ret);
  ASTType *newTemplate = new ASTBaseType("fun");
  void *ptr = this->stackFrame_->allocVariable(name, newTemplate, this);
  newLambda->run(this, stackFrame_, ptr);
}
void Runner::gc() {}
void Runner::errorAt(const ASTNode *node, std::string message) {
  errorWithMessage(message, code_, node->pos_);
}

class ASTPointer : public ASTType {
 public:
  ASTPointer(ASTType *type, unsigned long pos = 0)
      : ASTType(pos), type_(type) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Pointer: " << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(int *)out = sizeof(void *);
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:" + type_->returnType(runner, stack);
  }

 private:
  ASTType *type_;
};
class ASTArray : public ASTType {
 public:
  ASTArray(ASTType *type, ASTNode *size, unsigned long pos = 0)
      : ASTType(pos), type_(type), size_(size) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Array: " << std::endl;
    type_->print(out, level + 1);
    size_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int sizeOutput, typeOutput;
    size_->run(runner, stackFrame, &sizeOutput);
    type_->run(runner, stackFrame, &typeOutput);
    *(int *)out = sizeOutput * typeOutput;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "array:" + type_->returnType(runner, stack);
  }

 private:
  ASTType *type_;
  ASTNode *size_;
};
class ASTFunctionArg : public ASTNode {
 public:
  ASTFunctionArg(ASTType *type, const std::string &name, unsigned long pos = 0)
      : ASTNode(pos), name_(name), type_(type) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Argument: " << name_ << std::endl;
    type_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
    *(void **)out = ptr;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return type_->returnType(runner, stack);
  }
  std::string name_;

 private:
  ASTType *type_;
};

class ASTNumber : public ASTNode {
 public:
  ASTNumber(long int value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Number: " << value_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(long int *)out = value_;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "i64";
  }

 private:
  long int value_;
};
class ASTDouble : public ASTNode {
 public:
  ASTDouble(double value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Double: " << value_ << std::endl;
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "double";
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(double *)out = value_;
  }

 private:
  double value_;
};
class ASTString : public ASTNode {
 public:
  ASTString(const std::string &value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "String: \"" << value_ << "\"" << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    *(const char **)out = value_.c_str();
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:char";
  }

 private:
  std::string value_;
};

class ASTBinaryOp : public ASTNode {
 public:
  ASTBinaryOp(std::string op, ASTNode *left, ASTNode *right,
              unsigned long pos = 0)
      : ASTNode(pos), op_(op), left_(left), right_(right) {}

  virtual ~ASTBinaryOp() {
    delete left_;
    delete right_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "BinaryOp: " << op_ << std::endl;
    left_->print(out, level + 1);
    right_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int leftOutput, rightOutput;
    left_->run(runner, stackFrame, &leftOutput);
    right_->run(runner, stackFrame, &rightOutput);

    double leftOutputDouble, rightOutputDouble;
    left_->run(runner, stackFrame, &leftOutputDouble);
    right_->run(runner, stackFrame, &rightOutputDouble);

    if (op_ == "+") {
      *(int *)out = leftOutput + rightOutput;
    } else if (op_ == "-") {
      *(int *)out = leftOutput - rightOutput;
    } else if (op_ == "*") {
      *(int *)out = leftOutput * rightOutput;
    } else if (op_ == "/") {
      *(double *)out = leftOutputDouble / rightOutputDouble;
    } else if (op_ == "//") {
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
      runner->errorAt(this, "Unknown binary operator: " + op_);
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return op_ == "/" ? "double" : "i64";
  }

 private:
  std::string op_;
  ASTNode *left_;
  ASTNode *right_;
};

class ASTCast : public ASTNode {
 public:
  ASTCast(ASTNode *value, ASTType *type, unsigned long pos = 0)
      : ASTNode(pos), value_(value), type_(type) {}

  virtual ~ASTCast() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Cast: " << std::endl;
    type_->print(out, level + 1);
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    ValueType type = parseType(value_->returnType(runner, stackFrame));
    ValueType castType = parseType(type_->returnType(runner, stackFrame));

    if ((type == ValueType::i32 || type == ValueType::i64) &&
        castType == ValueType::double_) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(double *)out = (double)value;
    } else if (type == ValueType::double_ &&
               (castType == ValueType::i32 || castType == ValueType::i64)) {
      double value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = (long int)value;
    } else if (type == ValueType::i32 && castType == ValueType::i64) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = value;
    } else if (type == ValueType::i64 && castType == ValueType::i32) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = value;
    } else if (type == ValueType::pointer && castType == ValueType::i64) {
      void *value;
      value_->run(runner, stackFrame, &value);
      *(long int *)out = (long int)value;
    } else if (type == ValueType::i64 && castType == ValueType::pointer) {
      long int value;
      value_->run(runner, stackFrame, &value);
      *(void **)out = (void *)value;
    } else {
      runner->errorAt(
          this, "Unknown cast: " + value_->returnType(runner, stackFrame) +
                    " to " + type_->returnType(runner, stackFrame));
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return type_->returnType(runner, stack);
  }

 private:
  ASTNode *value_;
  ASTType *type_;
};

class ASTVariableDecl : public ASTNode {
 public:
  ASTVariableDecl(const std::string &name, ASTType *type, ASTNode *value,
                  unsigned long pos = 0)
      : ASTNode(pos), name_(name), type_(type), value_(value) {}

  virtual ~ASTVariableDecl() {
    delete type_;
    delete value_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "VariableDecl: " << name_ << std::endl;
    type_->print(out, level + 1);
    if (value_ != nullptr) {
      value_->print(out, level + 1);
    }
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (value_ != nullptr) {
      if (type_->returnType(runner, stackFrame) !=
          value_->returnType(runner, stackFrame)) {
        runner->errorAt(
            this, "Type mismatch: " + type_->returnType(runner, stackFrame) +
                      " != " + value_->returnType(runner, stackFrame));
      }
    }

    void *ptr = stackFrame->allocVariable(name_, (ASTType *)type_, runner);
    if (value_ != nullptr) {
      value_->run(runner, stackFrame, ptr);
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  std::string name_;
  ASTType *type_;
  ASTNode *value_;
};
class ASTVariable : public ASTNode {
 public:
  ASTVariable(const std::string &name, unsigned long pos = 0)
      : ASTNode(pos), name_(name) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Variable: " << name_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    auto it = stackFrame->getVariable(name_);

    if (out != nullptr) {
      *(unsigned long *)out = (unsigned long)*(unsigned long int **)(it->ptr_);
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return stack->getVariable(name_)->type_->returnType(runner, stack);
  }

 private:
  std::string name_;
};
class ASTVariableAssign : public ASTNode {
 public:
  ASTVariableAssign(const std::string &name, ASTNode *value,
                    unsigned long pos = 0)
      : ASTNode(pos), name_(name), value_(value) {}

  virtual ~ASTVariableAssign() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "VariableAssign: " << name_ << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    auto it = stackFrame->getVariable(name_)->ptr_;
    value_->run(runner, stackFrame, it);
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  std::string name_;
  ASTNode *value_;
};
class ASTFunctionCall : public ASTNode {
 public:
  ASTFunctionCall(ASTNode *pointer, ASTNodeList *args, unsigned long pos = 0)
      : ASTNode(pos), pointer_(pointer), args_(args) {}

  virtual ~ASTFunctionCall() { delete args_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "FunctionCall: " << std::endl;
    pointer_->print(out, level + 1);
    args_->print(out, level + 1);
  }

  ASTLambda *getLambda(Runner *runner, RunnerStackFrame *stackFrame) const {
    void *lambdaAddress;
    pointer_->run(runner, stackFrame, &lambdaAddress);

    ASTLambda *lambda = (ASTLambda *)lambdaAddress;
    return lambda;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    ASTLambda *lambda = getLambda(runner, stackFrame);
    RunnerStackFrame newStackFrame(stackFrame);
    int i = 0;
    if (lambda->args_->nodes_.size() != args_->nodes_.size()) {
      runner->errorAt(this, "Wrong number of arguments on function call: " +
                                std::to_string(lambda->args_->nodes_.size()) +
                                " expected, " +
                                std::to_string(args_->nodes_.size()) +
                                " given");
    }
    for (auto arg : lambda->args_->nodes_) {
      if (arg->returnType(runner, &newStackFrame) !=
          args_->nodes_[i]->returnType(runner, &newStackFrame)) {
        runner->errorAt(
            this, "Type mismatch on function argument" +
                      ((ASTFunctionArg *)arg)->name_ + ": " +
                      arg->returnType(runner, &newStackFrame) + " expected, " +
                      args_->nodes_[i]->returnType(runner, &newStackFrame) +
                      " given");
      }

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

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return getLambda(runner, stack)->type_->returnType(runner, stack);
  }

 private:
  ASTNode *pointer_;
  ASTNodeList *args_;
};
class ASTIf : public ASTNode {
 public:
  ASTIf(ASTNode *condition, ASTNode *body, ASTNode *elseBody,
        unsigned long pos = 0)
      : ASTNode(pos), condition_(condition), body_(body), elseBody_(elseBody) {}

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

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    if (conditionOutput) {
      body_->run(runner, stackFrame, out);
    } else if (elseBody_ != nullptr) {
      elseBody_->run(runner, stackFrame, out);
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
  ASTNode *elseBody_;
};
class ASTWhile : public ASTNode {
 public:
  ASTWhile(ASTNode *condition, ASTNode *body, unsigned long pos = 0)
      : ASTNode(pos), condition_(condition), body_(body) {}

  virtual ~ASTWhile() {
    delete condition_;
    delete body_;
  }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "While: " << std::endl;
    condition_->print(out, level + 1);
    body_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    int conditionOutput;
    condition_->run(runner, stackFrame, &conditionOutput);
    while (conditionOutput) {
      body_->run(runner, stackFrame, out);
      condition_->run(runner, stackFrame, &conditionOutput);
    }
  }

  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  ASTNode *condition_;
  ASTNode *body_;
};
class ASTReturn : public ASTNode {
 public:
  ASTReturn(ASTNode *value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual ~ASTReturn() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Return: " << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    value_->run(runner, stackFrame, out);
    runner->_return = true;
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "void";
  }

 private:
  ASTNode *value_;
};
class ASTRef : public ASTNode {
 public:
  ASTRef(std::string &variable, unsigned long pos = 0)
      : ASTNode(pos), variable_(variable) {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Ref: " << variable_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    void *variableAddress = stackFrame->getVariable(variable_)->ptr_;
    if (out != nullptr) {
      *(unsigned long **)out = (unsigned long *)variableAddress;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:" +
           stack->getVariable(variable_)->type_->returnType(runner, stack);
  }

 private:
  std::string variable_;
};
class ASTDeref : public ASTNode {
 public:
  ASTDeref(ASTNode *value, unsigned long pos = 0)
      : ASTNode(pos), value_(value) {}

  virtual ~ASTDeref() { delete value_; }

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Deref: " << std::endl;
    value_->print(out, level + 1);
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (value_->returnType(runner, stackFrame).substr(0, 8) != "pointer:") {
      runner->errorAt(this, "Cannot dereference non-pointer type");
    }

    void *value;
    value_->run(runner, stackFrame, &value);
    if (out != nullptr) {
      *(unsigned long *)out = *(unsigned long *)value;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    if (value_->returnType(runner, stack).substr(0, 8) == "pointer:") {
      return value_->returnType(runner, stack).substr(8);
    } else {
      runner->errorAt(this, "Cannot dereference non-pointer type");
      return "void";
    }
  }

 private:
  ASTNode *value_;
};
class ASTNull : public ASTNode {
 public:
  ASTNull(unsigned long pos = 0) : ASTNode(pos) {}

  virtual ~ASTNull() {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Null" << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (out != nullptr) {
      *(unsigned long *)out = 0;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "pointer:void";
  }

 private:
};
class ASTBool : public ASTNode {
 public:
  ASTBool(bool value, unsigned long pos = 0) : ASTNode(pos), value_(value) {}

  virtual ~ASTBool() {}

  virtual void print(std::ostream &out, int level) const {
    out << this->indent(level) << "Bool: " << value_ << std::endl;
  }

  virtual void run(Runner *runner, RunnerStackFrame *stackFrame,
                   void *out) const {
    if (out != nullptr) {
      *(bool *)out = value_;
    }
  }
  virtual const std::string returnType(Runner *runner,
                                       RunnerStackFrame *stack) const {
    return "bool";
  }

 private:
  bool value_;
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
      result->add(new ASTReturn(
          new ASTFunctionCall(new ASTVariable("main"), new ASTNodeList())));
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
    ASTType *type = parseType();
    std::string name = parseIdentifier();
    skipWhitespace();
    ASTNode *value;
    if (in_.peek() == '=') {
      in_.get();
      skipWhitespace();
      value = parseExpression();
      skipWhitespace();
    } else {
      value = nullptr;
    }
    return new ASTVariableDecl(name, type, value, in_.tellg());
  }

  ASTType *parseType() {
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
      ASTType *type = parseType();
      skipWhitespace();
      if (in_.peek() != '>') {
        throw std::runtime_error("Expected >");
      }
      in_.get();
      skipWhitespace();
      return new ASTTemplate(type, name, in_.tellg());
    }
    std::streampos newpos = in_.tellg();
    in_.seekg(oldpos);

    if (in_.peek() == '*') {
      in_.get();
      return new ASTPointer(parseType(), in_.tellg());
    } else if (in_.peek() == '[') {
      in_.get();
      if (in_.peek() == ']') {
        in_.get();
        return new ASTArray(parseType(), new ASTNumber(0, in_.tellg()),
                            in_.tellg());
      }
      ASTNode *size = parseExpression();
      if (in_.get() != ']') {
        throw std::runtime_error("Expected ']'");
      }
      return new ASTArray(parseType(), size, in_.tellg());
    }
    in_.seekg(newpos);
    return new ASTBaseType(name, in_.tellg());
  }

  ASTNodeList *parseGlobalBody() {
    auto result = parseBody(false);
    return result;
  }

  ASTNodeList *parseBody(bool errorOnEOF = true) {
    ASTNodeList *result = new ASTNodeList(in_.tellg());
    skipWhitespace();
    while (in_.peek() != '}') {
      skipWhitespace();
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
        delete parseType();
        skipWhitespace();
        std::string identifier = parseIdentifier();
        skipWhitespace();
        std::string nextOperator = parseOperator();
        if ((nextOperator == "=" || in_.peek() == ';') && identifier != "") {
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
        result->add(new ASTVariableAssign(name, value, in_.tellg()));
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
    ASTType *type = parseType();
    skipWhitespace();
    return new ASTFunctionArg(type, name, in_.tellg());
  }

  ASTNodeList *parseFunctionArgs() {
    ASTNodeList *result = new ASTNodeList(in_.tellg());
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
        ASTType *type = parseType();
        return new ASTLambda(args, body, type, in_.tellg());
      }
    }
    body->add(new ASTReturn(new ASTNumber(0, in_.tellg()), in_.tellg()));
    return new ASTLambda(args, body, new ASTBaseType("void", in_.tellg()),
                         in_.tellg());
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
        result = new ASTIf(condition, ifBody, elseBody, in_.tellg());
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
        result = new ASTWhile(condition, body, in_.tellg());
      } else if (name == "return" || name == "ret") {
        ASTNode *value = parseExpression();
        result = new ASTReturn(value, in_.tellg());
      } else if (name == "ref") {
        skipWhitespace();
        std::string value = parseIdentifier();
        result = new ASTRef(value, in_.tellg());
      } else if (name == "deref") {
        ASTNode *value = parseExpression();
        result = new ASTDeref(value, in_.tellg());
      } else if (name == "null") {
        result = new ASTNull(in_.tellg());
      } else if (name == "true") {
        result = new ASTBool(true, in_.tellg());
      } else if (name == "false") {
        result = new ASTBool(false, in_.tellg());
      } else {
        result = new ASTVariable(name, in_.tellg());
        skipWhitespace();
      }
    } else {
      result = parseTerm();
    }
    skipWhitespace();
    if (in_.peek() == '(') {
      in_.get();
      ASTNodeList *args = new ASTNodeList(in_.tellg());
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

      result = new ASTFunctionCall(result, args, in_.tellg());
    }
    skipWhitespace();
    if (in_.peek() == '-') {
      std::streampos oldpos = in_.tellg();
      std::string op = parseOperator();
      if (op == "->") {
        skipWhitespace();
        ASTType *type = parseType();
        result = new ASTCast(result, type, in_.tellg());
      } else {
        in_.seekg(oldpos);
      }
    }
    skipWhitespace();
    while (isValidOperatorChar(in_.peek())) {
      std::streampos oldpos = in_.tellg();
      std::string op = parseOperator();
      if (op != "=") {
        skipWhitespace();
        auto right = parseExpression();
        result = new ASTBinaryOp(op, result, right, in_.tellg());
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
      double value = 0;
      unsigned char isFloat = 0;
      while (isdigit(in_.peek()) || in_.peek() == '.') {
        if (in_.peek() == '.' && !isFloat) {
          isFloat = 1;
          in_.get();
        } else {
          if (isFloat) {
            value += (in_.get() - '0') / std::pow(10.0, isFloat);
            isFloat++;
          } else {
            value *= 10;
            value += in_.get() - '0';
          }
        }
      }
      if (isFloat) {
        return new ASTDouble(value, in_.tellg());
      } else {
        return new ASTNumber(value, in_.tellg());
      }
    } else if (in_.peek() == '"') {
      in_.get();
      std::string value;
      while (in_.peek() != '"') {
        value += in_.get();
      }
      in_.get();
      return new ASTString(value, in_.tellg());
    } else {
      throw std::runtime_error("Unexpected token: ");
    }
  }

  std::istream &in_;
};

int main() {
  std::cout.precision(std::numeric_limits<double>::max_digits10);

  std::ifstream file("tests/initial.wha");
  std::istream &code = static_cast<std::istream &>(file);

  Parser parser(code);

  ASTNode *ast;

  try {
    ast = parser.parse();
  } catch (std::runtime_error &e) {
    errorWithMessage(e.what(), code, (int)code.tellg() - 2);
  }

  ast->print(std::cout);

  int exitCode = 0;

  Runner runner(ast, code);

  runner.generateFunction("print",
                          new ASTNodeList({new ASTFunctionArg(
                              new ASTPointer(new ASTBaseType("char")), "str")}),
                          [](Runner *runner, RunnerStackFrame *stackFrame) {
                            char *str = stackFrame->getVariable<char *>("str");

                            std::cout << str << std::endl;
                          });
  runner.generateFunction(
      "printintl",
      new ASTNodeList(
          {new ASTFunctionArg(new ASTPointer(new ASTBaseType("char")), "str"),
           new ASTFunctionArg(new ASTBaseType("i64"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        char *str = stackFrame->getVariable<char *>("str");
        int number = stackFrame->getVariable<int>("number");

        std::cout << str << number << std::endl;
      });
  runner.generateFunction(
      "printint",
      new ASTNodeList({new ASTFunctionArg(new ASTBaseType("i64"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        int number = stackFrame->getVariable<int>("number");

        std::cout << number << std::endl;
      });
  runner.generateFunction(
      "printdouble",
      new ASTNodeList(
          {new ASTFunctionArg(new ASTBaseType("double"), "number")}),
      [](Runner *runner, RunnerStackFrame *stackFrame) {
        double number = stackFrame->getVariable<double>("number");

        std::cout << number << std::endl;
      });

  runner.run(&exitCode);

  return exitCode;
}